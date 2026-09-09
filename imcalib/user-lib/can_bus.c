#include "can_bus.h"

static can_bus_t can_bus[CAN_BUS_NUM];

static FDCAN_HandleTypeDef *handles[CAN_BUS_NUM] = { &hfdcan1, &hfdcan2, &hfdcan3 };

/* 句柄指针→下标 */
static int32_t Can_Bus_Hw_Index(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return 0;
    if (hfdcan == &hfdcan2) return 1;
    if (hfdcan == &hfdcan3) return 2;
    return -1;
}

/* 取消TX + 停止 */
static void Can_Bus_Stop(uint32_t idx)
{
    FDCAN_HandleTypeDef *hfdcan = handles[idx];
    SET_BIT(hfdcan->Instance->TXBCR, hfdcan->Instance->TXBRP);
    HAL_FDCAN_Stop(hfdcan);
}

/* 配置滤波器 + 启动 + 开中断 */
static void Can_Bus_Start(uint32_t idx)
{
    FDCAN_HandleTypeDef *hfdcan = handles[idx];

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;
    filter.FilterIndex = 0;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
}

/* 初始化 */
void Can_Bus_Init(void)
{
    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        can_bus_t *bus = &can_bus[i];

        bus->tx_template.Identifier          = 0;
        bus->tx_template.IdType              = FDCAN_STANDARD_ID;
        bus->tx_template.TxFrameType         = FDCAN_DATA_FRAME;
        bus->tx_template.DataLength          = FDCAN_DLC_BYTES_8;
        bus->tx_template.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        bus->tx_template.BitRateSwitch       = FDCAN_BRS_OFF;
        bus->tx_template.FDFormat            = FDCAN_CLASSIC_CAN;
        bus->tx_template.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        bus->tx_template.MessageMarker       = 0;

        bus->state        = CAN_BUS_STATE_ACTIVE;
        bus->route_cnt    = 0;
        bus->alive_cnt    = 0;
        bus->alive_prev   = 0;
        bus->last_tx_tick = HAL_GetTick();
        bus->dead_since   = HAL_GetTick();
        bus->reinit_tick  = HAL_GetTick();

        Can_Bus_Start(i);
    }
}

/* 路由注册 */
bool Can_Bus_Register(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                      can_parse_fn_t fn, void *ctx)
{
    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0 || fn == NULL) return false;

    can_bus_t *bus = &can_bus[idx];
    if (bus->route_cnt >= CAN_BUS_ROUTE_MAX) return false;

    for (uint32_t i = 0; i < bus->route_cnt; i++)
        if (bus->route[i].can_id == can_id) return false;

    can_route_t *r = &bus->route[bus->route_cnt++];
    r->can_id = can_id;
    r->parse  = fn;
    r->ctx    = ctx;
    return true;
}

/* 发送 */
HAL_StatusTypeDef Can_Bus_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const uint8_t *data, uint8_t len)
{
    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0 || len > 8) return HAL_ERROR;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        return HAL_ERROR;

    FDCAN_TxHeaderTypeDef tx_header = can_bus[idx].tx_template;
    tx_header.Identifier = can_id;
    tx_header.DataLength = len;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)data);
    if (st == HAL_OK)
        can_bus[idx].last_tx_tick = HAL_GetTick();
    return st;
}

/* ISR 接收 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0) return;

    can_bus_t *bus = &can_bus[idx];
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
            break;

        bus->alive_cnt++;

        for (uint32_t i = 0; i < bus->route_cnt; i++)
        {
            if (bus->route[i].can_id == rx_header.Identifier)
            {
                bus->route[i].parse(bus->route[i].ctx,
                                    rx_header.Identifier, data,
                                    (uint8_t)rx_header.DataLength);
                break;
            }
        }
    }
}

/* ISR 错误: bus-off → 停止 + 标记DEAD */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (!(ErrorStatusITs & FDCAN_IT_BUS_OFF)) return;

    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0) return;

    Can_Bus_Stop(idx);
    can_bus[idx].state = CAN_BUS_STATE_DEAD;
}

/* 看门狗 */
bool Can_Bus_Online(bool expect_traffic)
{
    uint32_t now = HAL_GetTick();
    bool all_ok = true;

    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        can_bus_t *bus = &can_bus[i];
        bool alive_changed = (bus->alive_cnt != bus->alive_prev);

        /* DEAD + 收到新帧 → 恢复 */
        if (bus->state == CAN_BUS_STATE_DEAD && alive_changed)
            bus->state = CAN_BUS_STATE_ACTIVE;

        /* DEAD → 每100ms重新启动 */
        if (bus->state == CAN_BUS_STATE_DEAD
            && now - bus->reinit_tick >= CAN_BUS_REINIT_MS)
        {
            Can_Bus_Stop(i);
            Can_Bus_Start(i);
            bus->reinit_tick = now;
        }

        if (!expect_traffic)
        {
            bus->alive_prev = bus->alive_cnt;
            bus->dead_since = now;
            if (bus->state == CAN_BUS_STATE_DEAD)
                all_ok = false;
            continue;
        }

        /* RX 看门狗 */
        if (alive_changed)
        {
            bus->alive_prev = bus->alive_cnt;
            bus->dead_since = now;
        }
        else if (now - bus->dead_since > CAN_BUS_DEAD_MS)
        {
            if (bus->state == CAN_BUS_STATE_ACTIVE)
                bus->state = CAN_BUS_STATE_DEAD;
        }

        if (bus->state == CAN_BUS_STATE_DEAD)
            all_ok = false;
    }
    return all_ok;
}
