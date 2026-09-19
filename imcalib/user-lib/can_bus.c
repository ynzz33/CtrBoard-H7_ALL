#include "can_bus.h"
#include "mono_ns.h"

static can_bus_t can_bus[CAN_BUS_NUM];

static FDCAN_HandleTypeDef *handles[CAN_BUS_NUM] = { &hfdcan1, &hfdcan2, &hfdcan3 };

/* TX 完成环形缓冲 (ISR 写 / 任务读) */
static can_tx_done_t tx_ring_buf[CAN_BUS_NUM][CAN_TX_RING_CAP];

/* 按总线号取句柄 */
FDCAN_HandleTypeDef *Can_Bus_Handle(uint8_t bus)
{
    if (bus < 1u || bus > CAN_BUS_NUM)
    {
        return NULL;
    }
    return handles[bus - 1u];
}

/* 收帧计数 */
uint32_t Can_Bus_Rx_Count(uint8_t bus)
{
    if (bus < 1u || bus > CAN_BUS_NUM)
    {
        return 0u;
    }
    return can_bus[bus - 1u].alive_cnt;
}

/* 最近一帧的 ID */
uint32_t Can_Bus_Last_Rx_Id(uint8_t bus)
{
    if (bus < 1u || bus > CAN_BUS_NUM)
    {
        return 0u;
    }
    return can_bus[bus - 1u].last_rx_id;
}

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
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0xFFFFFFFF);
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

        bus->tx_complete_cnt = 0;
        bus->tx_ring_w       = 0;
        bus->tx_ring_r       = 0;
        bus->tx_drop_cnt     = 0;
        for (uint32_t j = 0; j < CAN_TX_PENDING_MAX; j++)
        {
            bus->tx_pending[j].kind = 0;
            bus->tx_pending[j].seq  = 0;
        }
        for (uint32_t j = 0; j < CAN_TX_RING_CAP; j++)
        {
            tx_ring_buf[i][j].valid = 0;
        }

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

/* 发送 (兼容旧接口) */
HAL_StatusTypeDef Can_Bus_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const uint8_t *data, uint8_t len)
{
    return Can_Bus_Transmit_Tagged(hfdcan, can_id, data, len, 0, 0);
}

/* 发送 (带标签) */
HAL_StatusTypeDef Can_Bus_Transmit_Tagged(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                          const uint8_t *data, uint8_t len,
                                          uint8_t kind, uint16_t seq)
{
    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0 || len > 8) return HAL_ERROR;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
    {
        return HAL_ERROR;
    }

    FDCAN_TxHeaderTypeDef tx_header = can_bus[idx].tx_template;
    tx_header.Identifier = can_id;
    tx_header.DataLength = len;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)data);
    if (st == HAL_OK)
    {
        can_bus[idx].last_tx_tick = HAL_GetTick();

        uint32_t buf = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hfdcan);
        if (buf != 0u)
        {
            uint32_t bi = 31u - (uint32_t)__CLZ(buf);
            can_bus[idx].tx_pending[bi].kind  = kind;
            can_bus[idx].tx_pending[bi].seq   = seq;
            can_bus[idx].tx_pending[bi].valid = 1u;
        }
    }
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
        bus->last_rx_id = rx_header.Identifier;

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

/* ISR: TX 完成回调 */
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0) return;

    can_bus_t *bus = &can_bus[idx];
    uint64_t ns = Mono_Ns_Get();

    while (BufferIndexes != 0u)
    {
        uint32_t bi  = 31u - (uint32_t)__CLZ(BufferIndexes);
        uint32_t bit = 1u << bi;

        /* 只认本机登记过的元素, 其余位是硬件残留 */
        if (bus->tx_pending[bi].valid == 0u)
        {
            BufferIndexes &= ~bit;
            continue;
        }

        bus->tx_complete_cnt++;

        can_tx_done_t *e = &tx_ring_buf[idx][bus->tx_ring_w % CAN_TX_RING_CAP];
        e->kind  = bus->tx_pending[bi].kind;
        e->seq   = bus->tx_pending[bi].seq;
        e->tx_ns = ns;
        e->valid = 1;
        __DMB();

        uint32_t next_w = bus->tx_ring_w + 1u;
        if (next_w - bus->tx_ring_r > CAN_TX_RING_CAP)
        {
            bus->tx_drop_cnt++;
            bus->tx_ring_r++;
        }
        bus->tx_ring_w = next_w;

        bus->tx_pending[bi].kind  = 0;
        bus->tx_pending[bi].seq   = 0;
        bus->tx_pending[bi].valid = 0u;

        BufferIndexes &= ~bit;
    }
}

/* TX 完成事件出队 */
bool Can_Bus_Tx_Pop(uint8_t bus, uint8_t *kind, uint16_t *seq, uint64_t *tx_ns)
{
    if (bus < 1u || bus > CAN_BUS_NUM) return false;

    can_bus_t *b = &can_bus[bus - 1u];

    __DMB();
    if (b->tx_ring_r == b->tx_ring_w) return false;

    can_tx_done_t *e = &tx_ring_buf[bus - 1u][b->tx_ring_r % CAN_TX_RING_CAP];
    *kind  = e->kind;
    *seq   = e->seq;
    *tx_ns = e->tx_ns;
    __DMB();

    b->tx_ring_r++;
    return true;
}

/* TX 完成帧计数 */
uint32_t Can_Bus_Tx_Complete_Count(uint8_t bus)
{
    if (bus < 1u || bus > CAN_BUS_NUM) return 0u;
    return can_bus[bus - 1u].tx_complete_cnt;
}

/* TX 丢弃帧计数 */
uint32_t Can_Bus_Tx_Drop_Count(uint8_t bus)
{
    if (bus < 1u || bus > CAN_BUS_NUM) return 0u;
    return can_bus[bus - 1u].tx_drop_cnt;
}

/* 看门狗 */
bool Can_Bus_Online(bool expect_traffic)
{
    uint32_t now = HAL_GetTick();
    bool all_ok = true;

    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        can_bus_t *bus = &can_bus[i];

        /* 本机没在这条总线登记设备 → 不要求流量 */
        if (bus->route_cnt == 0u)
        {
            bus->alive_prev = bus->alive_cnt;
            bus->dead_since = now;
            continue;
        }

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
