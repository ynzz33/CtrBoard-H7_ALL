/*
 * 文件职责：FDCAN公共注册、接收分发与发送服务（can_bus.c）。
 * 输入/来源：CubeMX FDCAN句柄、注册ID及驱动收发请求。
 * 输出/去向：HAL中断读取帧并派发协议回调；统一构造发送帧、统计接收和错误。
 * 关键边界：本层不解释DM/DJI物理协议或策略意图；HAL_OK仅表明发送入队调用成功。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "can_bus.h"
#include "FreeRTOS.h"
#include "task.h"

static can_bus_t can_bus[CAN_BUS_NUM];

static FDCAN_HandleTypeDef *handles[CAN_BUS_NUM] = { &hfdcan1, &hfdcan2, &hfdcan3 };
static uint32_t last_hard_reset_tick;

/* FDCAN1/2/3共用RCC复位，不能在单路掉线时直接复位整个模块。 */
void Can_Bus_Reset_Hardware(void)
{
    const IRQn_Type irq[CAN_BUS_NUM][2] = {
        {FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn},
        {FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn},
        {FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn}
    };

    for (uint32_t i = 0u; i < CAN_BUS_NUM; i++)
    {
        HAL_NVIC_DisableIRQ(irq[i][0]);
        HAL_NVIC_DisableIRQ(irq[i][1]);
        HAL_NVIC_ClearPendingIRQ(irq[i][0]);
        HAL_NVIC_ClearPendingIRQ(irq[i][1]);
    }
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_FDCAN_FORCE_RESET();
    __DSB();
    __HAL_RCC_FDCAN_RELEASE_RESET();
    __DSB();
}


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

/* 配置滤波器 + 启动 + 开中断 */
static bool Can_Bus_Start(uint32_t idx)
{
    FDCAN_HandleTypeDef *hfdcan = handles[idx];
    can_bus_t *bus = &can_bus[idx];

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;
    filter.FilterIndex = 0;
    bus->failed_step = 3u;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return false;
    }
    bus->failed_step = 4u;
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return false;
    }
    bus->failed_step = 5u;
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK || HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) != HAL_OK || HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(hfdcan);
        return false;
    }
    bus->failed_step = 0u;
    return true;
}

/* 仅通信任务调用：保留路由，按原Init恢复对应控制器；HAL出错仍可下一轮重试。 */
static void Can_Bus_Recover(uint32_t idx, uint32_t now)
{
    can_bus_t *bus = &can_bus[idx];
    FDCAN_HandleTypeDef *hfdcan = handles[idx];

    bus->state = CAN_BUS_STATE_DEAD;
    bus->reinit_tick = now;
    bus->recover_attempts++;
    /* 先屏蔽本控制器中断；高优先级执行任务也不能在重建期间继续写队列。 */
    WRITE_REG(hfdcan->Instance->IE, 0u);
    WRITE_REG(hfdcan->Instance->ILE, 0u);
    SET_BIT(hfdcan->Instance->TXBCR, hfdcan->Instance->TXBRP);
    bus->failed_step = 1u;
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY && HAL_FDCAN_Stop(hfdcan) != HAL_OK)
    {
        bus->last_error = HAL_FDCAN_GetError(hfdcan); /* Stop失败后仍尝试重新初始化。 */
    }
    bus->failed_step = 2u;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
        goto failed;
    }
    bus->alive_prev = bus->alive_cnt; /* 旧接收计数不能证明本轮恢复。 */
    if (!Can_Bus_Start(idx))
    {
        goto failed;
    }
    bus->state = CAN_BUS_STATE_RECOVERING;
    return;
failed:
    bus->last_error = HAL_FDCAN_GetError(hfdcan);
}

/* 所有已登记总线均无新反馈才升级复位，保留路由和当前Init参数。 */
static void Can_Bus_Hard_Recover(uint32_t now)
{
    taskENTER_CRITICAL();
    for (uint32_t i = 0u; i < CAN_BUS_NUM; i++)
    {
        if (can_bus[i].route_cnt != 0u && (can_bus[i].state == CAN_BUS_STATE_ACTIVE || can_bus[i].alive_cnt != can_bus[i].alive_prev || now - can_bus[i].dead_since < CAN_BUS_HARD_RESET_MS))
        {
            taskEXIT_CRITICAL();
            return;
        }
    }
    for (uint32_t i = 0u; i < CAN_BUS_NUM; i++)
    {
        can_bus[i].state = CAN_BUS_STATE_DEAD;
        WRITE_REG(handles[i]->Instance->IE, 0u);
        WRITE_REG(handles[i]->Instance->ILE, 0u);
    }
    taskEXIT_CRITICAL();

    last_hard_reset_tick = now;
    for (uint32_t i = 0u; i < CAN_BUS_NUM; i++)
    {
        (void)HAL_FDCAN_DeInit(handles[i]);
    }
    Can_Bus_Reset_Hardware();
    for (uint32_t i = 0u; i < CAN_BUS_NUM; i++)
    {
        can_bus[i].hardware_resets++;
        Can_Bus_Recover(i, HAL_GetTick());
    }
}

/* 初始化 */
void Can_Bus_Init(void)
{
    last_hard_reset_tick = HAL_GetTick();
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
        bus->recover_attempts = 0u;
        bus->hardware_resets = 0u;
        bus->last_error = 0u;

        /* 统一应用机器Init参数；初始化失败交给任务重试，不在这里永久停机。 */
        Can_Bus_Recover(i, HAL_GetTick());
    }
}

/* 路由注册 */
bool Can_Bus_Register(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                      can_parse_fn_t fn, void *ctx)
{
    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0) return false;

    can_bus_t *bus = &can_bus[idx];
    if (bus->route_cnt >= CAN_BUS_ROUTE_MAX) return false;

    for (uint32_t i = 0; i < bus->route_cnt; i++)
        if (bus->route[i].can_id == can_id) return false;

    uint32_t slot = bus->route_cnt;
    can_route_t *r = &bus->route[slot];
    r->can_id = can_id;
    r->parse  = fn;
    r->ctx    = ctx;
    __DMB();
    bus->route_cnt = slot + 1u;
    return true;
}

/* 发送 (兼容旧接口) */
HAL_StatusTypeDef Can_Bus_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;
    HAL_StatusTypeDef status;
    int32_t idx;

    idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0 || len > 8u)
    {
        return HAL_ERROR;
    }
    /* 发送临界 */
    taskENTER_CRITICAL();
    if (can_bus[idx].state == CAN_BUS_STATE_DEAD)
    {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0u)
    {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    tx_header = can_bus[idx].tx_template;
    tx_header.Identifier = can_id;
    tx_header.DataLength = len;
    status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)data);
    if (status == HAL_OK)
    {
        can_bus[idx].last_tx_tick = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
    return status;
}


/* ISR 接收 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0) return;

    can_bus_t *bus = &can_bus[idx];
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
            break;

        bus->alive_cnt++;
        bus->last_rx_id = rx_header.Identifier;
        /* FD 帧也放行 (达妙 FD 1M/4M 反馈) */
        if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.RxFrameType != FDCAN_DATA_FRAME || rx_header.DataLength != FDCAN_DLC_BYTES_8)
        {
            continue;
        }

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

/* bus-off中断只记录真正进入故障，退出通知不停止控制器；重启交给通信任务。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (!(ErrorStatusITs & FDCAN_IT_BUS_OFF) || !(hfdcan->Instance->PSR & FDCAN_PSR_BO))
    {
        return;
    }

    int32_t idx = Can_Bus_Hw_Index(hfdcan);
    if (idx < 0) return;

    can_bus[idx].alive_prev = can_bus[idx].alive_cnt;
    can_bus[idx].reinit_tick = HAL_GetTick();
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

        /* 本机没在这条总线登记设备 → 不要求流量 */
        if (bus->route_cnt == 0u)
        {
            bus->alive_prev = bus->alive_cnt;
            bus->dead_since = now;
            continue;
        }

        bool alive_changed = (bus->alive_cnt != bus->alive_prev);
        bool running = handles[i]->State == HAL_FDCAN_STATE_BUSY && !(handles[i]->Instance->CCCR & FDCAN_CCCR_INIT) && !(handles[i]->Instance->PSR & FDCAN_PSR_BO);

        if (bus->state == CAN_BUS_STATE_ACTIVE && !running)
        {
            bus->state = CAN_BUS_STATE_DEAD;
            bus->alive_prev = bus->alive_cnt;
            bus->reinit_tick = now;
        }
        /* 只有本轮启动通过、硬件运行且收到新帧，才解除CAN故障。 */
        if (bus->state == CAN_BUS_STATE_RECOVERING && running && alive_changed)
        {
            bus->state = CAN_BUS_STATE_ACTIVE;
        }

        /* 失败或启动后无反馈，均保持100ms间隔继续尝试。 */
        if (bus->state != CAN_BUS_STATE_ACTIVE && now - bus->reinit_tick >= CAN_BUS_REINIT_MS)
        {
            Can_Bus_Recover(i, now);
            all_ok = false;
            continue;
        }

        if (!expect_traffic)
        {
            if (bus->state == CAN_BUS_STATE_ACTIVE)
            {
                bus->alive_prev = bus->alive_cnt;
            }
            bus->dead_since = now;
            if (bus->state != CAN_BUS_STATE_ACTIVE)
            {
                all_ok = false;
            }
            continue;
        }

        /* RX 看门狗 */
        if (alive_changed && bus->state == CAN_BUS_STATE_ACTIVE)
        {
            bus->alive_prev = bus->alive_cnt;
            bus->dead_since = now;
        }
        else if (now - bus->dead_since > CAN_BUS_DEAD_MS)
        {
            if (bus->state == CAN_BUS_STATE_ACTIVE)
            {
                bus->state = CAN_BUS_STATE_DEAD;
                bus->alive_prev = bus->alive_cnt;
                bus->reinit_tick = now;
            }
        }

        if (bus->state != CAN_BUS_STATE_ACTIVE)
        {
            all_ok = false;
        }
    }
    if (!all_ok && now - last_hard_reset_tick >= CAN_BUS_HARD_RESET_MS)
    {
        Can_Bus_Hard_Recover(now);
    }
    return all_ok;
}
