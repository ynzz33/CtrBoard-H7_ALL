# CAN Bus 驱动模块

## 1. 概述

基于 STM32H723 FDCAN 外设的 Classic CAN 驱动，支持：
- 3 条独立 CAN 总线（FDCAN1/2/3）
- 基于 ID 的路由分发机制
- TX 模板优化（减少每次发送的字段赋值）
- Bus-off 指数退避恢复
- RX 看门狗监控

**不支持**：FD 模式、BRS 切换、DLC↔字节转换。

## 2. 硬件配置

| 总线 | GPIO | 速率 | MessageRAM Offset |
|------|------|------|-------------------|
| FDCAN1 | PD0(RX) PD1(TX) | 1Mbps | 0x000 |
| FDCAN2 | PB5(RX) PB6(TX) | 1Mbps | 0x406 |
| FDCAN3 | PD12(RX) PD13(TX) | 1Mbps | 0x812 |

**时钟源**：HSE = 24MHz
**波特率**：24MHz / (3 × (1 + 5 + 2)) = 1Mbps

## 3. 架构

```
┌─────────────────────────────────────────────────┐
│                  应用层 (Task)                    │
│  Can_Bus_Register() → 注册路由                    │
│  Can_Bus_Transmit() → 发送帧                      │
│  Can_Bus_Online()   → 健康检查                    │
├─────────────────────────────────────────────────┤
│                  CAN 驱动层                       │
│  fdcan_map[]     → 句柄指针寻址                    │
│  can_bus[].route → 路由表 {ID, parse_fn, ctx}    │
│  tx_template     → 发送头模板                     │
├─────────────────────────────────────────────────┤
│                  HAL 回调层                       │
│  HAL_FDCAN_RxFifo0Callback()    → 批量接收+分发   │
│  HAL_FDCAN_ErrorStatusCallback() → bus-off 检测   │
├─────────────────────────────────────────────────┤
│                  CubeMX HAL                      │
│  FDCAN1/2/3 → Classic CAN, 1Mbps                │
└─────────────────────────────────────────────────┘
```

## 4. 句柄指针寻址

传统方案使用硬件基地址做数组偏移，需要编译期断言。本方案使用 CubeMX 句柄指针直接比较：

```c
static FDCAN_HandleTypeDef *fdcan_map[CAN_BUS_NUM] = {0};

static int32_t Can_Bus_Hw_Index(const FDCAN_HandleTypeDef *hfdcan)
{
    for (int32_t i = 0; i < CAN_BUS_NUM; i++)
        if (fdcan_map[i] == hfdcan) return i;
    return -1;
}
```

**优点**：O(1) 常量小（N=3），自适应任意芯片，无需地址计算。

## 5. 路由注册

```c
// 电机初始化时注册
Can_Bus_Register(&hfdcan1, 0x01, DM_Motor_Parse, &motor_left);
Can_Bus_Register(&hfdcan1, 0x02, DM_Motor_Parse, &motor_right);
```

路由表存储在 `can_bus[idx].route[]` 中，每条总线最多 `CAN_BUS_ROUTE_MAX`（8）条路由。ISR 收到帧后遍历路由表，匹配 `can_id` 后调用对应 `parse(ctx, id, data, len)`。

## 6. 发送优化

TX 模板在初始化时填充 9 个字段，每次发送只覆盖 `Identifier` 和 `DataLength`：

```c
// Init 时填充模板
bus->tx_template.IdType      = FDCAN_STANDARD_ID;
bus->tx_template.FDFormat    = FDCAN_CLASSIC_CAN;
// ... 共 9 个字段

// 发送时只改 2 个字段
FDCAN_TxHeaderTypeDef tx_header = bus->tx_template;
tx_header.Identifier = can_id;
tx_header.DataLength = len;  // HAL 自动 <<16 写入硬件
```

## 7. 接收回调

`HAL_FDCAN_RxFifo0Callback` 在 ISR 中执行：
1. 检查 FIFO0 溢出标志 → `rx_lost_cnt++`
2. 批量读取 FIFO0 所有帧（while 循环）
3. 每帧遍历路由表，匹配 ID 后调用解析回调
4. 每帧 `alive_cnt++` 用于看门狗

**注意**：`rx_header.DataLength` 已经是 HAL 解码后的字节数（0~8），无需手动 `>>16`。

## 8. Bus-off 恢复

检测到 bus-off 后进入 `RECOVERING` 状态，任务层调用 `Can_Bus_Online()` 执行指数退避：

```
退避序列：10ms → 20ms → 40ms → 80ms → 160ms → 320ms (max 500ms)
最大重试：5 次
恢复条件：收到新帧 → 回到 ACTIVE
死亡条件：重试耗尽 → DEAD
```

恢复操作：
1. `SET_BIT(TXBCR, TXBRP)` — 取消排队的发送请求
2. `CLEAR_BIT(CCCR, INIT)` — 退出初始化模式

## 9. RX 看门狗

当 `expect_traffic = true` 时启用：
- `alive_cnt` 在每次收到帧时递增
- 若 `alive_cnt` 在 `CAN_BUS_DEAD_MS`（100ms）内无变化 → 总线标记为 DEAD
- 收到新帧后自动恢复为 ACTIVE

## 10. 统计计数器

```c
can_bus_stats[i].tx_full_cnt        // TX FIFO 满丢帧数
can_bus_stats[i].rx_lost_cnt        // RX FIFO 溢出数
can_bus_stats[i].bus_off_cnt        // bus-off 发生次数
can_bus_stats[i].bus_off_recover_cnt // bus-off 恢复次数
can_bus_stats[i].err_passive_cnt    // error passive 次数
can_bus_stats[i].bus_state          // 当前状态 (0=ACTIVE, 1=RECOVERING, 2=DEAD)
```

## 11. TODO

- [ ] CubeMX 中关闭 AutoRetransmission（用户确认要关闭）
- [ ] 电机驱动层（dm.c/h, dji.c/h）
- [ ] 在 main.c 中调用 `Can_Bus_Init()`
- [ ] 在任务循环中调用 `Can_Bus_Online(true)`
- [ ] 为每条总线注册具体电机路由
