# RL 部署 · FreeRTOS 任务结构计划

> 最后更新：2026-09-08
> 状态：骨架阶段（任务体可跑，观测/推理/力矩为 TODO 占位）

---

## 1. 目标

在 STM32H723 + FreeRTOS(CMSIS v1) 上搭建 RL 部署的稳定任务骨架，先跑通节拍与数据流，再接运动学 / 观测 / CubeAI / 力矩。

主频：**SYSCLK = 550MHz（H723 上限），HCLK = 275MHz，定时器时钟 = 275MHz**。
（旧文档写 240MHz 是错的；PLLN=68 + FRACN=6144 + PLLP=1 → 550MHz。）

---

## 2. 任务结构总览

| 任务 | 优先级(CMSIS→FreeRTOS) | 栈(words) | 分配 | 频率 | 职责 |
|---|---|---|---|---|---|
| outputTask | osPriorityRealtime→6 | 1024 | Dynamic | 500Hz(TIM6) | 电机解析 + 力矩 + 安全门 + CAN 下发 |
| ctrlTask | osPriorityHigh→5 | 2048 | Dynamic | 100Hz | 观测量构建 + CubeAI 推理 |
| imuTask | osPriorityHigh→5 | 512 | Dynamic | 200Hz | HI229 通信 + 姿态 → imu_state |
| monitorTask | osPriorityAboveNormal→4 | 512 | Dynamic | 1kHz | DR16 解析 + 在线检测 + VOFA |
| defaultTask | osPriorityNormal→3 | 128 | Dynamic | - | USB 初始化（保留） |

TIM6：`PSC=549, ARR=999` → 500Hz 更新中断，NVIC 优先级 5。

---

## 3. 关键设计：任务体 = 独立函数

任务入口函数是 CubeMX 生成的**强定义**（没勾 As weak），不能跨文件覆盖。解法：

- 真正逻辑写成**独立函数** `xxx_task_body()`，放在 `imcalib/user-lib/robot_tasks.c`。
- CubeMX 生成的入口函数里只调用它（在 USER CODE 区，重生成不丢）：

```c
void ctrlTask_Entry(void const *argument)
{
  /* USER CODE BEGIN ctrlTask_Entry */
  ctrl_task_body();   /* 独立函数，永不返回 */
  /* USER CODE END ctrlTask_Entry */
}
```

好处：CubeMX 反复生成不丢实现，且逻辑集中在独立文件。

---

## 4. 数据流与共享状态（单写者）

```
[ISR] UART7 IDLE ─> hi229_rx.flag    (HI229)
[ISR] UART9 IDLE ─> dbus_rx.flag     (DR16)
[ISR] FDCAN     ─> dm/dji raw_pending (电机反馈)
[ISR] TIM6 500Hz ─> osSemaphoreRelease(ctrl_tick_sem)

imuTask     HI229_Process/Online → imu_state{quat,eul,gyr,acc,online}
ctrlTask    读 imu_state+motor_state+command_state → obs+history → 推理 → action_state
outputTask  Dm/Dji_Parse → motor_state → 力矩 → 安全门 → CAN
monitorTask DR16_Process/Online → command_state；在线检测 → ctrl_fault；VOFA
```

| 共享 | 写者 | 读者 | 说明 |
|---|---|---|---|
| imu_state | imuTask | ctrlTask / monitorTask | 单写者，32bit 对齐 float，无需锁 |
| motor_state | outputTask | ctrlTask | 同上 |
| action_state | ctrlTask | outputTask | 同上 |
| command_state | monitorTask | ctrlTask | 同上 |
| ctrl_fault | monitorTask | outputTask | volatile uint32 位掩码 |
| tick_count | outputTask | monitorTask | volatile uint32，VOFA 心跳 |

> 单写者 + 对齐 float 在 M7 上读写原子。跨字段撕裂（读到半新半旧）当前周期可容忍；
> 后续若需要强一致，再加版本号或双缓冲。

---

## 5. TIM6 节拍：为什么用信号量而不是标志位

| 方案 | CPU | 抖动 | 结论 |
|---|---|---|---|
| 标志位 + 忙等 `while(!flag)` | 100% 占用一核 | 极小 | 废 CPU，别的任务饿死 |
| 标志位 + `osDelay` 轮询 | 低 | **±1ms（=1 tick）** | 2ms 周期抖 50%，平衡环不可用 |
| **信号量阻塞** | 0（睡眠） | μs 级 | 最优 |

原因：`configTICK_RATE_HZ=1000`，FreeRTOS 时间片 = 1ms。500Hz 执行周期 = 2ms，用 `osDelay`/`vTaskDelayUntil` 唤醒时刻被量化到 1ms 边界，抖动 = 周期的 50%。力矩下发抖 1ms 直接进控制环。

信号量：任务 `osSemaphoreWait(..., osWaitForever)` 阻塞睡眠（零 CPU），TIM6 中断 `osSemaphoreRelease` 立即唤醒（中断→任务切换 μs 级），抖动亚微秒。这是硬实时节拍的标椎做法。

**分工**：`outputTask`（500Hz 力矩，最敏感）用 TIM6+信号量；`imu/ctrl/monitor` 用 `osDelay` 即可（非关键环，可容忍 1ms 抖动）。

---

## 6. 同步机制细节

- 信号量定义与创建在 `robot_tasks.c`，`Robot_Control_Init()` 在 freertos.c 的
  `USER CODE BEGIN RTOS_SEMAPHORES` 区调用（调度器启动前建好）。
- `osSemaphoreCreate(osSemaphore(ctrl_tick_sem), 1)` → count=1 → **二进制信号量，初始空**。
- TIM6 在 `output_task_body()` 里 `HAL_TIM_Base_Start_IT(&htim6)` 启动（此时调度器已跑、信号量已建）。
- TIM6 中断 → `HAL_TIM_IRQHandler(&htim6)` → `HAL_TIM_PeriodElapsedCallback` → 释放信号量。
  `osSemaphoreRelease` 内部 `inHandlerMode()` 判断，ISR 里自动走 `xSemaphoreGiveFromISR`（已核实）。

---

## 7. 各任务体职责（骨架）

### imu_task_body() — 200Hz
`HI229_Process()` → `HI229_Online()` → 拷贝 quat/eul/gyr(deg/s→rad/s)/acc → `imu_state`。
HI229 数据**只由本任务拥有**（Process+Online 同任务，避免跨任务访问 hi229_data）。

### ctrl_task_body() — 100Hz
TODO 占位。未来：读 imu_state + motor_state + command_state → 电机偏置 → 五连杆运动学 →
build obs(25) + history(125) → CubeAI 推理 → action_state。

### output_task_body() — 500Hz（TIM6）
`osSemaphoreWait` → `Dm_Parse()` + `Dji_Parse()` → 更新 motor_state →
查 `ctrl_fault`：非零则 `Dji_All_Stop()` + DM disable → 否则（TODO 力矩）下发零电流占位。

### monitor_task_body() — 1kHz
`DR16_Process()` → `DR16_Online()` → 写 command_state（TODO 通道映射/缩放）→
在线检测（imu_state.online / DR16 / Can_Bus_Online / 电机在线）→ 写 ctrl_fault →
VOFA 200Hz（心跳 + 欧拉角 + 通道 + fault 位）。

---

## 8. D-Cache 处理

D-Cache 会和所有 DMA 数据通路冲突（UART/CAN/SPI/VOFA 读旧数据）。**当前已关闭**：

- 短期：main.c 里 `SCB_EnableDCache()` 注释掉。
- 永久：CubeMX `CORTEX_M7` 面板关 D-Cache（否则每次生成都会加回来）。
- I-Cache 保留（无副作用，只加速）。
- 若后续为 CubeAI 性能重新开启 D-Cache：VOFA TX 必须在 DMA 前 Clean；UART DMA RX 必须在 CPU 读取前 Invalidate。工程已在 `dma_cache.h`、`Vofa_send.c`、`uart_idle.c` 实现，并将 UART DMA 缓冲区按 32B 对齐。

---

## 9. dji_motor_feedback 补 last_rx_tick

`dji_motor_feedback_t` 原来没有 `last_rx_tick`，无法做 DJI 在线检测。补：

- `dji.h`：加 `volatile uint32_t last_rx_tick;` + `#define DJI_OFFLINE_MS 10u` + `bool Dji_Is_Online(uint8_t)`。
- `dji.c`：`Dji_Read`(ISR) 里 `feedback->last_rx_tick = HAL_GetTick();`；新增 `Dji_Is_Online`。

---

## 10. 文件清单

| 文件 | 动作 |
|---|---|
| `md/leg-rl/TASKS_PLAN.md` | 新建（本文档） |
| `imcalib/user-lib/robot_tasks.h` | 新建：共享状态 + 信号量 + 任务体声明 |
| `imcalib/user-lib/robot_tasks.c` | 新建：4 个任务体 + 信号量 + 初始化 |
| `Core/Src/freertos.c` | 改：include + 信号量初始化 + 4 入口调任务体 |
| `Core/Src/main.c` | 改：include + TIM6 回调释放 + 注释 D-Cache |
| `imcalib/user-lib/dji.h` | 改：last_rx_tick + DJI_OFFLINE_MS + Dji_Is_Online |
| `imcalib/user-lib/dji.c` | 改：ISR 写 last_rx_tick + Dji_Is_Online |

> `imcalib/` 是 EIDE 的 `srcDirs`（递归扫描），新 .c 放这里自动进编译；`imcalib/user-lib` 在 include 路径里。

---

## 11. 验证步骤

1. 编译通过（无重复定义/未定义）。
2. VOFA 串口（USART1）看到 12 通道：`tick_count` 按 500Hz 递增 → 节拍正确。
3. 欧拉角随板子转动变化 → HI229 通信 + 姿态正常。
4. 拨 DR16 摇杆，ch0~ch3 变化 → 遥控正常。
5. 拔掉 HI229/DR16，fault 位对应位置 1 → 在线检测正常。

---

## 11.1 VOFA 基础链路调试

当前调试帧为 **12 通道 / 200Hz**（USART1 = 921600），用于先验证 HI229 通信。通道为：`tick_count`、Roll/Pitch/Yaw、IMU 在线、HI229 时间戳、加速度 XYZ、角速度 XYZ。

UART IDLE 接收使用环形 DMA 的读位置增量取帧，避免首帧后 DMA 长度累积，导致后续 HI229 帧被错误拒绝。

`Vofa_Send()` 在上一帧 DMA 未完成时跳过本帧，避免覆写 DMA 正在读取的静态发送缓冲区。

## 11.2 HI229 纯姿态解算

`Attitude_Algorithm` 仅接受 gyro(rad/s)、acc(g) 和 HI229 时间戳差值 dt，输出四元数、欧拉角和旋转矩阵；不依赖 BMI088、HAL 或 FreeRTOS。`imuTask` 使用 HI229 原始 acc/gyr 驱动该算法写入 `imu_state`；HI229 自带 quat/eul 仅保留为后续实机对照。

## 12. 后续阶段

1. 移植 `leg_solver`（五连杆运动学）+ `math_core`（obs/力矩）。
2. 接 CubeAI 推理（4 模型调度，激活缓存静态分配）。
3. 电机偏置标定 + 气弹簧补偿。
4. 实机调参 + IWDG 硬件看门狗。
