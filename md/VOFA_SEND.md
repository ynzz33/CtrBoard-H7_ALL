# VOFA+ 正常控制观测帧

> 对应代码：`imcalib/task/task_comm.c::Robot_Control_Send_Vofa()`、`imcalib/user-lib/Vofa_send.c/h`。

## 1. 串口与帧格式

- 协议：JustFloat，小端 `float32`，帧尾 `00 00 80 7F`。
- 通道数：32。
- 帧长：`32×4+4=132 B`。
- 发送频率：`commTask 1 kHz` 两分频，实际 500 Hz。
- 带宽：约 66 kB/s，占 1152000 波特 8N1 线速约 57%。
- `VOFA_PORT` 默认 1，即 USART1；可改为 8 使用 UART8。两路均为 1152000、TX DMA NORMAL。
- DMA 前由 `Vofa_Send()` 清理 D-Cache；UART 忙时本帧跳过。

UART7 被 HI229 占用，UART9 被 DR16 占用，不可作为 VOFA 输出口。

## 2. 当前 32 路通道

| ch | 内容 | 单位/位定义 |
|---:|---|---|
| 0 | 在线掩码 | bit0 IMU；bit1 遥控；bit2~5 左前/左后/右前/右后 DM；bit6~7 左/右 DJI |
| 1 | 状态位 | bit0 总使能；bit1 翻倒；bit2 左腿解算有效；bit3 右腿解算有效；bit4~7 四台 DM 已使能 |
| 2 | 控制策略 | 0 手动/RL；1 LQR；2 sysid |
| 3 | 左前髋零点后位置 | rad |
| 4 | 左后髋零点后位置 | rad |
| 5 | 右前髋零点后位置 | rad |
| 6 | 右后髋零点后位置 | rad |
| 7 | 左大腿角 | rad |
| 8 | 左虚拟小腿角 | rad |
| 9 | 左虚拟腿摆角 | rad |
| 10 | 左腿长 | m |
| 11 | 右大腿角 | rad |
| 12 | 右虚拟小腿角 | rad |
| 13 | 右虚拟腿摆角 | rad |
| 14 | 右腿长 | m |
| 15 | 左腿长目标 | m |
| 16 | 右腿长目标 | m |
| 17 | LQR 机体俯仰角 `x[THB]` | rad |
| 18 | LQR 俯仰角速度 `x[DTHB]` | rad/s |
| 19 | LQR 前进速度 `x[DS]` | m/s |
| 20 | LQR 左轮输出 `u[WL]` | N·m |
| 21 | LQR 右轮输出 `u[WR]` | N·m |
| 22 | LQR 左髋输出 `u[BL]` | N·m |
| 23 | LQR 右髋输出 `u[BR]` | N·m |
| 24 | 左轮输出轴转速 | rad/s |
| 25 | 右轮输出轴转速 | rad/s |
| 26 | 左轮实测电流 | A，`current_raw/819.2` |
| 27 | 右轮实测电流 | A，`current_raw/819.2` |
| 28 | 左前髋反馈力矩 | N·m |
| 29 | 左后髋反馈力矩 | N·m |
| 30 | 右前髋反馈力矩 | N·m |
| 31 | 右后髋反馈力矩 | N·m |

## 3. 小机器 LQR 首轮判读

- ch1 bit2/3 应同时为 1；bit4~7 应显示四台 DM 已使能。
- ch10/14 必须位于机器表与 K 表域的交集；小机器当前下界为 0.13 m。
- 抬机头时 ch17/18 应同时响应且方向一致；轴索引结论待台架。
- 架空推腿时看 ch19，并在调试器把 `lqr_debug.vel_leg_comp_sign` 切换为 `-1/+1`，选择波动较小者。
- 前倾时只读 ch20/21，预期方向必须先台架确认，未确认前保持总输出关闭。

## 4. sysid 帧

`SYSID_ENABLE=1` 的大机器测试帧、列布局和导出流程统一见 [`md/sysid/`](sysid/)；它不使用本页的正常控制 32 路布局。
