# 下位机 sysid 数据链路计划（轮 + 闭链腿）

> 依据：`md/chuanliantui-wheel-joint-sysid-handoff.md`（算法侧交接单）
> 分工：下位机只做"施加激励 + 打时间戳记原始数据"；摩擦/阻尼/延迟/弹性的拟合全部在算法侧 MuJoCo。
> 状态：**计划，未动代码**。作者确认后再实施。
> 最后更新：待填

---

## 1. 已定决策

| 项 | 决定 |
| --- | --- |
| 两台机器 | 写两套配置表，注释/宏切换 |
| 数据落盘 | 不落 MCU，全部走 VOFA 通道（UART8），Vofa+ 保存成 CSV/文件 |
| 时间戳 | 主控单调时钟 ns；CAN 接收时刻 + CAN 发送完成时刻 |
| FK | 用 RL 侧定义（`leg_solver` 输出），不叠 LQR 的世界系变换 |
| 激励形式 | 全部"直发"：轮 = 电流 raw，腿 = 力矩 N·m；不接任何位置/速度环 |
| 坐标系定义 | 见 §8，交付给强化训练端 |

---

## 2. 机器配置表

两台机器不只是电机型号不同，**腿几何与零位偏置也不同**，必须一起切换。

建议集中到 `imcalib/user-lib/machine_config.h`，用 `#if/#else` 分块，而不是散在文件里加注释开关——
漏切一处的后果是**静默产出错单位的数据**（例如 T_MAX 用成另一台的，记录的 N·m 全错）。

每套表必须包含：

- 轮：型号、CAN 总线和 ID、电流刻度（raw/A）、满量程、减速比 `G_total`、力矩常数来源
- 腿：DM 型号、MIT 四个满量程（位置/速度/力矩/温度）、`control_id/feedback_id`、`feedback_sign`
- 腿几何：`lu`、`lg`、`offset_f`、`offset_b`、`offset_phi0`（左/右各一套）、`mirror`
- 安全限值：力矩、速度、位置、温度、轮端允许电流（测试硬上限）
- `FK_VERSION` 字符串（写进 manifest）

| 项 | 机器① chuanliantui（本次 sysid 对象） | 机器② 当前固件默认 |
| --- | --- | --- |
| 轮 | M3508 + C620 + 自制减速箱 | M2006 + C610 协议 |
| 轮刻度 | 819.2 raw/A，±16384 ↔ ±20 A | 1000 raw/A，±10000 ↔ ±10 A |
| 轮减速比 | `G_total = 19.2(P19) × 自制箱比`，**待机械提供** | 36 |
| 腿 | DM8009P | DM J4310 |
| MIT 力矩满量程 | **待达妙说明书/上位机确认** | ±10 N·m |
| MIT 速度满量程 | 待确认（现在写死 ±30 rad/s） | ±30 rad/s |
| 代码位置 | `dji.c:4`、`dji.h:14-21`、`dm.c:5`、`dm.h:36-41`、`robot_control.c:43-63` | 同左 |

> 风险兜底：启动时把"当前机器签名 + FK 版本"作为常数通道打出（或写进 manifest），
> 未定义的机器用 `#error` 拦在编译期。

---

## 3. 时间戳（已确认按此实施）

- 新增 `imcalib/user-lib/mono_ns.c/h`：`DWT->CYCCNT`（CPU 550 MHz）+ 在已有的 TIM6 500 Hz 中断里扩展成
  64 位 ns，永不回绕。代码里自己开 `TRCENA`，不依赖调试器。
  （工程里的 TIM1/2/3/12 都是 PWM 输出、TIM6 是 500 Hz 节拍、TIM23 是 HAL tick，没有空闲定时器，所以走 DWT。）
- **CAN 接收**：`dji.c:Dji_Read()`、`dm.c:Dm_Read()` 在中断回调入口取 `mono_ns()`（现在存的是 `HAL_GetTick()`）。
- **CAN 发送完成**：`can_bus.c:39` 的通知里加 `FDCAN_IT_TX_COMPLETE`，在
  `HAL_FDCAN_TxBufferCompleteCallback` 里取时间（现在只判 TX FIFO 空位 + `HAL_GetTick()`）。
- 约定：`t_cmd` = 帧**发送完成**时刻。它比 C620/DM 内部电流环生效早约 0.13 ms（帧在总线上的时长），
  这部分是**已知恒定偏置**，由算法侧拟合的 delay 吸收。README/manifest 必须写明这条约定，
  避免真机与 MuJoCo 两边约定不一致。
- 数据行统一用 `mono_ns`，1 ms 的 `HAL_GetTick()` 不再进任何数据列。

---

## 4. VOFA 数据通路

### 4.1 带宽预算（硬约束）

UART8 @ 921600 8N1 = **92160 B/s** 上限。现有 `Vofa_Send()` 是 JustFloat
（N×float32 小端 + 帧尾 `00 00 80 7F`，见 `Vofa_send.c:6`），每帧 4N+4 字节。

| 实验 | 帧 | 通道 | 字节/帧 | 速率 | 占用 |
| --- | --- | --- | --- | --- | --- |
| A 轮 | 命令完成帧 | 8 | 36 | ≤500 Hz | 18 KB/s |
| A 轮 | 反馈接收帧 | 8 | 36 | ≤500 Hz | 18 KB/s |
| B 腿 | 力矩快照帧 | 13 | 56 | 500 Hz | 28 KB/s |

结论：**轮测试命令率 ≤ 500 Hz**（1 kHz 会到 72 KB/s，占 78%，一旦 Vofa+ 卡一下就丢数据）。
腿测试 500 Hz 安全；若加开旁证原始反馈帧（＋24 KB/s）也在余量内，但默认关。

### 4.2 时间戳怎么进 float32 通道

JustFloat 只有 float32。float32 能精确表示 <2²⁴ 的整数，所以把 ns 拆两通道：

```
t_hi = floor(ns / 2^20)         /* 2^20 = 1048576 */
t_lo = ns % 2^20
ns   = t_hi * 1048576 + t_lo    /* 运行时长 < 4.9 h 内精确重建 */
```

### 4.3 帧格式

**通道 0 固定是"行类型 kind"**，不同实验不混写、每帧通道数固定，Vofa+ 的列才对得齐。

A 轮（kind=1 命令行 / kind=2 反馈行）：

| ch | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 命令行 | 1 | seq | t_hi | t_lo | `cmd_test_wheel_raw` | 另一轮 raw（=0，作为旁证） | 0 | 0 |
| 反馈行 | 2 | seq | t_hi | t_lo | `ecd_raw` | `speed_rpm` | `torque_current_raw` | `temperature_c` |

B 腿（kind=3）：`[kind, phase_id, seq, t_hi, t_lo, tau_lf0, tau_lf00, tau_rf0, tau_rf00, leg_length_L, leg_pitch_L, leg_length_R, leg_pitch_R]`

- `seq`：全局递增整数，用来判丢帧（Vofa+ 存盘后检查连续）。
- `phase_id`：本行所属阶段，由上位机脚本映射成文档要的 `phase` 名。
- 可选旁证 kind=4（默认关）：`[4, seq, t_hi, t_lo, trq_raw×4, vel_raw×4]`，用于把"命令→实际力矩"和"实际力矩→运动"分开拟合。

### 4.4 必须改的两处实现

1. **sysid 模式下停发现有 48 通道调试帧**（`task_comm.c:223`）。同一 UART 上混不同长度的帧，
   Vofa+ 的列会错位，存出来的 CSV 不可用。
2. **`Vofa_Send()` 现在"UART 忙就直接丢帧"**（`Vofa_send.c:17`）。sysid 要改成环形缓冲 + TX 完成回调续发，
   并把丢帧计数带上；有丢帧的 run 直接判无效。sysid 帧用独立缓冲，不要和 48ch 调试缓冲共用。

### 4.5 触发与急停

- 推荐：UART8 的 RX 复用现有框架（`uart_idle.c` 已支持，`huart8` 的 DMA 和 IRQ 在 CubeMX 里已配好，
  但当前代码没有起 `HAL_UART_Receive_DMA`，也没有消费数据），协议用极简 ASCII：
  `RUN <run_id> <test_id>` / `ABORT`。Vofa+ 的发送栏可以直接发。
- 激励表放在固件常量表里（由上位机只触发、不逐周期下发），这样 USB/串口抖动不会改变激励本身。
- 遥控器 s1 继续当 deadman：一旦下位/失联，立刻置零并结束 run（`task_comm.c:203` 的使能机不动）。

---

## 5. 两个实验

### 5.1 试验 A：单轮（纯电流 raw）

- 每次只测一个轮：被测轮给电流，**另一轮同帧写 0**（0x200 一帧带 4 个电机槽），四腿 0 N·m 但仍使能。
- 命令与反馈分别一行，命令率与反馈率都是 500 Hz（C620 反馈率 = 命令率）。
- 限幅：`|raw| ≤ 12288`（±15 A）硬钳；C620 温度上限；`Dji_Is_Online` 超时置零。
- 序列（照抄文档）：`baseline_sign`、`stiction`（正/负各一个 run）、`plateau`（每个电流一个 run）、
  `step`（每个幅值一个 run，重复 3 次）、`holdout_step`（不参与拟合）。
- 算法侧还要的数字：`G_total`、`eta_total`、电机侧 Kt——不能抄 268/17、5.5 N·m、eta=1。

### 5.2 试验 B：闭链腿（纯力矩 N·m）

- 四腿 `torque/Nm`（`dm.c:304` 已经是 kp=0/kd=0 的纯力矩），轮 0，**不用位置 PD、不用 reference.csv**。
- 新增 sysid 独占模式：`task_actuation.c:50` 的仲裁里加第三路，进去后 LQR/RL/手动全部屏蔽，
  保证只有一处写 DM。
- 记录点：**量化后真正上总线的 N·m**（`dm.c:302` 之后回算），避免记录值与总线值差 1 LSB。
- 时序（保证"同一时刻"）：

```
取 leg_l/leg_r 最新 FK 快照
  → 发四路力矩帧（左腿 FDCAN1、右腿 FDCAN3）
  → 等四帧 TX 完成，取最后一帧的时刻作为 t_cmd
  → 打一帧 kind=3（快照 + t_cmd）
```

- 一行四路跨两条总线，帧间隔约 0.3 ms，README 里说明。
- 用例：`torque_baseline`（全零）、`torque_step`、`torque_chirp`/`torque_prbs`、`holdout_torque_*`；
  每个用例前记录 ≥5 s 全零、结束后四路置零再记 ≥5 s；3 次有效 run + 1 次 holdout。
- chirp/PRBS 需要生成器（正弦扫频 / LFSR），幅值和频率范围以签字的 `torque_program.yaml` 为准，
  不能由固件自己编数字。
- 采样率 500 Hz（文档要求 ≥200 Hz，推荐 500 Hz）。

### 5.3 phase_id 对照表（上位机脚本用）

| phase_id | A 轮 | B 腿 |
| --- | --- | --- |
| 0 | 静止/零 | zero |
| 1 | baseline_sign 正 | baseline_start |
| 2 | baseline_sign 负 | zero_hold |
| 3 | stiction 升 | baseline_end |
| 4 | stiction 降 | zero_before |
| 5 | plateau | excite |
| 6 | step | zero_after |
| 7 | holdout_step | — |
| 15 | abort / 故障置零 | abort / 故障置零 |

---

## 6. 安全与无效标记

| 触发 | 动作 |
| --- | --- |
| 轮 raw 超限、DM 力矩超限 | 钳到限值（并在日志里能看出被钳） |
| C620 温度、DM `temp_mos`/`temp_rotor` 超限 | 立刻置零，run 标 invalid |
| CAN 掉线 / 电机离线（10 ms 超时） | 立刻置零，run 标 invalid |
| DM `err_raw ≠ 0`（现在解析了却没人看，`dm.c:146`） | 立刻置零，run 标 invalid |
| 腿长越界（工作区间 0.13~0.21 m） | 立刻置零，run 标 invalid |
| 急停 / s1 下位 / 遥控失联 | 立刻置零，run 标 invalid |
| VOFA 丢帧（seq 不连续） | 保留 CSV，run 标 invalid |

manifest.yaml 里要签字的四个 DM 限值 + 轮端允许电流，先变成固件常量并被强制，签字只是追认。

---

## 7. 交付与验收

- [ ] 下位机：`mono_ns`、CAN RX/TX 时间戳、sysid 独占模式、序列执行器、VOFA 流、限幅与无效标记
- [ ] 上位机：Vofa+ 存盘 → Python 脚本拆 kind、重建 ns、补 `run_id/test_id/phase`、算 `checksum.sha256`、
      质检（行数=节拍数 / seq 连续 / 时间戳单调 / 峰值电流 ≤ 上限）
- [ ] 空跑验收（电机不上电）：时间戳单调且无 1 ms 台阶、列对齐、seq 无缺口、超限请求被钳住
- [ ] 静态 FK 核对（文档要求的 B 前置）：手推到几个已知姿态，用流里的 `leg_length/leg_pitch` 对卷尺和量角器
- [ ] 拔 CAN 线、拔遥控：立刻置零且该 run 被标记无效
- [ ] 试验 A 先做 ±0.5 A 的 `baseline_sign`；试验 B 先做 5 s 全零

---

## 8. 坐标系与 FK 定义（交付给强化训练端）

> 以下为**已确认**的约定（来源：`md/RL_OVERVIEW.md` 极性与坐标定义、`leg_solver.c`、`md/IO_CHAINS.md`）。
> 物理符号/轴向未经台架标定不得修改。

### 8.1 机体坐标系

- `x` 前、`y` 左、`z` 上；姿态由 HI229（串口）；右腿/右轮在**驱动解码边界**统一镜像到机体坐标
  （`dm.c`、`dji.c`），后续模块禁止重复取反。

### 8.2 髋关节角

| 量 | 定义 |
| --- | --- |
| 左髋反馈 | `feedback_sign = +1`；右髋 `= -1`（驱动层已取反） |
| 前髋几何角 | `hip_f = DM 反馈角 + π + offset_f` |
| 后髋几何角 | `hip_b = DM 反馈角 + offset_b` |
| 左右腿 | 使用同一组前后映射，`mirror = +1`，五连杆层不再镜像 |

### 8.3 腿任务坐标（试验 B 的交换量，与 MuJoCo 必须同名同义）

| 量 | 定义 | 正方向/零位 |
| --- | --- | --- |
| `leg_length` | `|OP|`，O = 两髋同轴中心，P = 两下杆交点（是否即轮心销**待机械确认**） | 恒为正，伸腿变大 |
| `leg_pitch` | 机体系内 O→P 相对竖直的夹角，`π/2 − atan2(y_p, x_p) + offset_phi0` | **前摆为正**；零位由 `offset_phi0` 标定（左右腿不同） |

- 这是 **机体系** 定义。LQR 用的"世界系腿摆角 = `−leg_pitch + pitch`"只属于 LQR，sysid 与 RL 均使用机体系。
- MuJoCo 侧必须输出同定义的 `leg_length` / `leg_pitch`，否则两边差一个常数，拟合直接失效。

### 8.4 电机力矩/电流方向

| 量 | 约定 |
| --- | --- |
| 记录进 CSV 的力矩 | **URDF 关节正方向**下的 N·m，与总线量化值一致 |
| 轮电流 | 电机侧电流，单位 A，raw = `round(I_A × 819.2)`；轮端 N·m 需 `G_total` 和 Kt，不在下位机换算 |
| 轮速/轮角 | 反馈是电机侧（`ecd_raw`、`speed_rpm`），轮端量 = 除以 `G_total`；前进为正 |

### 8.5 FK 版本与可追溯性

流里/ manifest 里必须带：`FK_VERSION`、当前机器签名、`lu/lg/offset_f/offset_b/offset_phi0`（左右各一套）、
`mirror`、电机索引 ↔ URDF 关节名（`lf0/lf00/rf0/rf00`）对照表、DM 的 `control_id/feedback_id` 与 `feedback_sign`。

---

## 9. 待确认（阻塞项）

1. `G_total`（19.2 × 自制减速箱比）、`eta_total`——机械/设计图纸
2. DM8009P 的 MIT 四个满量程与许可连续/峰值力矩、温度上限——达妙说明书/上位机
3. P 点是两下杆交点还是轮心销；`offset_phi0` 零点对应的物理姿态
4. 固件索引 ↔ URDF 关节名 `lf0/lf00/rf0/rf00` 的对应关系
5. 轮测试命令率取 500 Hz 是否满足算法侧的延迟估计精度要求
6. 是否加开 kind=4 旁证原始反馈帧（默认可关）
