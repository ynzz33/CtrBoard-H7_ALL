# sysid 改动链路速查

> 用途：把每次改动的 **输入 / 输出 / 调用链 / 核对结论** 记下来，方便回退与查错。
> 维护规则：一次改动 = 一节；先写链路，再写"已核对"与"待台架"。
> 配套：设计见 `md/sysid-lower-machine-plan.md`；I/O 总览见 `md/IO_CHAINS.md`。
> 最后更新：2026-09-17

---

## 变更 1 · `machine_config.h`（新增：机器配置表）

**输入**：宏 `MACHINE_CHUANLIANTUI`（0/1，可用 `-DMACHINE_CHUANLIANTUI=1` 覆盖）

**输出**（导出的宏）：

| 宏 | 机器① chuanliantui | 机器② 本机 |
| --- | --- | --- |
| `MACHINE_NAME` | `"chuanliantui"` | `"local-m2006-j4310"` |
| `DJI_WHEEL_MODEL` | 1 (DJI_M3508) | 0 (DJI_M2006) |
| `DJI_CURRENT_MAX_RAW` | 16384 | 10000 |
| `DJI_NM_PER_RAW` | 0.30×20/16384 | 0.18×10/10000 |
| `DJI_GEAR_RATIO` | 19.2 × 自制箱比（占位 1.0） | 36 |
| `DM_MIT_POS_MAX` / `VEL_MAX` / `TRQ_MAX` | 12.5 / 45 / 54 | 3.14159 / 30 / 10 |
| `SYSID_DM_TRQ_CLAMP` | 20 | 10 |

**调用链**：纯宏，无函数调用。引用方只有 `dji.c`、`dm.c`（全仓库 grep 确认）。

**核对结论**
- 旧宏 `DJI_NM_PER_RAW_M2006/M3508`、`DJI_CURRENT_MAX_M2006/M3508`、`DJI_GEAR_RATIO` 在代码里已无其它引用（文档引用已同步）。
- 机器② 的导出值与改前逐个一致 → 现机行为不变。
- 本文件不 include 任何模块头，避免循环依赖。
- 两个分支都编译通过：`-DMACHINE_CHUANLIANTUI=1` 下 `dji.c`、`dm.c` 均 rc=0。

**待办**：`CJL_WHEEL_BOX_RATIO` 仍是 1.0f 占位；腿几何（`lu/lg/offset_*`）还没进表，仍在 `robot_control.c:43-63`。

---

## 变更 2 · `dji.c` / `dji.h`（轮子刻度改从配置表取）

**输入**
- `Dji_Torque_To_Current(index, torque_nm)`：轮端约定 N·m
- `Dji_Send_Current(hfdcan, can_id, raw[4])`：裸 raw（供 sysid 直接发电流用）
- 反馈入口：FDCAN2 的 0x201 / 0x202 帧

**输出**
- `int16 raw` → 0x200 八字节帧（4 个 int16 大端）
- `Dji_Encoder_To_Rad()` / `Dji_Rpm_To_Rad_S()`：轮轴 rad / rad·s⁻¹
- `dji_motor_feedback[]`：`angle_raw / vel_raw / current_raw / angle_total / *_rad`

**调用链（下发）**
`task_actuation.c:output_task_body()` →（LQR 分支 `leg_balance.c` / 手动- RL 分支 `rl_torque.c`）→ `Dji_Send_Wheel_Torque(l, r)` → `Dji_Torque_To_Current()` → `Dji_Send_Current()` → `Can_Bus_Transmit()` → FDCAN2。

**调用链（接收）**
FDCAN2 RX 中断 → `HAL_FDCAN_RxFifo0Callback()`（`can_bus.c:114`）→ 路由表 → `Dji_Read()` → `raw_pending` →（commTask 1 kHz）`Dji_Parse()` → `dji_motor_feedback[]` → `Motor_State_Update()`（`task_comm.c:89`）→ `motor_state.dji` → RL 观测 / LQR 速度。

**核对结论**
- 机器② 的 6 个常量与改前逐项一致（型号 M2006、±10000、0.00018、36，RAD/RPM 换算公式未动）。
- `index` 参数现在只做边界检查；两轮同型号由配置表保证（混型不会再静默用错系数）。
- 新增编译期保护 `dji_wheel_model_check`：型号宏与 `dji_motor_type_t` 枚举错位就直接编译报错。
- `dji_motor_config[]` 里只改了 `.type` 的来源，`motor_id / feedback_id / control_id / feedback_sign` 未动。

**待台架**：`DJI_NM_PER_RAW` 沿用旧值，仍未实测。

---

## 变更 3 · `dm.c` / `dm.h`（MIT 量程 + 读参 + 启动自检）

**输入 / 输出**

| 单元 | 输入 | 输出 |
| --- | --- | --- |
| `Dm_Param_Read(index, rid, *value)` | 电机下标、寄存器号 | 0/1 成功标志 + 32 位寄存器值 |
| `Dm_Scale_Check()` | 无（自己发读帧） | 写全局 `dm_scale_check`（四台的 PMAX/VMAX/TMAX/CTRL_MODE + valid/match 标志） |
| `Dm_Read()` 拦截分支 | CAN 回帧 | 读参回帧 → `param_*`；正常反馈 → `raw_data`（与改前一致） |

**调用链（自检）**
`main.c` USER CODE 2 → `Dm_Scale_Check()` → 四台各 4 次 `Dm_Param_Read()` → `Can_Bus_Transmit(0x7FF, [CANID_L, CANID_H, 0x33, RID])` →（DM 电机）→ 回帧 → FDCAN1/3 RX 中断 → 路由（feedback_id）→ `Dm_Read()` 拦截（仅 `dm_param_wait=1` 时）→ `param_pending` + `param_raw_data` → `Dm_Param_Read()` 轮询取出并校验 → `dm_scale_check`。

**调用链（消费）**
`task_comm.c:Robot_Control_Send_Vofa()`（1 kHz / 5）→ 读 `dm_scale_check.all_valid / all_match` → VOFA ch3（bit2 / bit3）。

**核对结论**
- 拦截只在 `dm_param_wait = 1`（每次读帧发出→回帧/超时之间）生效；正常运行该标志为 0，`Dm_Parse()` 路径与改前完全一致。
- 回帧三重校验：`D[2]==0x33`、`D[3]==RID`、`D[0..1]==控制 ID`，避免错配。
- 读不到（无电机 / 总线异常）只是 `valid=0`，不阻塞启动、不影响控制。
- `Dm_Parse()`、`Dm_Mit_Control()`、`Dm_Send_Torque()`、`Dm_Send_Zero()` 逻辑未改，只换常量来源。
- `dm_motor_feedback_t` 增加 `param_pending` + `param_raw_data[8]`（每台 +9 字节）。

**已知取舍**
- 读参窗口内若恰好来一帧 `D[2]==0x33` 的正常反馈，会被当成回帧吃掉（仅启动期、电机未使能、1 帧）。
- 启动最多多花 4×4×10 ms = 160 ms（全读不到时）。

**待台架**：四台读回的 PMAX/VMAX/TMAX 是否等于固件常量（决定 ch3 的 bit3），以及 CTRL_MODE 是否为 1。

---

## 变更 4 · `main.c`（启动顺序 + TIM6 回调）

**启动顺序（USER CODE 2）**
`Mono_Ns_Init()` → `DR16_Init()` → `HI229_Init()` → `Can_Bus_Init()` → `Dji_Init()` → `Dm_Init()` → `Dm_Scale_Check()` → GPIO。

**TIM6 500 Hz 回调（`HAL_TIM_PeriodElapsedCallback`）**
`Mono_Ns_Tick()` → `osSemaphoreRelease(ctrl_tick_sem_handle)`。

**调用链**
`osKernelStart` 前：`Mono_Ns_Init`（开 DWT）→ 各外设 init → `Dm_Scale_Check`（需要 CAN 已启动，故必须排在 `Can_Bus_Init` 之后）。
调度器启动后：`actuationTask` → `output_task_init()` → `HAL_TIM_Base_Start_IT(&htim6)` → TIM6 ISR → `Mono_Ns_Tick()` + 释放信号量 → `output_task_body()`。

**核对结论**
- `Mono_Ns_Init()` 排在 `Can_Bus_Init()` 之前，保证第一帧 CAN 就有 ns 可用。
- `Dm_Scale_Check()` 排在 `Can_Bus_Init()` 之后，否则读帧发不出去。
- 自检用 `HAL_GetTick()` 做超时，HAL tick（TIM23）在 `HAL_Init()` 后即工作，不依赖调度器。

**待台架**：上电时序与自检耗时的实测。

---

## 变更 5 · `task_comm.c`（VOFA ch3 增加自检位）

**输入**：`dm_scale_check.all_valid`、`dm_scale_check.all_match`
**输出**：`dbg[3] = motor_enabled + base_action_locked×2 + all_valid×4 + all_match×8`
**调用链**：commTask（1 kHz）→ `Robot_Control_Send_Vofa()`（每 5 周期发一次）→ `Vofa_Send()` → UART8 DMA。

**核对结论**：低 2 位语义不变（`ch3 & 0x03` 的老用法仍有效）；新增位只在 sysid/自检语境下有含义。文档 `md/VOFA_SEND.md` 已同步。

---

## 变更 6 · `mono_ns.c/h`（新增：单调 ns 时钟）

**输入**：`DWT->CYCCNT`（CPU 周期，1.82 ns/LSB，550 MHz）
**输出**：`uint64_t Mono_Ns_Get()`（上电起算的单调 ns）

**调用链**
- `Mono_Ns_Init()`：`main.c` USER CODE 2 第一行 → 开 `TRCENA` → `DWT->LAR=0xC5ACCE55`（M7 解锁）→ 清 CYCCNT → 开 `CYCCNTENA`
- `Mono_Ns_Tick()`：TIM6 500 Hz ISR → 把 32 位 CYCCNT 的增量累加到 64 位基数
- `Mono_Ns_Get()`：目前无人调用；步 3 起由 CAN RX 中断、CAN TX 完成回调、sysid 帧组装调用

**核对结论**
- Tick 与 Get 都在 PRIMASK 临界区内完成"基数 + 增量"的一次性快照 → 不会出现撕裂值，也不会重复计数。
- CYCCNT 32 位回绕周期 = 2³² / 550 MHz ≈ 7.81 s；Tick 由 500 Hz 调用，远小于回绕周期。
- `cyc × 1000` 在 64 位下约 9 小时内不会溢出（1.8e19 / 1000 / 550e6 ≈ 9 h）。
- `DWT->LAR` 与 `CoreDebug_DEMCR_TRCENA_Msk` 在本工程 CMSIS（`core_cm7.h:1145`、`:1693`）中存在，编译通过。
- 工程登记：Keil 组 `imcalib/user-lib` 已加入 `mono_ns.c`；eIDE 走 `srcDirs` 自动扫描，无需改 yml。

**待台架**：跑 10 s 与 `HAL_GetTick()` 对比（偏差应 < 2 ms）；连续两个节拍差值应 ≈ 2 ms。

---

## 变更 7 · 切换机器到 chuanliantui（只改一个宏）

**输入**：`MACHINE_CHUANLIANTUI` 由 0 改为 1（`machine_config.h`）
**输出（现在生效的常量）**：轮 = M3508 / ±16384 raw / 0.000366 N·m·raw⁻¹ / 总传动比 19.2；腿 = PMAX 12.5、VMAX 45、TMAX 54；激励钳位 20 N·m
**调用链**：没有新增调用，只是在编译期替换 `dji.c`、`dm.c` 里的常量；功能链路与变更 2/3 相同。

**核对结论**
- 默认（=①）与 `-DMACHINE_CHUANLIANTUI=0`（=②）两边全量编译都通过：各 99 文件 0 fail / 0 warn。
- 反馈帧格式两边一样（M2006/C610 与 M3508/C620 都是 8192 线转子侧 + 同样 8 字节布局），所以 `Dji_Parse()` 不用改。

**这次切换带来的可见差异**

| 量 | 旧配置（本机） | 新配置（chuanliantui） |
| --- | --- | --- |
| 轮速/轮角换算 | `0.10472/36` | `0.10472/19.2`（若自制箱比 ≠1，读数按比例偏） |
| 同一 N·m 命令对应的电流 | ÷0.00018 | ÷0.000366（同值下电流更小） |
| DM 力矩量化满量程 | ±10 N·m | ±54 N·m（同 N·m 下 raw 更小） |
| DM 位置满量程 | ±π | ±12.5（腿长/腿倾角读数整体变大约 4 倍） |

**仍是占位 / 未确认的**
1. `CJL_WHEEL_BOX_RATIO = 1.0f`（自制减速箱比），只影响轮端速度/角度的读数与 RL 观测。
2. 腿几何仍是旧机数值（`robot_control.c:43-63`），影响腿长/腿倾角与力矩在两条髋上的分配。
3. DM 的总线与极性沿用旧接线假设（FDCAN1 左腿 / FDCAN3 右腿、ID 1~4、左 +1 右 −1）。

**首次上电要看的四件事**
1. VOFA ch3 是否含 **+4**（四台都读到参数且 MIT 模式）与 **+8**（三项满量程与固件一致）；不含 +8 就先在上位机核对 PMAX/VMAX/TMAX。
2. 腿的伸/收方向是否与动作一致（方向反了就是 `dm.c` 表里的 `feedback_sign` 问题）。
3. 轮子转向与速度读数是否合理。
4. 是否报 `FAULT_MOTOR`（10 ms 无反馈即判离线）。

---

## 变更 8 · 测试模式入口（左中 + 右中）

**输入**：遥控 `s1`、`s2`（左/右拨杆）。判定条件 `remote.s1 == DR16_SW_MID && remote.s2 == DR16_SW_MID`
**输出**：`ctrl_strategy = CTRL_STRATEGY_SYSID`（枚举值 2）→ VOFA ch30 显示 2；测试模式内下发零力矩
**调用链**
- 输入：`commTask`（1 kHz）→ `Remote_Control_Update()` → `DR16_Process()` 解析 `dr16.s2`（`dr16.c:34`）
- 判定：`actuationTask`（500 Hz）→ `output_task_body()` → `DR16_Snapshot()` → 三分支策略判定
- 输出：`output_task_sysid()` → `Dm_Send_Zero()` + `Dji_All_Stop()` → CAN
- 观测：`commTask` → `dbg[30] = (float)ctrl_strategy`

**核对结论**
- 全量编译 99 文件 0 fail / 0 warn。
- `s2` 非中位时行为与改前逐字一致（LQR 仍走中位分支、手动/RL 走上位分支）。
- 安全链未被绕过：`motor_enabled` 仍由「s1 非下位 + 无故障 + 未翻倒」决定；s1 下位立刻失能。
- 进测试模式时强制 `lqr_running = 0`，保证从测试模式切回 LQR 时会重新按当前姿态锁存。

**现在进去只置零（空壳）**：激励序列与数据采集在步 3~9 接入；当前作用是把入口/旁路/观测这条链打通，便于先上机验证拨杆读数。

**待台架**：右拨杆 `s2` 是否真能读到（`md/IO_CHAINS.md` 曾标"未接线"，指的是"未分配功能"）；进测试模式后 ch30 应显示 2。

---

## 变更 9 · 电机配置拆成两份表 + 测试代码独立成 `imcalib/Sysid/`

**目的**：两台机器随时切换；测试代码与主干分开，关掉开关即回到原样。

**输入 / 输出**

| 单元 | 输入 | 输出 |
| --- | --- | --- |
| `machine_table[]` / `machine` | `Machine_Select(id)` | 当前机器的轮刻度/减速比、DM 三个满量程、激励钳位 |
| `Sysid_Mode_Run()` | 无 | 测试模式下的电机输出（当前 = 零力矩） |
| `Sysid_Dm_Check()` | 无 | `sysid_dm_check`（四台满量程 + MIT 模式 + valid/match） |
| `SYSID_ENABLE` | 编译期宏 | `0` = 测试代码不被调用；`1` = 打开 |

**调用链**
- `main.c` USER CODE 2 → `Machine_Select(MACHINE_ID_CHUANLIANTUI)` → `machine` 指针
- `dji.c`（`Dji_Encoder_To_Rad` / `Dji_Rpm_To_Rad_S` / `Dji_Torque_To_Current`）、`dm.c`（`Dm_Parse` / `Dm_Send_Torque` / `Dm_Send_Zero`）→ 读 `machine->…`
- `task_actuation.c` →（`#if SYSID_ENABLE`）→ `Sysid_Mode_Run()`
- `main.c` →（`#if SYSID_ENABLE`）→ `Sysid_Dm_Check()`；`task_comm.c` →（`#if`）ch3 显示位

**核对结论**
- armcc 全量：`SYSID_ENABLE=1` 与 `=0` 都是 **103 文件 0 fail / 0 warn**。
- 两份表都编进固件，切换只改 `main.c` 一行 `Machine_Select(...)`（将来可接运行时输入）。
- 关闭开关时：策略仲裁回到原来的两分支；VOFA ch3 与改动前逐位一致；不跑自检。
- 登记：Keil `machine_config.c` 进 `imcalib/user-lib` 组 + 新组 `imcalib/Sysid`；IncludePath 加 `../imcalib/Sysid`；eIDE 的 `srcDirs` / `includeList` 同步。

**待台架**：上电后 ch3 的 +4/+8；`Machine_Select` 换表后读数是否符合预期。

---

## 变更 10 · 按作者意见回退：去掉 DM 开机自检；dji 恢复"型号参数在驱动、机器表只定型号 + 减速比"

**回退了变更 3 与变更 5**（变更 3 里"MIT 量程改从配置表取"的部分保留）。

**改动**
- 删除 `imcalib/Sysid/sysid_dm_check.c/h`：连带去掉 `Dm_Param_Read()`、`Dm_Read()` 的读参拦截、`dm_param_wait`、反馈结构体的 `param_*` 字段、`dm.h` 的 RID/超时/容差宏
- `main.c` 去掉自检调用；`task_comm.c` 的 VOFA ch3 恢复原样（不再有 +4/+8）
- `dji.h` 恢复 `DJI_CURRENT_MAX_M2006/M3508`、`DJI_NM_PER_RAW_M2006/M3508`（**电机固有参数**）
- `machine_config.c` 的轮字段改为 `dji_type`(0=M2006, 1=M3508) + `dji_gear_ratio`
- `dji.c` 的 `Dji_Torque_To_Current()` 按 `machine->dji_type` 选型号常数（恢复"按型号取参数"的结构）

**理由**
- DM 满量程用达妙上位机人工核对一次即可，不值得常驻一套读参逻辑
- M2006 与 M3508 协议/驱动完全相同，只差参数（力矩常数、满量程、减速比），驱动结构不必大改

**核对**：见附录 A 的全量编译（`SYSID_ENABLE=1/0`）。

**遗留**：DM 满量程核对方式改为"上位机人工核对"（计划 §3.2 / §8.2 已同步）。

---

## 变更 11 · 换机器收敛成一行；去掉 DM 的 `type` 标签；力矩限幅改满限幅

**① 换机器只改一行**
- `machine_config.h` 新增 `#define MACHINE_DEFAULT MACHINE_ID_LOCAL`（**换机器改这一行**）
- `machine_config.c`：表指针初始化改为 `&machine_table[MACHINE_DEFAULT]`
- `main.c`：`Machine_Select(MACHINE_DEFAULT);`（不再是硬编码的机器号）
- 说明：`machine_config.c` 里那行 `const machine_cfg_t *machine = ...` 只在"没人调用 `Machine_Select` 时"生效；实际生效点是这个宏。以后要接运行时切换，调用 `Machine_Select(id)` 即可。

**② 去掉 `dm_motor_config_t.type` 与 `dm_motor_type_t`**
- 原来 `.type = DM_MOTOR_J4310` 只被 `Dm_Init()` 用来做一次边界检查，不参与协议/刻度；换电机后它会变成过期标签
- 现在：字段与枚举删除，`Dm_Init()` 的检查里去掉了 type 一项；DM 的实际刻度来自 `machine->dm_pos_max / dm_vel_max / dm_trq_max`

**③ 力矩限幅改"满限幅"（按机器取）**
- `machine_config.h` 的 `sysid_trq_clamp` 更名为 `dm_trq_clamp`，并新增 `dji_trq_clamp`（满限幅 = 型号常数 × 满量程电流）
- 值：大机器 腿 20 N·m / 轮 6.0 N·m；小机器 腿 10 N·m / 轮 1.8 N·m
- `rl_torque.c` 删掉 `RL_TQ_LEG_LIMIT/WHEEL_LIMIT/JUMP_WHEEL_LIMIT` 三个硬编码 5/5/4 N·m，改为 `machine->dm_trq_clamp` / `machine->dji_trq_clamp`（Jump 模式不再单独收窄）
- 末端仍有硬件级钳位：`dm.c` 按 MIT 满量程、`dji.c` 按 raw 满量程

**核对**：全量编译（`SYSID_ENABLE=1/0`）见附录 A。

---

## 变更 12 · DM/DJI 配置表统一（共用 `motor_cfg_t`）+ 输出极性集中

**改动**
- `can_bus.h` 新增共用结构 `motor_cfg_t`：`handle / feedback_id / control_id / feedback_sign / output_sign`
- `dm.h` → `typedef motor_cfg_t dm_motor_config_t;`；`dji.h` → `typedef motor_cfg_t dji_motor_config_t;`（两个原结构体删除）
- **dji 表删掉 `motor_id`**（原先只被 `Dji_Init()` 用来做一次边界检查）
- 两张表都补 `output_sign`（取值与 `feedback_sign` 相同：左 +1 / 右 -1）
- 极性只剩两个统一入口：
  - **反馈**：`Dm_Parse()` / `Dji_Parse()` 用 `feedback_sign`
  - **输出**：`Dm_Send_Torque()`（`output_sign` 为负则取反）、`Dji_Send_Wheel_Torque()`（`torque × output_sign`）
- 调用方的零散负号删除：`rl_torque.c` 右轮 `-tau_v[VJ_R_WHEEL]`、`leg_balance.c` 右轮 `-u[LQR_U_WR]`
  （净行为不变：原来在调用方取反，现在在驱动边界按 `output_sign` 取反一次）

**⚠️ 本轮修掉一个我上一轮引入的 bug**
- 编辑 `dji.c` 时误删了右轮的 `.feedback_id = 0x202u` → 右轮反馈永远匹配不上 → 判离线 → `FAULT_MOTOR` → 整车失能。已恢复。

**核对**：全量编译 102 文件 0 fail / 0 warn；配置结构里已无 `motor_id`（反馈结构体里的 `motor_id` 是 DM 回帧字段，保留）。

---

## 变更 13 · 极性搬进机器配置表（`dm_sign` / `dji_sign`）

**动机**：极性随**安装**变（装机/换电机/换机器都可能翻），放在驱动表里等于"换机器要改两处"；放机器表里就跟机器走。

**改动**
- `machine_config.h` 新增 `motor_sign_t { int8_t fb; int8_t out; }`，机器结构体加：
  - `motor_sign_t dm_sign[MACHINE_LEG_NUM]`（前左/后左/前右/后右）
  - `motor_sign_t dji_sign[MACHINE_WHEEL_NUM]`（左/右）
  - 常量 `MACHINE_LEG_NUM = 4` / `MACHINE_WHEEL_NUM = 2`
- `machine_config.c` 两台机器都填上（当前都是左 +1、右 -1）
- `motor_cfg_t`（`can_bus.h`）**去掉两个 sign 字段** → 只剩 `handle / feedback_id / control_id`
- 驱动表（`dm.c` / `dji.c`）删掉所有 `.feedback_sign` / `.output_sign` 行
- 使用点收敛成 4 处，全部读 `machine->…`：
  - 反馈：`Dm_Parse()` → `machine->dm_sign[i].fb`；`Dji_Parse()` → `machine->dji_sign[i].fb`
  - 输出：`Dm_Send_Torque()` → `machine->dm_sign[i].out`；`Dji_Send_Wheel_Torque()` → `machine->dji_sign[…].out`
- 两个驱动加编译期检查：`MACHINE_LEG_NUM == DM_MOTOR_NUM`、`MACHINE_WHEEL_NUM == DJI_MOTOR_NUM`

**顺带修掉一个 warning**：搬完后 `Dm_Parse()` 里的 `config` 变量没人用了（→ 删除声明）。

**核对**：全量编译 102 文件 0 fail / 0 warn。

**换机器要改的（现在全部集中）**：`machine_config.h` 的 `MACHINE_DEFAULT` + 对应机器在 `machine_config.c` 里的参数与极性。

---

## 变更 14 · 接上"总输出开关" + VOFA 串口改成可切换宏

**① 总输出开关（原来只是死代码）**
- 现状：`torque_output_enabled` 在 `robot_control.c` 里被声明并赋值，但**全工程没人读**（文档写了、实现没有）
- 改法（第一版曾放在 `output_task_body()` 开头早退，会冻住所有目标/误差/LQR 通道 → 已按作者意见改掉）：
  - `task_actuation.c` 新增 `output_send(const torque_output_t *)`，作为**唯一的下发点**
  - 开关为 0 时 → 只发 `Dm_Send_Zero()` + `Dji_All_Stop()`；为 1 时 → 正常 `Dm_Send_Torque()` + `Dji_Send_Wheel_Torque()`
  - LQR 链路与手动/RL 链路都改成调用 `output_send(&torque)`
  - **控制链路本身照常运行**：策略仲裁、LQR 状态估计+控制、RL 力矩计算、目标的误差量都还在算 → VOFA 的 `dbg[5]/[9]/[13]/[17]`（目标）、`[7]/[11]/[15]/[19]`（虚拟力矩）、`[30]`（策略）、`[31..47]`（LQR）都会正常刷新
- 当前值：`robot_control.c:39` = **0**（按作者要求关闭所有输出，供关节回馈测试）
- 注意两点：
  - 零力矩帧**必须继续发**，因为 DM 电机只在收到帧时才回状态帧；停了就收不到回馈
  - 开关为 0 时 `dbg[20..23]` 显示的是"算出来的力矩"而不是"发出去的力矩"（实际发出去的是 0）；`output_debug_dm_sent` 标志为 0 才代表真的没发力矩
- 恢复：把该行改回 `1u`

**② VOFA 串口选择宏**
- `Vofa_send.h` 由裸 `#define VOFA_UART &huart8` 改为 `VOFA_PORT` 数字选择（8=UART8 默认 / 1=USART1），`VOFA_UART` 由其推导
- 支持构建系统覆盖：`-DVOFA_PORT=1`（Keil 的 Define / eIDE 的预定义宏），不必改文件
- 填其它数字 **编译期报错**（`#error`），避免选到被占用/无 TX DMA 的口
- 已确认各口：UART8（921600 + DMA1_Stream7 NORMAL ✓）、USART1（921600 + DMA1_Stream6 NORMAL ✓、无模块占用）、UART7（HI229 占用 + TX DMA CIRCULAR ✗）、UART9（DR16 占用 + 无 TX DMA ✗）

**核对**：`VOFA_PORT=8`、`VOFA_PORT=1` 各 103 文件 0 fail / 0 warn；`VOFA_PORT=7` 如期报 `#error`。

---

## 变更 15 · CAN 总线分配也进机器配置表；IMU 新包核对结论

**① 总线分配进配置表**
- 接线差异：大机器 = 四个 DM 全在 **FDCAN1**、两个 3508 在 **FDCAN3**；小机器 = DM 左腿 FDCAN1 / 右腿 FDCAN3、轮 **FDCAN2**
- `machine_config.h`：机器结构体加 `uint8_t dm_bus[MACHINE_LEG_NUM]`（1/2/3 = FDCANx）与 `uint8_t dji_bus`
- `machine_config.c`：chuanliantui `{1,1,1,1}` + `3`；LOCAL `{1,1,3,3}` + `2`
- `can_bus.h/.c`：新增 `Can_Bus_Handle(bus)`（复用原有句柄表）
- `motor_cfg_t` 去掉 `handle` 字段 → 只剩 `feedback_id / control_id`（总线是机器级）
- `dm.c` / `dji.c`：注册与发送统一用 `Can_Bus_Handle(machine->dm_bus[i])` / `Can_Bus_Handle(machine->dji_bus)`

**核对**
- 全量编译 103 文件 0 fail / 0 warn；驱动里 `->handle` 已无残留
- 逐台脚本比对：LOCAL 腿 `[1,1,3,3]` / 轮 `2` 与 HEAD **完全一致** → 小机器行为不变 ✓
- CHUANLIANTUI：腿全 `1`、轮 `3` ✓

**② IMU 大机器新包：不需要改代码**
- 旧包 `tag(1) id(1) rev[2] prs(4) ts(4) acc gyr mag eul quat` = 76 字节
- 新包 `tag(1) status(2) tmp(1) prs(4) ts(4) acc gyr mag eul quat` = **同样 76 字节**
- 两者只有 **offset 1-3** 的含义不同（`id+rev` ↔ `status+tmp`），而固件只读 `ts/acc/gyr/eul/quat`（offset 4 之后）→ **偏移完全一致**
- 结论：只要帧头（0x5A 0xA5）、长度字段（76）、波特率（921600）不变，解析无需改动
- 若之后要显示新包的 `status` / `tmp`，再加两个字段即可（暂缓）

---

## 变更 16 · DM 反馈 ID 兼容两种 Master ID

**背景**：换大机器后 ch0=193（只有 IMU + 两轮在线），四台腿全离线；总线上没有匹配到任何反馈帧。
原因：DM 回帧 ID = 电机里的 **Master ID + CAN ID**，表里假设的是 `0x10 + ID`（→ 0x11..0x14）；另一批电机的 Master ID 常为 **0**（→ 回帧就是 0x01..0x04）。

**改动**：`dm.c` 的 `Dm_Init()` 对每台电机**注册两个反馈 ID**：`feedback_id`（0x11..0x14）与 `control_id`（0x01..0x04），两者都路由到同一 ctx。

**核对**：编译 103 文件 0 fail / 0 warn；路由数仍在 `CAN_BUS_ROUTE_MAX = 8` 内（大机器 FDCAN1 恰好 8、FDCAN3 2；小机器 4/4/2）。

**若仍离线**：说明电机 CAN ID 不是 0x01..0x04（我们发帧没人应答）或未上电/终端电阻缺失 → 用达妙上位机读每台的 CAN ID / Master ID。

---

## 变更 17 · DR16 接收放宽 + 加三个诊断通道

**怀疑**：`DR16_Process()` 原来是 `if (len == DR16_FRAME_LEN)`（严格 18 字节）才解析；若 IDLE 分包导致 19 字节或丢 1 字节，会**整帧丢弃** → 现象正是"接收机通信正常但主控认不到"。

**改动**
- `dr16.c`：解析条件放宽为 `len >= DR16_FRAME_LEN`（尾部带下一帧字节也能解析）；帧长不足仍丢弃
- `dr16.c/.h`：新增三个诊断量 `dr16_idle_cnt`（IDLE 事件数）、`dr16_last_len`（最近一帧字节数）、`dr16_ok_cnt`（解析成功帧数）
- `task_comm.c`：把 **ch45/46/47**（原 LQR 左腿三个量）临时改为这三个诊断量；`md/VOFA_SEND.md` 已标注，调试完恢复

**判定表**

| ch45 (idle) | ch46 (len) | ch47 (ok) | 结论 |
| --- | --- | --- | --- |
| 不涨 | 0 | 0 | UART9 一个字节都没收到 → 数据线（**PD14**）/接收机模式（必须 DBUS，SBUS 收不到） |
| 涨 | 恒 18 | 涨 | 一切正常（ch0 的 bit1 应亮） |
| 涨 | 乱跳 | 不涨 | 分包/丢字节 → 本轮放宽后应恢复 |

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 18 · VOFA ch45~47 改为 FDCAN1 接收诊断

**改动**
- `can_bus.h/.c`：`can_bus_t` 增加 `last_rx_id`（RX 中断里记录）；新增 `Can_Bus_Rx_Count(bus)` / `Can_Bus_Last_Rx_Id(bus)`
- `task_comm.c`：**替换**（不新增）ch45/46/47：
  - ch45 = **FDCAN1 收帧计数**（累计）
  - ch46 = **FDCAN1 最近一帧 CAN ID**
  - ch47 = DR16 解析成功帧数（保留遥控诊断）
- `md/VOFA_SEND.md` 同步标注

**怎么读**
| 现象 | 结论 |
| --- | --- |
| ch45 不涨、ch46 = 0 | FDCAN1 上一帧都收不到（经典配置下收到 FD 帧会被判格式错误，**这是现在 1 Mbps 配置的预期现象**） |
| 改成 FD(1M/4M) 后 ch45 开始涨、ch46 在 17/18/19/20（0x11~0x14）跳 | **腿回馈通了** → ch0 四台腿应同时亮 |
| ch45 涨但 ch46 是别的值 | 电机回帧 ID 不是 0x11~0x14 → 把该值告诉我，路由按它改 |

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 19 · ch42~47 改为六台电机反馈观测

**改动**（`task_comm.c`，只替换、不新增通道）
| 通道 | 内容 |
| --- | --- |
| ch42 | DM 左前髋位置 `dm.pos_rad[F_LFT]` |
| ch43 | DM 左后髋位置 |
| ch44 | DM 右前髋位置 |
| ch45 | DM 右后髋位置 |
| ch46 | 左轮位置 `dji.angle_total_rad[LFT]`（多圈累计） |
| ch47 | 右轮位置 `dji.angle_total_rad[RGT]`（多圈累计） |

**怎么读**：掰腿 → ch42~45 变化；转轮 → ch46/47 变化。**值在动 = 该电机通信正常**；一直不动 = 没通。
六路都是位置（速度快看不出来）：DM 是逻辑坐标（右侧已取反）；轮是多圈累计角（转起来持续变化，不绕回）。

**核对**：编译 103 文件 0 fail / 0 warn；`md/VOFA_SEND.md` 同步标注。

---

## 变更 20 · 腿几何（杆长 + 零位偏置）搬进机器配置表

**改动**
- `machine_config.h`：`machine_cfg_t` 增加
  | 字段 | 含义 |
  | --- | --- |
  | `leg_lu` / `leg_lg` | 大腿杆长 / 小腿杆长（m） |
  | `leg_off_f[2]` | 前髋零位偏置（左/右，rad） |
  | `leg_off_b[2]` | 后髋零位偏置（左/右，rad） |
  | `leg_off_phi0[2]` | 虚拟小腿零位偏置（左/右，rad） |
- `machine_config.c`：两份机器表各填一组；大机器 `lu=0.21 / lg=0.25`，零点**照抄参考固件表** `{0.476998, -1.974491} / {1.974491, -0.476998}`（前左/前右 / 后左/后右）。
- `robot_control.c`：五连杆几何与偏置从 `machine->leg_*` 读取（新增 `#include "machine_config.h"`），不再硬编码。

**输入 / 输出 / 调用链**
- 输入：`Machine_Select()` 选中的机器表
- 输出：`leg_l/leg_r.config`（`lu/lg/offset_f/offset_b/offset_phi0`）
- 调用链：`main.c:134` → `Machine_Select` → `freertos.c:117` → `Robot_Control_Init()` → `leg_*.config.*` → `task_comm.c` `Leg_State_Update()` 组 `input.hip_*` → `Leg_Solve()` → `output.*` → `rl_torque.c` / `lqr_balance.c` / `leg_balance.c` / `task_policy.c`

**怎么读**：`leg.output.valid` 为 0 说明几何/偏置不成立（腿长落到 0.136~0.46 m 之外或开方为负）。

**核对**：编译 103 文件 0 fail / 0 warn；换机器只改 `MACHINE_DEFAULT` 一处。

**待台架**：零点照抄参考固件（`实际值 = raw × 极性 + 零点`），**未实测**。注意 `task_comm.c:110` 前髋还额外 `+ LEG_PI`（原有代码，未动），所以前髋**实际零点 = 图值 + π**（左前 = 3.6186 rad）。腿长/倾角离谱 → 按卷尺 + 角度计重标。`leg_off_phi0` 两台都还是占位值。

⚠️ 本节描述的 `leg_off_f`/`leg_off_b` 字段已被变更 22 移到 `dm_zero`，以变更 22 为准。

---

## 变更 21 · VOFA ch31~47 改为腿部解算 + 电机观测（临时占用 LQR 通道）

**改动**（`task_comm.c`，只替换、不新增通道）
| 通道 | 内容 |
| --- | --- |
| ch31 / ch39 | 左 / 右 腿长 `virtual_leg_length`（m） |
| ch32 / ch40 | 左 / 右 腿摆倾角 `virtual_leg_angle`（rad） |
| ch33 / ch41 | 左 / 右 大腿角 `thigh_angle`（rad） |
| ch34 / ch42 | 左 / 右 虚拟小腿角 `virtual_shank_angle`（rad） |
| ch35~38 | 左前髋位置 / 左后髋位置 / 左前髋速度 / 左后髋速度 |
| ch43~46 | 右前髋位置 / 右后髋位置 / 右前髋速度 / 右后髋速度 |
| ch47 | 解算有效掩码（1=左, 2=右, 3=两侧） |

**输入 / 输出 / 调用链**
- 输入：`leg_solver` 输出的 `leg_l/leg_r.output`，`motor_state.dm.pos_rad/vel_rad_s`
- 输出：`dbg[48]` → `Vofa_Send(dbg, 48u)` → 当前 `VOFA_PORT` 串口
- 调用链：`comm_task_body()` → `Motor_State_Update()` → `Leg_Debug_Send()` → `Vofa_Send()`

**怎么读**
| 现象 | 结论 |
| --- | --- |
| ch47 ≠ 3 | 有一侧解算无效，后面数值都不用信 |
| ch31/ch39 竖直时 ≠ 卷尺量的轴心-足端距离 | 杆长或偏置不对 |
| 左右摆成对称姿态时 ch31 与 ch39 反向 | 右腿前后电机表项顺序或 `dm_sign` 反了 |
| ch35~38/ch43~46 值不动 | 该侧电机没通 |

**核对**：编译 103 文件 0 fail / 0 warn；`md/VOFA_SEND.md` 已同步（含原始 LQR 布局的恢复表）。

⚠️ 本节描述的 48 通道布局已被变更 24 取代，以变更 24 为准。

---

## 变更 22 · 电机零点搬到 dm.c 解码层（原始值与零点值分开）

**动机**：原来电机零点（`offset_f`/`offset_b`）在任务层 `Leg_State_Update()` 里叠加，导致 VOFA 上看到的 `pos_rad` 是"裸解码角"而不是"实际被消费的关节角"，调试时容易误判。搬到 `dm.c` 解码层后，零点后值 `pos_zero_rad` 是**唯一被消费的电机位置**（进腿部解算、进 PID、进 RL 观测），原始解码角仍然保留在 `dm_motor_feedback[].pos_rad` / `motor_state.dm.pos_rad[]`（调试器 Watch 可见），两个值互不覆盖。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/machine_config.h` | `machine_cfg_t` 删除 `leg_off_f[2]` / `leg_off_b[2]`，新增 `float dm_zero[MACHINE_LEG_NUM]`（4 台腿电机零点，顺序前左/后左/前右/后右） |
| `imcalib/user-lib/machine_config.c` | 两份机器表填 `.dm_zero`；大机器见 `MACHINE_ID_CHUANLIANTUI` 表项，小机器见 `MACHINE_ID_LOCAL` 表项（数值与原 `leg_off_f/leg_off_b` 逐项一致） |
| `imcalib/user-lib/dm.h` | `dm_motor_feedback_t` 新增 `float pos_zero_rad`（`pos_rad` 注释改为"解码角"，`pos_zero_rad` 注释"加零点"） |
| `imcalib/user-lib/dm.c` | `Dm_Parse()` 解码后新增 `feedback->pos_zero_rad = feedback->pos_rad + machine->dm_zero[i]` |
| `imcalib/task/inc/robot_control.h` | `dm_motor_state_t` 新增 `float pos_zero_rad[DM_MOTOR_NUM]` |
| `imcalib/task/task_comm.c` | `Motor_State_Update()` 同时拷贝 `pos_rad` 和 `pos_zero_rad`；`Leg_State_Update()` 改用 `dm.pos_zero_rad[]` 组 `input.hip_f/hip_b`（前髋 `+ LEG_PI` 保留，任务层不再加任何 off 项）；VOFA ch35/36/43/44 改为 `dm.pos_zero_rad[]` |
| `imcalib/Algorithm/leg_solver.h` | `leg_config_t` 删除 `offset_f` / `offset_b`（`offset_phi0` 保留，仍被求解器使用） |
| `imcalib/task/robot_control.c` | `Robot_Control_Init()` 不再写 `leg_*.config.offset_f/offset_b` |

**输入 / 输出 / 调用链**

```
machine_config.c 的 .dm_zero
    │
    │  Dm_Parse() @ imcalib/user-lib/dm.c
    ▼
dm_motor_feedback[].pos_zero_rad  (= pos_rad + dm_zero[i])
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state.dm.pos_zero_rad[]
    │
    │  Leg_State_Update() @ task_comm.c
    ▼
leg_l/leg_r.input.hip_f = pos_zero_rad[F] + LEG_PI   (前髋额外 +π)
leg_l/leg_r.input.hip_b = pos_zero_rad[B]
    │
    │  Leg_Solve()
    ▼
消费方:
  rl_torque.c    — 位置环 PID 当前值
  task_policy.c  — RL 观测 obs.joint_pos
  lqr_balance.c  — LQR 状态
  leg_balance.c  — 腿部力控
```

**怎么读**
- VOFA ch35/36/43/44 是零点后值，就是进解算和 PID 实际消费的那个值。
- 原始解码角没有上 VOFA，需要时用调试器看 `motor_state.dm.pos_rad[]`。

**数值等价性**：这次是纯搬家，**任何数值都没有变化**。旧代码 `hip_f = pos_rad + π + offset_f`、`hip_b = pos_rad + offset_b`；新代码 `pos_zero_rad = pos_rad + dm_zero[i]`，然后 `hip_f = pos_zero_rad + π`、`hip_b = pos_zero_rad`，两边相加结果完全一致。

**核对**：编译 103 文件 0 fail / 0 warn；`grep` 确认全工程 `offset_f` / `offset_b` / `leg_off_f` / `leg_off_b` 已无残留。

**待台架**：大机器 4 个零点是从参考固件表照抄的，未经实测；前髋因为代码里原有 `+π`，其等效零点是"`.dm_zero` 值 + π"。

---

## 变更 23 · 大机器 MIT 位置满量程 12.5 → ±π（作者读上位机确认）

**改动**（`machine_config.c` 大机器段，只改一行）
- `.dm_pos_max`：`12.5f` → `3.14159f`
- `.dm_vel_max = 45.0f` / `.dm_trq_max = 54.0f` **保持不变**（作者在上位机核对：速度 ±45、力矩 ±54、反馈与输出同一套刻度）

**输入 / 输出 / 调用链**
- 输入：`machine->dm_pos_max`（唯一读点 `dm.c:162`）
- 输出：`dm_motor_feedback[].pos_rad` → `pos_zero_rad` → `motor_state.dm.pos_zero_rad[]` → `leg.input.hip_f/hip_b` → `Leg_Solve()` → 腿长/腿角/雅可比 → LQR / RL 观测 / 力矩映射
- **没有位置下发路径**：全工程无 `Dm_Float_To_Uint` 编码位置（只有力矩编码，`dm.c:272` / `dm.c:304`），所以此改动只影响反馈解码

**为什么改**：手册特征表为"磁编（单圈 输出轴一圈绝对位置）"，备注①"上电后，电机位置输出限定在 [-π,π]rad 之间"。原值 12.5 取自手册中"Pos 预设 ±12.5"一句，但同一段紧接着标注"（下图仅作示例，与实际数据无关）"，且 12.5 会让**所有反馈角度放大 12.5/π ≈ 3.98 倍**。

**怎么读**：改后 ch35/36/43/44（零点后电机角）应收敛到 ±π 量级；ch31/ch39（腿长）应落回 0.136~0.46 m 的合理区间。

**核对**：编译 103 文件 0 fail / 0 warn。

**待台架**：`dm_vel_max = 45` / `dm_trq_max = 54` 为作者读上位机所得，尚未与固件其余量纲联调验证；力矩刻度错会让"下发值"和"回读值"反向偏差同一倍数（实际 = 下发 × 电机TMAX/固件TMAX），直接污染系统辨识。

---

## 变更 24 · VOFA 通道从 48 砍到 27（上限 32）

**动机**：原 48 通道里有大量已过期的 RL 手动遥操量、旧 LQR 通道和轮子诊断量，实际只用到约一半。精简到 27 路可降低带宽（帧长从 196 → 112 字节）并留出裕量（`VOFA_MAX_CH` = 32）。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/Vofa_send.h` | `VOFA_MAX_CH` 从 48 改为 **32**（注释：上限 32；当前实际发 27 路） |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` **整体重写**：`dbg[48]` → `dbg[VOFA_MAX_CH]`，发送 `Vofa_Send(dbg, 27u)`；删掉旧的 RL 通道（大腿/小腿的当前/目标/误差/虚拟力矩）、旧轮子量、旧 LQR 通道；新增电机原始解码角 ch3~6 与零点后角 ch7~10 成对显示 |

**新通道表**：27 路，详见 `md/VOFA_SEND.md`「当前 VOFA 通道布局」。

**输入 / 输出 / 调用链**
`Robot_Control_Send_Vofa()`（`task_comm.c`）→ 组装 `dbg[27]`（在线掩码 / 解算有效 / 策略 / 电机原始角 / 零点后角 / 速度 / 左右腿解算 / 下发力矩）→ `Vofa_Send(dbg, 27u)`（`Vofa_send.c`）→ `VOFA_UART` DMA 发送

**怎么读**
- ch3/7 成对：同一台电机的"原始解码角 vs 零点后角"，差一个 `dm_zero` 常数。
- ch1 = 3 表示两腿解算都有效；不是 3 则有一侧几何/偏置不成立。
- 200Hz 发送（每 5 个 1kHz 周期一次），帧长 27×4+4 = 112 字节。

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 25 · 腿长工作区间进机器配置表

**改动**
| 文件:行 | 内容 |
| --- | --- |
| `machine_config.h:47-48` | 新增字段 `leg_len_min` / `leg_len_max` |
| `machine_config.c:28-29` | 大机器 `0.04 / 0.46` — **AI 填的理论极限占位值，待作者给实测工作区间** |
| `machine_config.c:51-52` | 小机器 `0.10 / 0.20`（作者给值） |
| `lqr_balance.h:35` | 删除 `LQR_LEG_LEN_MIN/MAX`（原 0.13 / 0.21，小机器遗留） |
| `lqr_balance.c:3 / 64-67 / 114` | 加 `#include "machine_config.h"`；使能判定与目标限幅改读 `machine->leg_len_min/max` |

**输入 / 输出 / 调用链**：`machine_config.c` 的表 → `machine->leg_len_*` → `LQR_Enable_Latch()`（腿长是否在有效域内）与 `LQR_Target_Update()`（腿长目标限幅）。

**核对**：编译 103 文件 0 fail / 0 warn；`grep LQR_LEG_LEN` 全工程无残留。

**待台架**：大机器的 0.04 / 0.46 是 `|lg−lu| ~ lu+lg` 的理论极限，不是实测工作区间。

---

## 变更 26 · ⚠️ 事故：AI 擅改大机器前髋极性 → 作者台架测出并还原

**事故**
- AI 在变更 20 同一批里，依据"参考固件极性表反推"，把大机器 `dm_sign` 从 `{{1,1},{1,1},{-1,-1},{-1,-1}}` 改成 `{{-1,-1},{1,1},{1,1},{-1,-1}}`（两个前髋取反），**未单独报备、未单独记录**。
- 台架现象：**摆动腿时腿长跟着动、腿摆角却不变**。原因：同一条腿两台电机在模型里一正一反 → 两杆"对转" → 杆夹角差 `Δ=qf−qb` 在变（腿长乱动），而两杆平均方向不变（腿摆角不动）。
- 2026-09-18 作者用"摆动/伸缩"测试判定"两个前髋反了"，**自行改回** `{{1,1},{1,1},{-1,-1},{-1,-1}}`；改回后 VOFA 通道与腿部解算正常。

**处置**
- 已在 `md/AGENTS.md` 新增 **§0.1 物理量绝对红线（最高约束）**：极性 / 零点 / 轴向 / 量程 / 符号项一律不得由 AI 改动，只能台架标定；AI 只允许"指可疑点 + 给验证方法 + 解释现象"。
- 今后此类改动即便作者要求，也必须先复述"改哪一行、从什么改成什么、依据是什么"，并在本文件**单独记一条**。

**教训（写给后续 AI）**：相位 / 极性只有台架说了算。参考固件、手册、仿真只能用来**提假设**，不能用来**改值**。

---

## 变更 27 · 打开总输出，手动遥操联调（极性台架自检）

**改动（只有一处）**
| 文件:行 | 内容 |
| --- | --- |
| `imcalib/task/robot_control.c:40` | `torque_output_enabled = 0u` → `1u`（总输出打开） |

**手动控制链路（本来已接好，无需另接线）**
`task_policy.c` `Manual_Lock_On_Enable()`（使能边沿锁存当前姿态为基准，因此使能不跳）→ `Remote_Command_Apply()` 按摇杆生成 `action_state.a` → `task_actuation.c:137` `RL_Torque_Compute()` → `output_send()` → `Dm_Send_Torque()` / `Dji_Send_Wheel_Torque()`。

**触发条件（缺一不可）**：遥控在线 → 左拨杆**上位** → 无故障且未翻倒（`motor_enabled`）→ 两腿解算有效（ch1 = 3）→ `base_action_locked`。

**摇杆映射**
| 遥控 | 动的虚拟关节 | 幅度 |
| --- | --- | --- |
| 左摇杆竖直 `ch3` | 大腿角（左右同时） | ±2 rad 目标偏移 |
| 左拨轮 `wheel` | 虚拟小腿角（左右同时） | ±2 rad |
| 右摇杆竖直 `ch1` | 轮子速度（两个轮同时） | ±80 rad/s |

**极性自检（代码层硬约束）**：每台电机的 `dm_sign[i].out` 必须等于 `dm_sign[i].fb`，否则该关节位置环变成正反馈。当前 `machine_config.c:18` 为 `{{1,1},{1,1},{-1,-1},{-1,-1}}`，四条都相等 ✓。

**核对**：编译 103 文件 0 fail / 0 warn。

**注意**：中位 = LQR（未实测）；大机器 `leg_len_min/max` 仍是 AI 填的占位值 0.04/0.46，会让 LQR 使能判定轻易通过 → 联调期间左拨杆必须停在上位。

---

## 变更 28 · 首次上电联调：限幅降到安全值

**改动（作者要求"先做安全测试，后续再放开限幅"，只改两行）**
| 文件:行 | 原值 | 现值 | 恢复值 |
| --- | --- | --- | --- |
| `machine_config.c:12` `dji_trq_clamp` | 6.0 | **1.5 Nm** | 6.0 |
| `machine_config.c:16` `dm_trq_clamp` | 20.0 | **3.0 Nm** | 20.0 |

- 只动"我们自己的限幅"，**没动 MIT 刻度**（`dm_pos_max / dm_vel_max / dm_trq_max` 按 §0.1 属物理量，未触碰）；`dm_trq_max` 行加了"勿改"注释。
- 大机器腿部额定 20 Nm，3.0 Nm ≈ 15%：够让腿在架子上慢速动作，推不动整机、也伤不到结构。

**未改动但决定"目标给多大"的三个数（属控制参数，未获授权不动）**
| 位置 | 值 | 效果 |
| --- | --- | --- |
| `task_policy.c:8` `MANUAL_ACTION_SCALE` | 4.0 | 满杆动作 ±4 → 关节目标偏移 ±2 rad（`RL_TQ_POS_SCALE 0.5`） |
| `rl_torque.c:8` `RL_TQ_WHEEL_VEL_SCALE` | 20.0 | 满杆轮速目标 **±80 rad/s** |
| `lqr_balance.h:44-45` LQR 输出限幅 | 1.5 / 2.0 Nm | 本来就是保守值，无需改 |

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 29 · 🐞 修 CAN 误判：未使用的总线不再计入在线检查（作者报"无法使能"）

**现象**：大机器上电机永远无法使能（遥控在线、拨杆到位也不出力）。

**根因（可证）**
- `can_bus.c` 的 `Can_Bus_Init()` 无条件启动 3 条 FDCAN；`Can_Bus_Online(true)` 对**每一条**总线都要求 100 ms 内有帧，否则判 DEAD。
- 大机器 `dm_bus={1,1,1,1}` + `dji_bus=3` → **FDCAN2 上没有任何登记设备**，必然在 100 ms 后判 DEAD → `task_comm.c:169` 永久置 `FAULT_CAN`(0x08) → `task_comm.c:207` 的 `enable_request` 恒为 0 → **使能永不成立**。
- 小机器 `dm_bus={1,1,3,3}` + `dji_bus=2`，三条总线都有设备，所以这个坑一直没暴露。

**改动**（`imcalib/user-lib/can_bus.c`，`Can_Bus_Online()` 循环开头加 4 行守卫）
```c
/* 本机没在这条总线登记设备 → 不要求流量 */
if (bus->route_cnt == 0u)
{
    bus->alive_prev = bus->alive_cnt;
    bus->dead_since = now;
    continue;
}
```

**输入 / 输出 / 调用链**：`machine_config.c` 的 `dm_bus[]` / `dji_bus` → `Dm_Init()` / `Dji_Init()` 在对应总线 `Can_Bus_Register()` → `bus->route_cnt` → `Can_Bus_Online()` 只检查有设备的总线 → `task_comm.c:169` `FAULT_CAN` → `Robot_Enable_Update()` 的使能门禁。

**核对**：编译 103 文件 0 fail / 0 warn。

**怎么验**：Watch `ctrl_fault` 应为 **0**；ch0 掩码应为 **255**。

---

## 附录 A · 每次改完必须跑的核对

1. 全量编译：按 `build/CtrBoard-H7_ALL/compile_commands.json` 逐条执行 armcc 命令（`-o` 指到临时目录即可）→ 要求 `0 fail / 0 warn`。
2. 另一台机器分支也要能编：在同样命令后追加 `-DMACHINE_CHUANLIANTUI=1`，至少覆盖 `dji.c`、`dm.c`。
3. `grep` 本次改动的宏/函数名，确认没有残留旧引用、没有死链。
4. 逐项对照本节记录的"输入 / 输出 / 调用链"，确认代码与文档一致。
5. 把不确定的写成"待台架"，不要写成"已完成"。

---

## 附录 B · 已知取舍与风险

| 项 | 内容 | 处理 |
| --- | --- | --- |
| 读参窗口丢帧 | 启动期可能吃掉 1 帧正常反馈 | 仅启动期、电机未使能，接受 |
| 启动延时 | 自检最多 +160 ms | 接受；失败不阻塞 |
| 满量程不一致 | ~~`P_MAX` 12.5 而实际 ±π~~ → 变更 23 已改为 ±π | 已解决；VMAX/TMAX 由作者上位机核对为 45/54 |
| 时钟调用周期 | Tick 必须 ≥ 每 7.81 s 一次 | 目前唯一挂在 500 Hz TIM6 上，满足 |
| 时钟溢出 | `cyc×1000` 约 9 小时上限 | 单次实验远小于该时长 |
| 力矩记录点 | 现在仍是"命令值"，量化后回算在步 5 | 见计划 §10.2 |
| 腿几何已进配置表 | ~~切机器时 `robot_control.c:43-63` 必须手改~~ → 变更 20 后只改 `MACHINE_DEFAULT` | 已完成 |
| 腿偏置为换算值 | 大机器偏置按参考固件表换算，未经卷尺/角度计验证 | 待台架标定；现象离谱就重标 |
