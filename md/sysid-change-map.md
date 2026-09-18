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
| 满量程不一致 | `P_MAX` 若实际 12.5 而固件为 π，则角度整体缩放 | 靠 ch3 bit3 暴露；改不改等台架结论 |
| 时钟调用周期 | Tick 必须 ≥ 每 7.81 s 一次 | 目前唯一挂在 500 Hz TIM6 上，满足 |
| 时钟溢出 | `cyc×1000` 约 9 小时上限 | 单次实验远小于该时长 |
| 力矩记录点 | 现在仍是"命令值"，量化后回算在步 5 | 见计划 §10.2 |
| 腿几何未进配置表 | 切机器时 `robot_control.c:43-63` 必须手改 | 待机器①几何确认后再挪 |
