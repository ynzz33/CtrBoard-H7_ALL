# sysid 改动链路速查

> 用途：把每次改动的 **输入 / 输出 / 调用链 / 核对结论** 记下来，方便回退与查错。
> 维护规则：一次改动 = 一节；先写链路，再写"已核对"与"待台架"。
> 配套：设计见 `md/sysid/sysid-lower-machine-plan.md`；I/O 总览见 `md/IO_CHAINS.md`。
> 最后更新：2026-09-18

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

⚠️ 该 27 路布局已被变更 31 取代，以变更 31 为准。

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

## 变更 30 · ⚠️ 轮子抖震排查 + 限幅放开到满值

**1. 限幅放开（作者要求"限幅给大"）**
| 文件:行 | 变更 28 的安全值 | 现值 |
| --- | --- | --- |
| `machine_config.c:12` `dji_trq_clamp` | 1.5 | **6.0**（M3508 输出轴满力矩） |
| `machine_config.c:16` `dm_trq_clamp` | 3.0 | **20.0**（作者许可力矩） |

**2. 抖震排查（作者报"右轮速度环抖震很厉害，现在大腿也有"）**

代码层面**核对通过**的点（排除嫌疑）：
- 控制节拍：TIM6 = 275 MHz/(550×1000) = **500 Hz**，`pid_calc` 的 `dt` 固定 0.002 ✓ 一致。
- PID 本体：纯 P（腿 3.5 / 轮 8.0），环绕施加在**误差**上 ✓ 正确；`abs_limit` 的 NaN 保护 ✓。
- 环路符号自洽：`dm_sign` / `dji_sign` 每条都 `out == fb` ✓。
- 轮子力矩换算：`DJI_NM_PER_RAW_M3508 = 0.30×20/16384` 就是**输出轴 6 Nm 满量程**，不是漏乘减速比 ✓。
- DM 力矩场与量程 ✓。

**首要怀疑：右轮反馈极性与实机相反 → 正反馈 → 指令力矩在 ±限幅间来回打**，抖动经结构传到整条右腿（解释了"先前小腿、现在大腿"）。
- 判据（不用开输出）：手把两轮**同向**（机器人前进方向）转，Watch `motor_state.dji.vel_rad_s[0]/[1]` 应**同号**；一正一负即该路反馈反了。
- 判据（低速小目标）：Watch `rl_control.torque_state.last_torque.dji[1]`；在 ±6 之间来回打 = 正反馈；小幅波动且转速跟得上目标 = 增益/负载问题。

**次要怀疑**：`rl_torque.c:84-85` 轮速环 `wheel_pid[..][0] = 8.0` 对**空载**轮子偏高（纯 P 一阶环极点 = 1 − K·dt/J：空载 J≈0.01 → 1.6 越过稳定边界；落地 J≈0.09 → 0.18 稳）→ 同样表现为"架起来抖、落地不抖"。**未改**（属控制参数，待极性结论）。

**核对**：编译 103 文件 0 fail / 0 warn。**极性表一个字未动。**

**3. 追加：作者反馈"闭环正常、就是抖动很厉害，感觉是软件问题"后的复核**

代码层面**全部验证正确**（排除嫌疑）：
- TIM6 = 275 MHz/(550×1000) = **500 Hz**，`pid_calc` 的 dt = 0.002 ✓ 一致
- tick = 1000 Hz；commTask `osDelay(1)`≈1 kHz、imuTask 2 ms、policyTask 10 ms ✓
- 中断优先级 TIM6/DMA/FDCAN/UART 全为 5，`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` → ISR 内调 RTOS API **合法** ✓
- 任务优先级 actuation(Realtime) > imu/policy(High) > comm(AboveNormal) → **控制任务最高** ✓
- FDCAN1 = 1 M/4 M FD_BRS（DM 电机）、FDCAN3 = CLASSIC 1 M（C620）✓ 帧格式正确
- 环路符号 fb == out ✓（与"闭环方向正常"一致）

**改动**：`imcalib/Algorithm/rl_torque.c:84-85`，STABLE 档轮速环 P：**8.0 → 2.0**。

依据（可算，不是"降增益掩盖"）：轮速环是纯 P 的一阶采样环，闭环极点 = 1 − K·dt/J。
- 本机空载轮子 J ≈ 0.01 kg·m²（M3508 转子惯量折到输出轴 ≈ 9e-3 + 轮盘 ≈ 7e-4）
- dt = 2 ms → 稳定上限 K < 2J/dt ≈ 10；环里还有 1~2 个周期的反馈/执行延迟，工程上打 3~5 折 → **K ≈ 2~3**
- 原来 8.0 超出 3~6 倍 → 表现就是"跟得上目标，但叠一层高频抖" ✓

**腿的位置环未动**：ω_n = √(3.5/0.05) ≈ 8.4 rad/s（≈1.3 Hz），离 500 Hz 采样极限很远，自身不会高频抖 → 腿上的抖判断为**被轮子振动经结构带上来**。验证：轮子断电或夹住时，腿是否还抖。

**未改但记为隐患**：`ws2812.c:16-22` 的 `WS2812_Ctrl()` 在 **commTask** 里做阻塞式 SPI（含 `while (State != HAL_SPI_STATE_READY);` 无超时死等 + 100 次阻塞发送），每 ~10 ms 触发一次，会推迟 commTask 里的 DM 反馈解码 → 让控制环吃到更旧的反馈。属可优化项，等作者定。

---

## 变更 31 · VOFA 换成虚拟关节 PID 视图（27 → 32 路）

**改动**（`imcalib/task/task_comm.c`，`Robot_Control_Send_Vofa()` **整体重写**）

| 变化 | 说明 |
| --- | --- |
| 发送路数 | `Vofa_Send(dbg, 27u)` → `Vofa_Send(dbg, 32u)`（`VOFA_MAX_CH` 仍为 32，32 路全部使用） |
| 帧长 | 27×4+4 = 112 字节 → 32×4+4 = **132 字节** |
| 新增 ch3~20 | 6 个虚拟关节 PID（含两个轮）：当前值 `get` / 目标值 `set` / 输出 `pos_out`，各 6 路（`vj_all[6]={0,1,2,3,4,5}` = 左大腿/左小腿/左轮/右大腿/右小腿/右轮） |
| 新增 ch25~26 | 轮子下发力矩：左轮 / 右轮 |
| 新增 ch27/28 | 腿长：左 / 右 `virtual_leg_length` |
| 新增 ch29~31 | 小腿雅可比：左 `vshank_jac[0]/[1]` + 右 `vshank_jac[0]` |
| 删除 | 误差块 `err`（err = 目标 − 当前，图上可目视）；电机原始解码角；电机速度；腿摆倾角；腿电机零点后角；左右腿的大腿角/虚拟小腿角 |

**新通道表**：32 路，详见 `md/VOFA_SEND.md`「当前 VOFA 通道布局」。

**输入 / 输出 / 调用链**

```
rl_control.torque_state.controller[]  (6 个虚拟关节 PID，含轮)
rl_control.torque_state.last_torque   (电机下发力矩，DM + DJI)
leg_l|r.output.virtual_leg_length     (腿长)
leg_l|r.output.vshank_jac[]           (小腿雅可比)
    │
    │  Robot_Control_Send_Vofa() @ task_comm.c（每 5 个 1kHz 周期）
    ▼
dbg[32]
    │
    │  Vofa_Send(dbg, 32u) @ Vofa_send.c
    ▼
VOFA_UART DMA 发送（132 字节 + 4 字节帧尾）
```

**为什么这么改**：作者要查"小腿不动"的原因，需要虚拟关节的当前/目标/PID输出 以及小腿雅可比，才能完整追踪"虚拟力矩 → 雅可比 → 电机指令力矩"这条链路。追加后扩展到 6 个虚拟关节（含两个轮），并补上轮子下发力矩，使全部执行链路可见。

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 32 · 系统辨识前置：CAN 接收打 ns 时间戳（计划缺口 A）

**背景**：按 `md/sysid/sysid-lower-machine-plan.md` 执行"6 关节建模"，缺口 A 是全部数据采集的前置（`rx_ns` 决定每一行的时刻精度）。

**改动**（4 个文件，无控制行为变化、不涉及任何物理量）
| 文件:行 | 内容 |
| --- | --- |
| `imcalib/user-lib/dm.h:54` | `dm_motor_feedback_t` 新增 `uint64_t rx_ns;` |
| `imcalib/user-lib/dji.h:47` | `dji_motor_feedback_t` 新增 `uint64_t rx_ns;` |
| `imcalib/user-lib/dm.c:94` | `Dm_Read()` ISR 入口记录 `Mono_Ns_Get()`；新增 `#include "mono_ns.h"` |
| `imcalib/user-lib/dji.c:94` | `Dji_Read()` ISR 入口同上 |

**输入 / 输出 / 调用链**：CAN RX 中断 → `HAL_FDCAN_RxFifo0Callback` → `Dm_Read()` / `Dji_Read()` → 打 `rx_ns`（DWT + TIM6 扩展的单调 ns）→ `raw_pending` → `Dm_Parse()` / `Dji_Parse()` 解码 → 后续 sysid 数据行用 `rx_ns` 作时间戳。

**为什么安全**：`Mono_Ns_Get()` 只读 DWT_CYCCNT + 一个由 TIM6 维护的纪元；TIM6 与 FDCAN 中断同为优先级 5（不能互相抢占），读不会撕裂；ISR 内耗时几十 ns，DM 反馈 2000 帧/s 下开销可忽略。

**核对**：编译 103 文件 0 fail / 0 warn。

**待台架验证**：连续帧的 `rx_ns` 差值应单调递增且 DM 约 2 ms（500 Hz）量级；与 `last_rx_tick` 的毫秒部分应一致。

---

## 变更 33 · 测试模式的遥控进入条件改为"左上位 + 右中位"（作者定）

**改动**（`imcalib/task/task_actuation.c:76-83`，只改条件与注释）
- 进入 `CTRL_STRATEGY_SYSID` 的条件：`s1 == 中位 && s2 == 中位` → **`s1 == 上位 && s2 == 中位`**
- 其余不变：`s1 中位` → LQR；否则 → 手动遥操/RL。因此**只要左拨杆不在上位、或右拨杆不在中位，下一个控制周期（2 ms）就立刻离开测试模式**。

**依据**：作者 2026-09-18 明示"改成 s1 上位跟 s2 中位再开启，然后只要不是这个就立马退出"。

**注意**：`SYSID_ENABLE` 仍为 0，测试分支当前不参与编译；等 sysid 主体（计划缺口 C/D）落地后再打开。

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 34 · 大机器两项机械参数按作者实测填入（腿长区间 + 轮总减速比）

**依据**：作者 2026-09-18 确认"0.14 到 0.34 是实际区间""总减速比是 15.5"。

| 位置 | 原值 | 现值 | 来源 |
| --- | --- | --- | --- |
| `machine_config.c:10` `dji_gear_ratio` | `19.2f * CJL_WHEEL_BOX_RATIO`（=19.2，占位） | **`15.5f`** | 作者实测（转子→轮子总比） |
| `machine_config.c:27-28` `leg_len_min/max` | 0.04 / 0.46（AI 填的理论极限） | **0.14 / 0.34** | 作者实测工作区间 |
| `machine_config.c:3-4` | `CJL_WHEEL_BOX_RATIO` 占位宏（1.0） | **删除** | 总比已含箱比，占位宏失效 |

**影响面**
- `dji_gear_ratio`：是 `Dji_Rpm_To_Rad_S()` / `Dji_Encoder_To_Rad()` 的除数 → 轮子速度/角度读数按 19.2/15.5 = 1.24 倍变化（现在是真实值）。
- `leg_len_min/max`：`LQR_Enable_Latch()` 的使能判定与 `LQR_Target_Update()` 的腿长目标限幅（`imcalib/Algorithm/lqr_balance.c`）。

**核对**：编译 103 文件 0 fail / 0 warn；`grep CJL_WHEEL_BOX_RATIO` 全工程无残留。

**⚠️ 连带未处理项（等作者确认）**：`dji.h` 的 `DJI_NM_PER_RAW_M3508 = 0.30f * 20.0f / 16384.0f` 中那个 **0.30 已隐含 19.2 的减速比**（0.015625 Nm/A × 19.2 = 0.30；佐证：×20 A = 6.0 Nm = M3508 输出轴堵转力矩；M2006 同构：0.005 × 36 × 10 A = 1.8 Nm）。总比改成 15.5 后**轮子力矩换算会偏大 1.24 倍**。建议改成按 `machine->dji_gear_ratio` 显式换算（对 36 / 19.2 标准机型数值完全等价）。

---

## 变更 35 · 系统辨识前置：CAN 发送完成打 ns 时间戳 + TX pending 表（计划缺口 B）

**改动**（只动 `imcalib/user-lib/can_bus.c` / `can_bus.h`：+113 / +30 行）
| 位置 | 内容 |
| --- | --- |
| `can_bus.h` | 新增类型 `can_tx_pending_t` / `can_tx_done_t`；`can_bus_t` 新增 `tx_complete_cnt`、`tx_pending[32]`、`tx_ring_w/r`、`tx_drop_cnt`；新增 4 个 API 声明 |
| `can_bus.c` `Can_Bus_Start()` | 追加 `HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0xFFFFFFFF)` |
| `can_bus.c` `Can_Bus_Transmit()` | 改为调用新函数 `Can_Bus_Transmit_Tagged(..., 0, 0)`，**签名与语义完全不变**（`last_tx_tick` 照旧更新） |
| `can_bus.c` 新增 `Can_Bus_Transmit_Tagged()` | 入队后在 pending 表登记 `{kind, seq}`，元素索引取 `HAL_FDCAN_GetLatestTxFifoQRequestBuffer()` 的位掩码再转索引 |
| `can_bus.c` 新增 `HAL_FDCAN_TxBufferCompleteCallback()` | 对 `BufferIndexes` 每一位取 `Mono_Ns_Get()`，按元素索引从 pending 表取出标签，写入 SPSC 完成环形缓冲（每条总线 64 项，满则丢最旧并 `tx_drop_cnt++`） |
| `can_bus.c` 新增 `Can_Bus_Tx_Pop()` / `Can_Bus_Tx_Complete_Count()` / `Can_Bus_Tx_Drop_Count()` | 任务侧取完成事件与计数 |

**关键设计**：FDCAN 配的是 `FDCAN_TX_FIFO_OPERATION`（优先级排序），完成顺序 ≠ 入队顺序 → **必须按 TX 元素索引配对**，故用 pending 表 + 位掩码取索引。

**独立核对（AI 亲自验证，非子代理自述）**
1. `HAL_FDCAN_GetLatestTxFifoQRequestBuffer()` 返回的是**位掩码**：HAL 文档同一段明确说该返回值可交给 `HAL_FDCAN_AbortTxRequest(BufferIndexes)` 使用，而后者取掩码 → `31 - __CLZ(mask)` 取索引正确 ✓
2. `HAL_FDCAN_TxBufferCompleteCallback(hfdcan, BufferIndexes)` 签名与 HAL 弱定义一致（不一致会编译报错，实测 0 warn）✓
3. TX 完成中断的两条 NVIC 线（`FDCANx_IT0/IT1`）在三路 FDCAN 上都已使能且 ISR 都调 `HAL_FDCAN_IRQHandler`（`Core/Src/stm32h7xx_it.c:281/309/295/323/439/453`）→ 中断会真正进来 ✓

**已知小瑕疵（不阻塞）**：环形缓冲**溢出**时由 ISR 推进 `tx_ring_r`，与任务侧 `Can_Bus_Tx_Pop()` 同时写该索引；正常不溢出时是严格 SPSC，无影响。`can_tx_done_t.valid` 字段当前冗余未被读取。

**核对**：编译 103 文件 0 fail / 0 warn（AI 亲自跑）。

**待台架验证**：`Can_Bus_Tx_Complete_Count(1)` 应 ≈ 发送帧数；`Can_Bus_Tx_Pop()` 出的 `tx_ns` 差值应合理、单调；`Can_Bus_Tx_Drop_Count()` 应为 0。

---

## 变更 36 · 轮电机力矩刻度按总减速比缩放 + 限幅降到物理上限（作者批准）

**依据**
- 作者 2026-09-18 批准："用配置表配置是可以的""第二个也可以"。
- 参考工程交叉验证：`XYEGA_RM2026_.../readme.md:118` 明写"轮电机3508…在我们 **16.33** 减速比的减速箱条件下，它的最大输出力矩也就是 **5Nm，超过了会失控**"。用我们的常数按比例推：`6.0 × 16.33 / 19.2 = 5.1 Nm` ✓ 与参考一致 → 证明 **原来的 0.30 里确实含 19.2**，必须随实机总比缩放。
- 参考工程驱动层同样按减速比算：轮子配置带 `.reduction_radio`，力矩上限 = `CURRENT_BIT_2_A_M3508 × getDJITorqueConstant()`（按电机实例，含减速比）。

**改动**
| 文件:行 | 内容 |
| --- | --- |
| `imcalib/user-lib/dji.h:17-27` | 删 `DJI_NM_PER_RAW_M2006/M3508`；改为 `DJI_NM_FULL_M2006/M3508`（满电流输出轴堵转力矩，标准比下）+ `DJI_RATIO_STD_M2006/M3508`（36 / 19.2） |
| `imcalib/user-lib/dji.c:46-60` | `Dji_Torque_To_Current()` 的 `per_raw` 改为 `满力矩 / 满raw × (实机总比 / 标准比)` |
| `imcalib/user-lib/machine_config.c:12` | 大机器 `dji_trq_clamp` 6.0 → **4.8**（15.5 箱比下的输出轴物理上限） |

**数值自检（AI 亲自算，非子代理自述）**
| 机型 / 总比 | `per_raw` | 满量程 |
| --- | --- | --- |
| M2006 @36（小机器） | 1.800e-4 | **1.80 Nm**（与原值完全一致 → 小机器行为不变） |
| M3508 @19.2（标准比） | 3.662e-4 | **6.00 Nm**（与原值完全一致） |
| M3508 @15.5（大机器实机） | 2.956e-4 | **4.84 Nm** ← 修正后的真值 |

**核对**：编译 103 文件 0 fail / 0 warn；`grep DJI_NM_PER_RAW` 全工程无残留。

**遗留**：轮端力矩常数仍未台架实测（`md/RL_OVERVIEW.md` 记的"悬臂挂砝码法"）；本次只把"减速比"这一因子放对。

---

## 变更 37 · VOFA 通道换成 sysid 数据自检视图（作者台架核对用）

**背景**：作者要求"先把这些需要我观测的数据替换 vofa 通道，让我观测一下对不对，做台架验证"。独立 sysid 帧（计划缺口 C）仍在后面做，本次只是把将来要进帧的数据先摆到 Vofa+ 上核对。

**改动**
| 位置 | 内容 |
| --- | --- |
| `imcalib/user-lib/Vofa_send.h:9-12` | 新增布局开关 `VOFA_LAYOUT`（`#ifndef` 保护，可 `-DVOFA_LAYOUT=n` 覆盖）：1 = 虚拟关节 PID 视图，**2 = sysid 数据自检（当前默认）** |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` 拆成"共用状态块 + `#if` 两套布局"；新增 layout 2 的自检填充 |
| `imcalib/task/task_comm.c` 头部 | 新增 `#include "machine_config.h"`（layout 2 用 `machine->dji_sign / dji_bus / dm_bus`） |

**layout 2 通道表（32 路，帧长 132 字节，200Hz）**
| ch | 内容 | 单位 / 来源 |
| --- | --- | --- |
| 0~2 | 在线掩码 / 解算有效 / 策略号 | 与之前一致 |
| 3~6 | 腿**指令力矩** 前左/后左/前右/后右 | Nm，`last_torque.dm[]` |
| 7~10 | 腿**零点后角** 同上 | rad，`motor_state.dm.pos_zero_rad[]` |
| 11~12 | 腿长 左/右 | m |
| 13~14 | 腿摆倾角 左/右 | rad |
| 15~16 | 轮**指令电流 raw** 左/右 | `Dji_Torque_To_Current(i, tau × out_sign)` |
| 17~18 | 轮**转速** 左/右 | rad/s |
| 19~20 | 轮**编码器 raw** 左/右 | `dji_motor_feedback[].angle_raw` |
| 21~22 | 轮**实际电流 raw** 左/右 | `dji_motor_feedback[].current_raw` |
| 23 | 轮温度（左） | `temp_raw` |
| 24 | 轮总线 **TX 完成间隔** | µs（相邻两次取到的完成时刻之差） |
| 25 | 轮 **RX 到达间隔** | µs |
| 26 | 腿总线 **TX 完成间隔** | µs |
| 27 | 轮总线本周期 **TX 完成条数** | 每周期应为 1 |
| 28 | 腿总线本周期 **TX 完成条数** | 每周期应为 4 |
| 29 | 轮总线 **TX 丢帧累计** | 必须为 0 |
| 30~31 | 轮 RX 时刻原始拆分 hi/lo | `rx_ns >> 20` / `rx_ns & 0xFFFFF` |

**注意（写给后续）**：该视图用 `Can_Bus_Tx_Pop()` 取完成事件，**会消费掉事件**。等真正的 sysid 数据帧落地时，本视图必须让位（不能与数据帧抢事件），或改成只读计数。

**核对**：两种布局都编译通过 —— `py cc_check.py` 与 `py cc_check.py -DVOFA_LAYOUT=1` 均 **103 文件 0 fail / 0 warn**。

**待作者台架核对**：腿 4 路力矩/角度的方向；轮指令 raw 与实测转速的正负关系；`rx_ns`/`tx_ns` 间隔是否 ≈ 2000 µs（500Hz）且单调；`tx_drop` 是否为 0。

---

## 变更 38 · 🐞 修 TX 完成中断登记：只认本机登记过的元素（作者台架测出）

**现象（作者台架读数）**：ch27 / ch28（每窗口 TX 完成条数）**稳定在 64**；ch29（TX 丢帧）持续增长。

**根因（可证）**
- 缺口 B 的 `HAL_FDCAN_TxBufferCompleteCallback()` 对 `BufferIndexes` 掩码里**每一个置位的元素**都写一条完成记录，**没有检查该元素是否本机登记过**。
- 每窗口稳定 64 = **环形缓冲容量（`CAN_TX_RING_CAP` = 64）**，即缓冲每窗口都被填满。按"**32 位 × 每窗口约 2 次中断 = 64**"推断：硬件把**整个 TXBCF 掩码（32 位全置位）**都报给了回调，其中绝大多数元素没有任何登记信息 → 被当成"完成"写进环 → 环立即满 → 持续丢帧。

**改动**（`imcalib/user-lib/can_bus.c/h`）
| 位置 | 内容 |
| --- | --- |
| `can_bus.h:31-36` | `can_tx_pending_t` 增加 `uint8_t valid;` |
| `can_bus.c:174-178` | `Can_Bus_Transmit_Tagged()` 登记时置 `valid = 1` |
| `can_bus.c:239-270` | ISR 内**先判 `valid`，未登记的位直接跳过**（不计数、不写环）；处理完清 `valid` |

**预期（修后）**：ch27 ≈ **2~3** /窗口（轮 500Hz × 200Hz 采样）、ch28 ≈ **10** /窗口（腿 4 台 × 500Hz）、ch29 **保持 0**。

**核对**：两种布局（`VOFA_LAYOUT` 1/2）均编译 103 文件 0 fail / 0 warn。

---

## 变更 39 · 测试模式补全：激励序列 + 状态机 + 事件标记 + 安全

**目的**：把 sysid 从"空壳零力矩"补成完整的数据采集链路：激励序列、自动跑批、事件标记、安全/无效判定。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_config.h` | `SYSID_ENABLE` 默认改为 **1**；新增 `SYSID_PLAN` 宏（1=仅腿 2=仅轮 3=全部，默认 1） |
| `imcalib/Sysid/sysid_mode.c` | **整体重写**：新增 run 描述结构 + 激励序列表 + 单周期状态机 |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | `imcalib/Sysid` 组新增 `sysid_log.c`（之前缺失，SYSID_ENABLE=1 会链接失败） |

**新增宏（sysid_mode.c 文件顶部，易改）**

| 宏 | 值 | 含义 |
| --- | --- | --- |
| `SYSID_TRQ_LIMIT_NM` | 3.0 | 腿力矩限幅 Nm，待台架 |
| `SYSID_CURRENT_LIMIT_RAW` | 12288 | 轮电流限幅 raw（±15A） |
| `SYSID_RAW_PER_A` | 819.2 | C620 raw/A |
| `SYSID_TEMP_LIMIT_C` | 80 | 温度限 °C，待台架 |
| `SYSID_PLAN` | 1 | 1=仅腿 2=仅轮 3=全部 |

**激励序列表（sysid_runs[]，共 79 run）**

腿（SYSID_PLAN=1/3，33 run）：

| 用例 | run 数 | 结构 |
| --- | --- | --- |
| `torque_baseline` | 1 | 全零 5s |
| `torque_step` | 24 | 4路 × ±1/±2/±3 Nm，每 run：zero 2s → step 0.5s → zero 3s × 3 |
| `torque_chirp` | 4 | 4路各 1.5 Nm，0.2→10 Hz 扫频 10s，前后 zero 2s/3s |
| `holdout_torque_chirp` | 4 | 同 chirp 但 2.0 Nm |

轮（SYSID_PLAN=2/3，46 run，默认不启用）：

| 用例 | run 数 | 结构 |
| --- | --- | --- |
| `baseline_sign` | 2 | 左右各 1 run：0A 5s → +0.5A 1s → 0A 2s → -0.5A 1s → 0A 5s |
| `stiction` | 2 | 正/负各 1 run（左轮）：17 级阶梯，每级 1s |
| `plateau` | 22 | ±1~±15A 各 run（左轮）：zero 2s → 平台 5s → zero 2s |
| `step` | 10 | ±1~±15A 各 run（左轮）：zero 2s → 阶跃 0.5s → zero 3s × 3 |
| `holdout_step` | 10 | 重做 step |

**状态机**

```
每个 tick:
  重入检测 (>10ms 未调用 → 重新初始化)
  安全检查 → 失败则 abort + 停止
  循环回绕 (run_idx >= 总数 → 回到 0)
  run 开始 → 推 kind=5 标记 (phase_or_event=-1)
  计算目标值 → 限幅 → 发送命令 → 快照 → 推数据行
  run 结束 → 发零 → 推 kind=5 标记 (-2) → 下一个 run
```

**事件标记**

| phase_or_event | 列 5 | 列 6 | 列 7 | 列 8 |
| --- | --- | --- | --- | --- |
| -1 (run 开始) | run_index | test_id | 总段数 | 0 |
| -2 (run 结束) | 0 | 0 | 0 | 0 |
| -3 (中止) | 0 | 0 | 0 | 0 |

**安全链**

| 条件 | 动作 |
| --- | --- |
| `torque_output_enabled==0` | abort + 停止 |
| DM 掉线 (`Dm_Is_Online` 失败) | abort + 停止 |
| 腿解算无效 (`!leg_*.output.valid`) | abort + 停止 |
| 腿长超出 `leg_len_min/max` | abort + 停止 |
| DM 温度 > 80°C | abort + 停止 |
| 拨杆离开组合 | actuation 任务不调用 → 隐式中止（自动回零） |

停止后必须拨杆离开再回来才重新启动。

**轮电流接口**

直接调用 `Dji_Send_Current()` 发 raw 电流（`can_bus.h` 已有），不经 `Dji_Send_Wheel_Torque()`（不应用 `dji_sign.out`）。amplitude(A) × 819.2 = raw，限幅 ±12288。`tau_cmd[]` 列记录 raw 值（kind=3 行）。

**时间戳**

保持现有 pop 方式：先发命令，再从 CAN TX 完成环取最晚时刻。每周期同时 pop 腿总线和轮总线（修复原有只 pop 腿总线导致轮总线环溢出的隐含问题）。leg/wheel 行分别取对应总线的 TX 时刻。

**输入 / 输出 / 调用链**

```
actuationTask (500Hz) → output_task_body()
  → 拨杆判定: s1=UP + s2=MID → ctrl_strategy=CTRL_STRATEGY_SYSID
  → Sysid_Mode_Run()
    → sysid_safe() → 安全门禁
    → sysid_target() → 计算激励值
    → Dm_Send_Torque() / Dji_Send_Zero() (腿行)
    → Dm_Send_Zero() / Dji_Send_Current() (轮行)
    → sysid_fill_fb() → 快照
    → Sysid_Log_Push() → 环形缓冲

commTask (1kHz) → comm_task_body()
  → Sysid_Log_Send_Pump() → 500Hz DMA 发送
```

**核对**：4 种配置均 105 文件 0 fail / 0 warn：
- 默认（`SYSID_ENABLE=1, SYSID_PLAN=1`）
- `-DSYSID_ENABLE=0`
- `-DSYSID_PLAN=2`
- `-DSYSID_PLAN=3`

**已知取舍**

- 环形缓冲满时 `Sysid_Log_Push()` 返回 false，现有代码只对局部变量 `snap.whl_drop_cnt++`（不生效），此 bug 未修（不影响 CAN tx_drop 计数）
- 轮测试默认只测左轮（stiction/plateau/step），如需右轮可在表里追加
- marker 行的 `t_cmd_ns` 取自最近一次 TX 完成（可能滞后 1 个周期 ≈2ms）

**待台架**

- `SYSID_TRQ_LIMIT_NM = 3.0` 是否需要放开（大机器额定 20 Nm）
- `SYSID_TEMP_LIMIT_C = 80` 是否合适
- DM 温度 `temp_mos` / `temp_rotor` 80°C 阈值
- 轮命令与物理正方向的对应关系（变更 40 已改为按 `dji_sign.out` 换算，训练端不必再自行标定）

**⚠️ 列数未变**：仍为 31 列，帧长 128B，`sysid-delivery.md` 和 `sysid_export.py` 不需要同步。

---

## 变更 40 · 复核修正：轮命令极性同域 + 帧内轮命令列约定 + 导出填充

**背景**：变更 39 完成后主代理复核，发现两处不一致（不影响腿测试，只影响轮测试）：

1. 轮命令走 `Dji_Send_Current()` 直发 raw，绕过了 `dji_sign.out`；而帧内轮反馈在 `dji.c` 解析时已按 `dji_sign.fb` 取反 → **右轮（out = −1）命令与反馈不同域**，配对会出现"负增益"假象。
2. 帧内其实已经带了轮命令值（kind=3 行的列 5/6 写的是逻辑 raw），但列约定没写进 `sysid_log.h` / 交付文档，导出脚本把 `cmd_test_wheel_raw` 留空。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ① 轮分支上线前按 `machine->dji_sign[target].out` 换算，帧内仍记逻辑 raw；② `sysid_safe()` **删掉腿长区间门与温度门**（只留 `torque_output_enabled`、DM 离线、解算无效三个硬故障门），对齐作者既定决策"位置/速度/温度只记录、不设阈值"；`SYSID_TEMP_LIMIT_C` 保留但标注"当前未启用" |
| `imcalib/Sysid/sysid_log.h` | 注释补"行类型补充约定"：kind=3 列 5/6 = 轮命令 raw；kind=5 列 5~8 = run_index/test_id/总段数/0；kind=5 的列 3/4 仅作参考 |
| `tools/sysid_export.py` | `_write_wheel_csv_cmd()` 增 `side` 参数，按侧填 `cmd_test_wheel_raw`（左取列 5、右取列 6），两处调用同步 |
| `md/sysid/sysid-delivery.md` | 新增 §1.3.1「轮电流行（kind=3）列 5/6 约定」；§1.3 补标记行 `t_cmd_ns` 说明；按 §0.1 删去未经台架确认的极性断言（腿摆角正方向、"X 轴 = 前进方向"、轮 +0.5A 转向预期），改为"作者台架标定项"；§5.2/5.3 改成与现状一致（自动跑批、单轮时长、`SYSID_ENABLE` 默认 1、`torque_output_enabled` 前提）；§5.5 修正导出命令（`py` + `--out`）并补产物清单与列名说明 |
| `.gitignore` | 新增 `/data`（sysid 采集与导出产物不入库） |

**输入 / 输出 / 调用链**

- 轮：输入 `sysid_runs[].target`（0=左 1=右）、`sysid_to_raw(幅值A)` = 逻辑 raw、`machine->dji_sign[target].out`；输出 `Dji_Send_Current()` 的线上 raw = 逻辑 raw × 输出极性，帧列 5/6 记**逻辑** raw（与列 23~28 反馈同域）。
  调用链：`actuationTask → Sysid_Mode_Run() → sysid_clamp_f/sysid_to_raw → Dji_Send_Current → Can_Bus_Transmit → dji_bus`
  导出链：`VOFA 文件 → tools/sysid_export.py → wheel-<side>-<run>/c620_command_raw.csv`
- 腿链路未动：`sysid_clamp_f → Dm_Send_Torque → dm.c 内按 dm_sign.out 换算`，帧列 5~8 记逻辑 Nm，与腿反馈同域。

**核对**

- 编译：默认 / `-DSYSID_ENABLE=0` / `-DSYSID_ENABLE=1` 均 105 文件 0 fail / 0 warn（主代理复核）
- 导出自测：`py tools/sysid_export.py --selftest` 全项通过
- 未动任何物理量：`dji_sign` / `dm_sign` / `dm_zero` / `leg_off_phi0` 等值一律未改，只是新增"按表使用极性"的调用

**待台架**

- 轮命令与轮实际转动的物理方向仍需作者目视确认（表内 `dji_sign` 是作者标定值）
- 若作者要求帧内改记线上 raw（未乘极性），只需改列 5/6 赋值那一行，并同步交付文档与导出脚本

---

## 变更 41 · 诊断加固：心跳行 + 状态码 + 丢帧可见（作者台架排障）

**背景**：作者上机后看到 ch0=1、ch2（段号）恒为 1、ch5~8 全 0。旧代码在两种完全不同的故障下表现一模一样：①状态机没跑/被停机；②VOFA+ 没换成 JustFloat 连接或列错位。旧代码还有两处静默失败：

1. **停机后完全不推帧** → "流断了"和"没数据"无法区分
2. **环形缓冲满时丢帧不计任何数**（变更 39 已记录该 bug）→ 丢帧看不见

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①`sysid_safe()` → `sysid_fault()`，返回状态码（`SYSID_ST_*`）；②新增心跳行推送 `sysid_push_heartbeat()`，每 250 ms 一行、**任何状态都推**；③新增 `reinit_cnt`（重入重置累计，`Init` 里累加不清零）；④心跳行复用列 9~12 放诊断位 |
| `imcalib/Sysid/sysid_log.c` | ①新增 `volatile uint32_t sysid_log_drop_cnt / sysid_log_busy_cnt`；②`Sysid_Log_Push()` 失败时累加 drop；③发送泵**先判串口空闲再取数据**（原来先 `ring_r++` 再判断，UART 忙时那一帧被静默丢掉），忙则累加 busy 并保留数据 |
| `imcalib/Sysid/sysid_log.h` | 加 `SYSID_EVENT_HEARTBEAT 0`；注释补心跳行列定义；`extern` 两个诊断计数 |
| `md/sysid/sysid-delivery.md` | 新增 §1.3.2 心跳行与状态码表 |

**输入 / 输出 / 调用链**

- 输入：`stopped` / `stop_code`（`sysid_fault()` 的返回值）、`run_idx` / `run_active` / `tick_in_run`、`sysid_log_drop_cnt` / `sysid_log_busy_cnt` / `reinit_cnt`、`Mono_Ns_Get()`
- 输出：`kind=5, phase_or_event=0` 的心跳行进环形缓冲 → 发送泵 → VOFA；列 5~12 = run_idx / test_id / phase / 状态码 / 重入 / 丢帧 / 串口忙 / 心跳计数
- 调用链：`actuationTask → Sysid_Mode_Run() → hb_tick 计数 → 125 拍(250ms) → sysid_push_heartbeat() → Sysid_Log_Push() → commTask → Sysid_Log_Send_Pump() → HAL_UART_Transmit_DMA`
- 停机路径：`sysid_fault()≠0 → 推 ABORT 标记(-3) + stop_code=fault + stopped=1`，之后每周期只发零力矩，但**心跳照推**（列 8 = 停机原因）
- 导出侧：`tools/sysid_export.py` 的 `RunSplitter` 只认 -1/-2/-3 为 run 边界，心跳行（0）不影响切分，也不写入任何 CSV（未改动工具）

**核对**

- 编译：默认 / `-DSYSID_ENABLE=0` / `-DSYSID_ENABLE=1` 三种配置 0 fail / 0 warn
- 导出自测：`py tools/sysid_export.py --selftest` 全项通过
- 未动任何物理量；未改帧长（仍 31 列 / 128 B）

**待台架**

- 心跳周期 250 ms 是否合适（可改 `SYSID_HB_TICKS`）
- 作者需重新 Build + Download 才能看到心跳行

---

## 变更 42 · 激励幅值提高（作者要求：3 Nm 顶不动气弹簧）

**背景**：台架实测列 5 偶尔出现 +1 Nm 脉冲（说明状态机、激励、发送都正常），但四个髋角（列 9~12）几乎不动 —— 1~3 Nm 相对气弹簧 + 自重太小，信噪比不够，拟合不出摩擦/延迟。作者要求把激励给大。

**改动**（全部在 `imcalib/Sysid/sysid_mode.c`）

| 位置 | 原值 | 新值 |
| --- | --- | --- |
| `SYSID_TRQ_LIMIT_NM`（文件顶部宏） | 3.0f | **8.0f** |
| `sysid_runs[]` 的 `torque_step` 幅值（24 个 run） | ±1 / ±2 / ±3 Nm | **±2 / ±4 / ±6 Nm** |
| `torque_chirp` 幅值（4 个 run） | 1.5 Nm | **4.0 Nm** |
| `holdout_torque_chirp` 幅值（4 个 run） | 2.0 Nm | **5.0 Nm** |
| `md/sysid/sysid-delivery.md` | §4.1 用例表幅值、§5.3 限幅值同步 | — |

**输入 / 输出 / 调用链**

- 输入：`sysid_run_t.amplitude`（新幅值）→ `sysid_target()` → `sysid_clamp_f(target, ±SYSID_TRQ_LIMIT_NM=8)` → `Dm_Send_Torque()`
- 输出：帧列 5~8 = 已限幅的实际下发力矩（现最大 ±6 Nm，仍低于 8 Nm 上限）
- 轮用例不受影响（幅值是安培，上限 `SYSID_CURRENT_LIMIT_RAW = 12288` 未变）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 安全性：最大 6 Nm 单路、每段仅 0.5 s（阶跃）或 4~5 Nm 正弦（扫频），相对 `dm_trq_clamp = 20 Nm` 仍保守；腿周边需清空
- 未动任何物理量；未改帧长

**待台架**

- 6 Nm 是否会让腿撞限位过猛（若撞击剧烈，把最大幅值降到 4 Nm 或缩短阶跃段）
- 8 Nm 上限是否需要按结构承受能力再调（作者定）

---

## 变更 43 · 新增腿用例预压（工作点）+ 限幅放到满限幅

**背景**：台架发现腿悬空时被气弹簧顶在**最长限位**：往"伸"的方向推力矩全被限位吃掉，只有"收"的方向能动；腿长只覆盖工作区间（0.14~0.34 m）最上端；1~3 Nm 也看不出版应。作者要求加"初始目标力矩"作为工作点，从这个值起测。

**改动**（`imcalib/Sysid/sysid_mode.c`）

| 位置 | 内容 |
| --- | --- |
| 文件顶部 | 新增 `SYSID_PRELOAD_L_N` / `SYSID_PRELOAD_R_N`（沿腿力 N，负=收腿，0=不加）。初值 -80（≈11Nm/髋）；作者台架试出"太大"，改为 **-22（≈3Nm/髋）** |
| 文件顶部 | `SYSID_TRQ_LIMIT_NM`：3.0 → 8.0 → **20.0**（= 电机满限幅，给预压+激励留头寸） |
| 腿分支 | 非 baseline 的腿 run：`tau_cmd[F/B_LFT] += 预压_L × leg_jac[0][0/1]`，右侧同理，然后整组限幅 |
| `md/sysid/sysid-delivery.md` | 新增 §4.3 预压说明（换算、调法、削平检查） |

**输入 / 输出 / 调用链**

- 输入：宏常量 `SYSID_PRELOAD_*_N`、解算输出 `leg_l/leg_r.output.leg_jac[0][0..1]`（腿长对前/后髋角的偏导，`leg_solver.c` 的 `leg_jac[0][*]`）
- 计算：`τ_f = 预压 × ∂l0/∂q_f`，`τ_b = 预压 × ∂l0/∂q_b` —— 等价于"沿腿加一根常力弹簧"，负值收腿
- 输出：与激励相加后按 ±20 Nm 限幅 → `Dm_Send_Torque()`；帧列 5~8 记录**预压+激励的净力矩**
- 调用链：`actuationTask → Sysid_Mode_Run() → sysid_target() → 预压叠加 → sysid_clamp_f → Dm_Send_Torque → Can_Bus_Transmit`
- 轮分支不变（轮测试要求四路腿零命令，故不加预压）；`torque_baseline` 不加预压（保纯零基线）

**数值换算（几何数值核验）**

用 `leg_solver.c` 的几何（`lu=0.21, lg=0.25`）在腿长 0.14~0.34 m 范围内扫 2669 个姿态，得到 `|∂l0/∂q| ≈ 0.138 m/rad`（最大 0.1435），因此：

```
单髋力矩(Nm) ≈ 沿腿力(N) × 0.138      →   43 N ≈ 6 Nm,  80 N ≈ 11 Nm
```

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量（没有改 dm_sign/dm_zero/leg_off_phi0）；未改帧长
- `md/sysid-change-map.md` 里预先记录：预压力矩会与被激励电机叠加，**预压 + 激励 > 20 Nm 会削平**

**待台架**

- `SYSID_PRELOAD_*_N` 的实际取值（作者按列 17/19 的腿长迭代，目标 0.22~0.25 m）
- 预压符号方向（负是否真的是收腿）
- 20 Nm 限幅是否合适（预压是持续力矩，激励是短脉冲）

---

## 变更 44 · 预压调到 6Nm + 预压分量/削平标志上 VOFA

**背景**：作者台架反馈 `-22 N`（≈3 Nm/髋）"力太小"，要求调大；并要求把预压相关数据放进 VOFA 便于检测。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①`SYSID_PRELOAD_L_N/R_N`：-22 → -43 → 作者再调为 **-34**（≈4.7 Nm/髋）；②新增静态 `sysid_pre[4]` 保存本周期预压分量，与激励分开计算后再相加限幅；③帧填充时把预压分量写入 `snap.preload[]`，并统计被限幅的电机数 `snap.clamp_cnt` |
| `imcalib/Sysid/sysid_log.h` | `sysid_snap_t` 加 `preload[4]` / `clamp_cnt`；注释补"kind=1 行复用列 23~27" |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()`：kind=1 行把 23~27 写成预压分量与削平计数（kind=3/5 行 23~28 含义不变） |
| `md/sysid/sysid-delivery.md` | 新增 §1.3.3 腿行列 23~27 复用说明 |

**输入 / 输出 / 调用链**

- 输入：`SYSID_PRELOAD_*_N`、`leg_l/leg_r.output.leg_jac[0][*]`
- 计算：`sysid_pre[i] = 预压 × leg_jac[0][k]` → `tau_cmd[i] = clamp(tau_cmd[i] + sysid_pre[i], ±20)`
- 输出（VOFA 31 列，腿行）：列 5~8 = 净力矩，**列 23~26 = 四个髋的预压分量 Nm，列 27 = 削平电机数**，列 17~20 = 腿长/腿摆角
- 调用链：`Sysid_Mode_Run()` → 腿分支（预压+激励）→ 快照（`snap.preload/clamp_cnt`）→ `Sysid_Log_Push` → `commTask` → `Sysid_Log_Send_Pump` → `assemble_frame()` 按 kind 分支写列 23~27 → DMA → VOFA
- 导出侧：`tools/sysid_export.py` 的腿 CSV 只读列 3/4/5~8/17~20，**不受影响**（无需改工具）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列 / 128 B
- 削平判据：`|净力矩| >= 20 Nm` 即计入 `clamp_cnt`

**待台架**

- -43 N（≈6 Nm/髋）是否合适（作者按列 17/19 腿长继续迭代）
- 列 27 是否出现 >0（出现即说明预压+激励超限，需减幅或减预压）

---

## 变更 45 · 预压改为遥控滚轮实时可调 + 预压力值上 VOFA

**背景**：作者台架反馈"只有摆角、没有伸缩，摆角还很小"（-34 N ≈ 4.7 Nm/髋 仍压不动腿，且改一次值就要重烧一次）。为了不靠"改数-重烧"试凑，把预压做成**运行时可用遥控滚轮连续调节**，并把当前的沿腿力数值直接送上 VOFA。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①新增 `#include "dr16.h"`；②新增宏 `SYSID_PRELOAD_TUNE`（1=滚轮实时调，0=用固定宏）与 `SYSID_PRELOAD_TUNE_N`（滚轮到底 = 150 N）；③`Sysid_Mode_Run()` 顶部每个周期读一次滚轮并算出 `pre_n`（`-[0..150] N`，双向）；④腿分支用 `pre_n` 统一驱动左右腿；⑤快照填 `preload_n` |
| `imcalib/Sysid/sysid_log.h` | `sysid_snap_t` 加 `float preload_n;`；注释补列 28 |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()`：kind=1 行的列 28 写 `preload_n` |
| `md/sysid/sysid-delivery.md` | §1.3.3 加列 28；§4.3 加"实时调节模式"用法 |

**输入 / 输出 / 调用链**

- 输入：`DR16_Snapshot().wheel`（遥控滚轮原始值，±660）→ `DR16_Deadline(raw, 20)` 去死区 → 限幅 → 归一化 `[-1, 1]`
- 计算：`pre_n = -150 N × 归一化值`（正 = 伸腿，负 = 收腿） → `sysid_pre[F/B] = pre_n × leg_jac[0][*]`
- 输出（VOFA 腿行）：列 28 = 当前预压沿腿力 N；列 23~26 = 四个髋的预压分量 Nm；列 27 = 削平计数
- 调用链：`actuationTask → Sysid_Mode_Run() → DR16_Snapshot → pre_n → 腿分支预压 → 快照 → 发送泵 → VOFA`
- 反向固化：读列 28 的值 → 填回 `SYSID_PRELOAD_L_N/R_N` → `SYSID_PRELOAD_TUNE` 改 0 → 重新编译

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列；滚轮只在测试模式的腿用例里被读取，不影响其他模式（`task_policy.c` 的 height_cmd 只在非测试模式用）
- 轮用例、baseline 用例不加预压（`preload_n` 记 0）

**待台架**

- 滚轮转到哪个方向是"收腿"（双向可试，不会做反）
- 定下来的预压力值需要作者确认后固化
- 若滚轮到底（150 N ≈ 21 Nm/髋）仍压不动腿，说明不是力不够，而是**腿正顶在机械限位上/解算姿态接近奇异**，要换思路（机械调整或改激励方式）

---

## 变更 46 · 预压固化 6 Nm/髋（作者拍板，关闭滚轮实时模式）

**背景**：变更 45 的滚轮实时调节已用于台架定值；作者决定"按 6 Nm 来"，并指出换算误差（雅可比随姿态变化）。

**改动**（`imcalib/Sysid/sysid_mode.c` 文件顶部）

| 宏 | 原值 | 新值 |
| --- | --- | --- |
| `SYSID_PRELOAD_L_N` / `SYSID_PRELOAD_R_N` | -34 | -43（6 Nm）→ -51（7 Nm）→ -58（8 Nm）→ **-54（7.5 Nm，作者最终定值）** |
| `SYSID_PRELOAD_TUNE` | 1（滚轮实时调） | **0（用固定宏）** |
| 换算注释 | 补一句"雅可比随姿态变，实际力矩以帧列 23~26 为准" | — |

`SYSID_PRELOAD_TUNE_N`（滚轮到底 = 150 N）保留，随时可把 `SYSID_PRELOAD_TUNE` 改回 1 再用滚轮试。

**输入 / 输出 / 调用链**

- 输入：`SYSID_PRELOAD_L_N/R_N = -43 N`
- 计算：`sysid_pre[F/B] = -43 × leg_jac[0][*]`（左右腿同值）→ 与激励相加 → ±20 Nm 限幅 → `Dm_Send_Torque()`
- 输出：帧列 23~26 = 四个髋的**实际**预压力矩 Nm（随姿态的雅可比浮动）→ 作者可直接读数核对是否 ≈ ±6 Nm
- 滚轮在 TUNE=0 时不再被读取（`Dial` 分支编译掉）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列

**待台架**

- 列 23~26 的实际值是否 ≈ −6 Nm（若非，说明该姿态下雅可比与平均 0.138 差别较大，需按实测值反推沿腿力）
- 6 Nm 能否把腿压到 0.22~0.25 m；若不能，优先用机械方式把腿放到中段

---

## 变更 47 · 两条腿同步激励（作者要求"两支腿一起测试"）

**背景**：原表每次只激励一条腿的一个电机（target 0~3 依次），对侧腿只吃预压。作者要求两条腿一起测。改动是**对称激励**：激励某一路时，对侧腿的对应电机给同一个逻辑力矩值（`0↔2`、`1↔3`）。

**为什么是对的**：左右电机在驱动边界已按 `dm_sign` 处理，两条腿在解算里用的是同一套镜像后的坐标，所以**同一个逻辑力矩值 = 镜像同向的物理动作** ✓ 两腿受力对称，机体只受竖直合力、不产生偏转。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | 新增宏 `SYSID_EXCITE_BOTH`（默认 **1**）；腿分支激励赋值后加一行 `tau_cmd[target ^ 2] = tau_cmd[target]`；文件顶部加注释 |
| `md/sysid/sysid-delivery.md` | §4.1 `torque_step` 力矩说明标注"两条腿的对应电机同步" |

**输入 / 输出 / 调用链**

- 输入：`run->target`（0=前左 1=后左 2=前右 3=后右）、`sysid_target()` 的激励值、`SYSID_EXCITE_BOTH`
- 输出：`tau_cmd[target] = 激励`，`tau_cmd[target ^ 2] = 同值` → 再叠加预压 → ±20 Nm 限幅 → `Dm_Send_Torque()`
- 帧记录不变：列 5~8 = 四路净力矩 → 训练端能看到"两路同值"的实际下发值 ✓
- 关掉方式：`SYSID_EXCITE_BOTH = 0` 即回到单腿激励

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列；轮用例不受影响（只改腿分支）

**待台架**

- 对称激励下机体是否稳定（两腿同向发力 → 竖直合力；若悬吊有弹性会上下晃）
- 与单腿激励的数据质量对比（单腿激励通道分离更干净；双腿同时可以一次拿两倍数据）

---

## 变更 48 · 帧扩到 33 列：补输入输出时间戳 + 腿解算导数（作者要求"所有测试数据都要有"）

**背景**：作者要求 VOFA 上有全部测试相关数据（输入输出时间戳、各电机输出力矩、解算得到的腿部数据）。原 31 列缺 **DM 反馈到达时刻** 和 **腿长/腿摆角速度**（交接文档要求回放比对"值 + 导数"）。同时把 4 列预压分量挤出帧（净力矩列 5~8 已含预压，`preload_n` 列 30 保留总量，够用）。

**新布局（33 列 / 136 B / 66 kB/s，占 921600 的 71.6%）**

| 列 | 内容 | 变化 |
| --- | --- | --- |
| 0~22 | kind / seq / phase / t_cmd / 四路力矩 / 四路髋角 / 四路角速度 / 腿长摆角 L,R / t_rx_whl | 不变 |
| **23/24** | **DM 反馈最近接收时刻 hi/lo** | 新增 |
| **25/26** | **左腿长速度 / 左腿摆角速度** | 新增 |
| **27/28** | **右腿长速度 / 右腿摆角速度** | 新增 |
| **29** | 削平计数 clamp_cnt | 原位（原 27） |
| **30** | 预压沿腿力 preload_n | 原位（原 28） |
| 31/32 | 轮丢帧 / 腿丢帧累计 | 原 29/30 |

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_log.h` | `SYSID_LOG_FRAME_N` 31→33；注释块重写；结构体去掉 `preload[4]`，新增 `t_dm_rx_ns` / `d_leg[2]` / `d_pitch[2]` |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()` 尾部重排（kind=1 用 23~30，kind=3/5 保留轮数据） |
| `imcalib/Sysid/sysid_mode.c` | `sysid_fill_fb()` 增加：四个 DM 反馈 `rx_ns` 取最新；四个解算导数；快照填值改为不再写 `preload[i]` |
| `tools/sysid_export.py` | `FRAME_FLOATS` 31→33（二进制解析按帧尾定位，长度必须同步） |
| `md/sysid/sysid-delivery.md` | §1.1/§1.2/§1.3.3 同步到 33 列 |

**输入 / 输出 / 调用链**

- 输入：`dm_motor_feedback[i].rx_ns`、`leg_l/r.output.d_virtual_leg_length`、`d_virtual_leg_angle`
- 输出：VOFA 帧 23~30 列；导出 CSV 的列名与顺序**不变**（腿 CSV 只取 3/4/5~8/17~20），无需改训练端契约
- 调用链：`Leg_State_Update()（commTask）→ leg_solver 算出 d_*` → `Sysid_Mode_Run() → sysid_fill_fb()` → 快照 → 发送泵 → `assemble_frame()` → DMA

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 导出自测：`py tools/sysid_export.py --selftest` 全项通过
- 带宽：136 B × 500 Hz = 68 kB/s（921600 的 **73.8%**，含帧尾）——比原来 128 B 高 6 个百分点，仍是安全范围

**待台架**

- 33 列下 VOFA+ 的通道数要改成 33（否则显示会错位）
- 旧窗口的通道名建议按新表重命名

---

## 变更 49 · 预压回到 8 Nm + 新增《VOFA 数据对照表》

**背景**：作者要求把预压给回 8 Nm；并指出 VOFA 列太多显得乱，需要一份给训练端看的完整对照文档。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | `SYSID_PRELOAD_L_N/R_N`：-54（7.5 Nm）→ **-58（8 Nm，作者最终值）** |
| `md/sysid/vofa-channel-map.md` | **新增**：33 列逐列对照（列号/名称/单位/来源），按行类型的差异、标记/心跳行、时间戳还原、CSV 字段来源对照、快速自查表、"看起来不对"的常见解释 |
| `md/sysid/sysid-delivery.md` | §1.2 的长表改为"快速索引 + 指向新文档"，避免两处维护 |
| `md/AGENTS.md` | 文件树补 `vofa-channel-map.md` |

**输入 / 输出 / 调用链**

- 预压：同变更 43~46，值改回 -58 N（≈8 Nm/髋），帧列 5~8 记净力矩、列 30 记预压沿腿力
- 文档：`vofa-channel-map.md` 是列定义的**单一出处**，`sysid-delivery.md` §1.2 与 `imcalib/Sysid/sysid_log.h` 注释块必须与它保持一致（改列必须同步这三处 + `tools/sysid_export.py` 的 `FRAME_FLOATS`）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 新文档内容与 `sysid_log.h` 注释块逐列核对一致

**待台架**

- 8 Nm 预压能否把腿压到 0.22~0.30 m（作者观察列 17/19）

---

## 变更 50 · 发送余量 + 串口卡死自恢复（作者反馈"偶尔 VOFA 卡住没数据"）

**背景**：作者反馈 VOFA 偶尔卡住没数据。查发送链路：帧 136 B × 500 Hz = 68 kB/s，占 921600 的 **73.8%**，而单帧在线上要 1.476 ms、发送泵每 2 ms 才被调一次 → 只剩 0.52 ms 余量，受 FreeRTOS 1 ms tick 抖动影响会出现跳周期 → 环形缓冲堆积丢帧；而"完全卡住"最可能是 **HAL 发送状态机卡死**（丢一次 TC 中断，`gState` 永远 BUSY，泵会永远提前返回）。**不是波特率问题**（波特率不匹配会乱码/列错位，不会"偶尔停一下"）。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_log.h` | 新增 `SYSID_TX_DIV`（默认 **4** = 250 Hz）与 `SYSID_TX_STALL_MS`（默认 50 ms）；extern `sysid_log_stall_cnt` |
| `imcalib/Sysid/sysid_log.c` | ①分频改用 `SYSID_TX_DIV`；②发送前加**卡死看门狗**：连续忙 ≥50 ms 就 `HAL_UART_AbortTransmit()` 强制复位状态机并计数；③新增 `sysid_log_stall_cnt` |
| `imcalib/Sysid/sysid_mode.c` | 心跳行新增列 13 = 串口卡死自恢复次数 |
| `md/sysid/vofa-channel-map.md` | 心跳行列 13 含义；§1.1 发送频率说明 |
| `md/sysid/sysid-delivery.md` | §1.1 发送频率 500 → 250 Hz |

**输入 / 输出 / 调用链**

- 输入：`send_div`、`VOFA_UART->gState`、`HAL_GetTick()`
- 输出：常态 250 Hz 发送（占链路 37%）；卡死超时则 abort 并恢复发送；计数进心跳行列 11（忙跳过）/列 13（卡死恢复）
- 调用链：`commTask → Sysid_Log_Send_Pump() → gState 检查/看门狗 → HAL_UART_AbortTransmit(超时) → ring 取帧 → assemble_frame → Clean_Tx → HAL_UART_Transmit_DMA`
- 250 Hz 仍高于交接契约的 ≥200 Hz 下限

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 33 列

**待台架**

- 250 Hz 下 VOFA 是否稳定（稳定后可把 `SYSID_TX_DIV` 改回 2 试 500 Hz）
- 心跳列 13 是否出现非 0（出现即说明确实发生过 HAL 状态卡死）
- VOFA+ 侧的"记录到文件"功能本身也可能拖慢接收，建议卡顿时先关记录试

---

## 变更 51 · 测试方式改为「位置扫描」（作者定：real2sim 位置跟踪，而非直接力矩）

**背景**：作者/训练端确认单关节开环力矩辨识意义不大（参数不准），轮腿的 gap 主要来自并联耦合。改为 **real2sim**：真机驱动关节跟踪预设轨迹并录包，MuJoCo 用同一套控制律跟踪同一条轨迹，比对角度曲线后调仿真参数。因此下位机要提供的是**位置跟踪 + 记录（指令/实测/力矩）**，而不是直接下发力矩。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①新增测试方式开关 `SYSID_MODE`（`SYSID_MODE_POSE`=0 位置扫描，默认；`SYSID_MODE_TORQUE`=1 原力矩激励）；②位置扫描参数 `SYSID_POSE_RAMP_S`(0.5s)/`SYSID_POSE_HOLD_S`(2s)/`SYSID_POSE_KP`(25)/`SYSID_POSE_KD`(300)；③新增模板 `TPL_POSE` 与姿态表 `sysid_pose_runs[]`（大腿角 {-0.15,0,+0.15} × 虚拟小腿角 {2.50,2.80,3.10} 共 9 个姿态）；④腿分支新增位置扫描实现：线性斜坡到目标 → 组装 RL 动作 → 调 **`RL_Torque_Compute()`**（复用 RL 同一条力矩链路）→ 下发；⑤`Sysid_Mode_Init()` 里初始化虚拟关节 PD（只保留腿的位置环，轮增益归零）；⑥力矩模式与预压代码全部用 `#if SYSID_MODE` 保留 |
| `imcalib/Sysid/sysid_log.h` | 帧 33 → **35 列**（144 B）；新增列 33/34 = `thigh_tgt` / `shank_tgt`；结构体加 `pose_tgt[2]` |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()` 写列 33/34 |
| `tools/sysid_export.py` | `FRAME_FLOATS` 33 → 35 |
| `md/sysid/controller-spec-for-mujoco.md` | **新增**（子代理写）：控制律复刻说明书——PD 精确离散形式（D 不除 dt）、增益/周期/限幅/环绕、虚拟关节→电机映射、MIT 帧 kp=kd=0 证据、轨迹格式、气弹簧与摩擦两个坑 |

**位置扫描的控制链**

- 目标：`(大腿角, 虚拟小腿角)`，线性斜坡 0.5 s 从**上一姿态目标**滑到本姿态目标，再保持 2 s
- 控制器：`τ_i = kp·wrap180(q_des−q) + kd·(e[k]−e[k−1])`，kp=25、kd=300（等效阻尼 0.6 Nm·s/rad），500 Hz
- 映射：`τ_前髋 = τ_大腿 + τ_小腿×vshank_jac[1]`、`τ_后髋 = τ_小腿×vshank_jac[0]`，电机侧 kp=kd=0（纯力矩执行），限幅 20 Nm
- 记录：帧列 33/34 = 当前目标角（指令），列 9~12 = 实测髋角，列 5~8 = 实际下发力矩

**核对**

- 编译三种配置全部 0 fail / 0 warn：位置扫描（默认）、`-DSYSID_MODE=1`（力矩）、`-DSYSID_ENABLE=0`
- 导出工具自测通过
- 未动任何物理量；力矩模式与预压机制完整保留（改 `SYSID_MODE` 即可回退）

**待台架**

- `SYSID_POSE_KP/KD` 是否合适（kp=25 偏软，静差可能 0.2~0.4 rad；跟踪不好就加 kp，kd≈kp×12）
- 9 个姿态是否都可到达（撞限位的姿态数据判无效）
- 气弹簧造成的系统偏置需要训练端在仿真里等效加入，否则角度曲线不可能重合

---

## 变更 52 · 加「手动 PD 测试模式」+ 姿态表只跑 2 遍 + 降增益

**背景**：作者要求：先把 kp/kd 降下来；先用**手动**方式测 PD 闭环效果；测试次数改为 **2 次**，不要一直循环。

**改动**（`imcalib/Sysid/sysid_mode.c`）

| 项 | 原 | 新 |
| --- | --- | --- |
| 测试方式 | 位置扫描 / 力矩 | 新增第三种 **`SYSID_MODE_MANUAL`（默认）**：目标角由遥控给，用来手测闭环 |
| `SYSID_POSE_KP` / `KD` | 25 / 300 | **10 / 120**（等效阻尼 0.24 Nm·s/rad） |
| 姿态表遍数 | 一直循环 | **`SYSID_LOOP_CNT = 2`**：整表跑 2 遍后自动停机（状态码 **5 = 表跑完**），改 0 可恢复一直循环 |
| 手动模式摇杆 | — | **左摇杆上下（ch3）→ 大腿角 ±0.6 rad；滚轮（wheel）→ 虚拟小腿角 ±0.6 rad**，零点 = 进入测试模式时的实测姿态 |
| 手动模式表 | — | 单个 3600 s 的长 run（不进姿态表、不循环、不会被遍数停机） |

**输入 / 输出 / 调用链**

- 手动：`DR16_Snapshot()` → `sysid_stick()`（去死区+限幅→[-1,1]）→ `thigh_t = 进入姿态 + 摇杆×范围`、`shank_t = 进入姿态 + 滚轮×范围` → 组装 RL 动作 → `RL_Torque_Compute()` → 4 路腿力矩 → CAN
- 记录不变：列 33/34 = 当前目标角（手动模式也记，便于回看），列 9~12 实测角，列 5~8 实际力矩
- 停机：`loop_cnt` 达到 `SYSID_LOOP_CNT` → 只发零力矩 + 心跳（状态码 5）

**核对**

- 四种编译配置全部 0 fail / 0 warn：手动 PD（默认）、位置扫描（`-DSYSID_MODE=0`）、力矩（`-DSYSID_MODE=1`）、关闭测试（`-DSYSID_ENABLE=0`）

**待台架**

- 手动模式下 kp=10 是否偏软（静差大 → 加 kp，kd≈kp×12）
- 手动模式的角范围 ±0.6 rad 是否合适（撞限位就改 `SYSID_MAN_*_RANGE`）

---

## 变更 53 · 去掉 D 项 + 删除新增的「手动 PD 模式」（作者：手动测试用原来的左上模式即可）

**背景**：作者指出 ①D 项去掉；②"手动模式不就是原来的 RL 测试（左拨杆上位）吗，为什么要新增"。核查确认：**左上（不动右拨杆）= 正常手动模式**，其链路是 `task_policy.c` 的 `Manual_Lock_On_Enable()`（使能瞬间锁存当前姿态为 `base_action`）+ 摇杆偏移 → `RL_Torque_Compute()`（同一套虚拟关节 PD + 雅可比映射 + 限幅），**没有策略在环**（`task_policy.c` 的 `ctrl_task_body()` 不调用策略推理）。也就是说它已经是"零点=使能姿态、摇杆给目标偏移"的手动 PD 测试，变更 52 新增的那套是重复实现。

**改动**（`imcalib/Sysid/sysid_mode.c`，全部为删除/降值）

| 项 | 原 | 新 |
| --- | --- | --- |
| `SYSID_POSE_KD` | 120 | **0**（D 项去掉） |
| `SYSID_MODE_MANUAL`（测试方式第 3 种） | 有，且为默认 | **删除**，默认回到 `SYSID_MODE_POSE` |
| `SYSID_MAN_THIGH_RANGE` / `SYSID_MAN_SHANK_RANGE` / `SYSID_MAN_DEADBAND` | 有 | **删除** |
| `sysid_stick()` 辅助函数 | 有 | **删除** |
| 手动长 run 表 `sysid_manual_runs[]` | 有 | **删除** |
| 腿分支里的手动目标计算 | 有 | **删除**，只保留位置扫描的斜坡逻辑 |

**现在的两套测试方式**

- `SYSID_MODE_POSE = 0`（默认）：位置扫描，自动跑姿态表 `SYSID_LOOP_CNT = 2` 遍后停机（状态码 5）
- `SYSID_MODE_TORQUE = 1`：力矩激励（原方案）
- 手动 PD 手测：**不进测试模式**，左拨杆上位即可（增益用 RL 模型参数表里的值，不在本模块里）

**核对**

- 编译：位置扫描（默认）/ 力矩 / 关闭测试，三种配置全部 `0 fail / 0 warn`
- `grep` 确认 `MANUAL` / `sysid_stick` / `SYSID_MAN_*` 无残留

**待台架**

- 位置扫描用 `SYSID_POSE_KP = 10`、`KD = 0`：D 去掉后若出现摆动/振荡，需要降 kp；采集前请把最终值固定并告知训练端（手动模式用的是另一套增益，不要混）
- 手动模式（左上）的增益是 RL 模型参数表的 `p_gains`（大腿 3.5 / 小腿 15.5，D=0），比位置扫描软

---

## 变更 54 · 去掉 VOFA_LAYOUT，只留一套 VOFA 输出（作者：不要多套布局）

**背景**：作者要求去掉 `VOFA_LAYOUT` 那套机制与多套 VOFA 输出，通道直接改成测试相关数据，后续只改参数即可。另外大腿角区间改为 45°~145°。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/Vofa_send.h` | **删除 `VOFA_LAYOUT` 宏**（保留 `VOFA_MAX_CH`、`VOFA_PORT`） |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` 删除 `#if VOFA_LAYOUT == 1`（虚拟关节 PID 视图）整个分支，只保留测试相关通道那一套（力矩 / 髋角 / 腿长摆角 / 轮 / 时间戳）；注释改为"通道内容直接改这个函数" |
| `imcalib/Sysid/sysid_mode.c` | 姿态表大腿角改为 **45° / 95° / 145°**（0.7854 / 1.6581 / 2.5307 rad）；`SYSID_POSE_RAMP_S` 0.5 → **1.5 s**（跨度大使斜坡给足时间） |

**现在的 VOFA 输出只有两种（各自一条路径，不再有布局开关）**

- **正常模式**：32 路 FireWater 调试帧（测试相关通道），改通道就改 `task_comm.c` 的 `Robot_Control_Send_Vofa()`
- **测试模式**：35 列 JustFloat sysid 帧（列定义见 `md/sysid/vofa-channel-map.md`），改列就改 `Sysid/sysid_log.h` + `sysid_log.c` + `tools/sysid_export.py` 的 `FRAME_FLOATS`

**核对**

- 编译：`-DSYSID_ENABLE=1` 与 `-DSYSID_ENABLE=0` 均 105 文件 0 fail / 0 warn
- `grep VOFA_LAYOUT` 在 `imcalib/` 下已无残留

**待台架**

- 大腿角 45°~145° 的**坐标约定核对**：固件里 `thigh_angle = wrap(前髋电机角 + π)`，即 VOFA 列 9 读数 + π；先用列 33（目标）与列 9 对照确认，不一致就按实测改表

---

## 变更 55 · 斜坡改成标准线性插值 + 小腿角区间改 2.3~3.0

**改动**（`imcalib/Sysid/sysid_mode.c`）

| 项 | 原 | 新 |
| --- | --- | --- |
| 姿态表虚拟小腿角 | 2.50 / 2.80 / 3.10 | **2.30 / 2.65 / 3.00**（大腿角仍 45°/95°/145°） |
| 斜坡实现 | 每周期用 `sysid_pose_prev` 做插值（prev 每周期被改写成中间值 → 形状不是直线，前慢后快） | 新增 `sysid_pose_from[2]`：**每段开头锁存起点**（= 上一段目标；首段 = 进入时实测角），然后 `目标 = 起点 + (本段目标 − 起点) × min(1, t/1.5s)` —— **标准线性斜坡**，到 1.5 s 精确等于目标 |
| 段尾的"记住本段目标" | 有 | 删除（起点改在段首锁存，`sysid_pose_prev` 只在段首写入本段目标） |

**验证设计依据**：帧列 33/34 记录的是**每周期实际的斜坡值**，训练端按记录值回放，所以斜坡形状不影响比对；但改成标准直线后，目标序列可以由姿态表 + 起点完整复现。

**核对**：`-DSYSID_ENABLE=1`（位置扫描）与 `-DSYSID_MODE=1`（力矩）均 105 文件 0 fail / 0 warn。

---

## 变更 56 · 斜坡函数化：lowpass → simple-function（新增 Ramp_*）

**背景**：作者要求斜坡不要在测试模块里自己做，而是做成**可复用函数**，放在原 `lowpass` 文件里，并把文件改名为 `simple-function`，用正确的斜坡函数形式。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/lowpass.c/h` → **`simple-function.c/h`** | 文件改名（低通 API `Lowpass_*` 不变） |
| `simple-function.h/.c` | 新增斜坡类型与函数：`ramp_t {out, rate}`、`Ramp_Init(r, rate)`、`Ramp_Update(r, target, dt)`、`Ramp_Reset(r, value)`；**形式 = 斜率限制**：每周期最多 `rate×dt`，到目标直接等于目标（不超调、目标中途变化也正确） |
| `imcalib/Algorithm/lqr_balance.h` | `#include "lowpass.h"` → `"simple-function.h"` |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | 工程文件项 `lowpass.c` → `simple-function.c` |
| `imcalib/Sysid/sysid_mode.c` | 删除自写的插值斜坡（`sysid_pose_from`/`sysid_pose_prev`）；改为 `thigh_t = Ramp_Update(&sysid_ramp_th, run->amplitude, 0.002f)`、`shank_t` 同理；首次进入时用 `Ramp_Reset()` 对齐实测角；`SYSID_POSE_RAMP_S`（时间）→ `SYSID_POSE_RAMP_RATE = 1.2f`（rad/s） |
| `md/AGENTS.md` | 文件树同步为新文件名 |

**输入 / 输出 / 调用链**

- 输入：目标角 `run->amplitude` / `run->amp2`、`dt = 0.002 s`、`rate = 1.2 rad/s`
- 输出：限斜率后的目标角 → 组装 RL 动作 → `RL_Torque_Compute()` → 4 路腿力矩；帧列 33/34 记录该斜坡值
- 调用链：`Sysid_Mode_Run() → Ramp_Update()（simple-function.c）→ act_buf → RL_Torque_Compute()`

**核对**：三种配置（位置扫描默认 / 力矩 `-DSYSID_MODE=1` / 关闭 `-DSYSID_ENABLE=0`）均 105 文件 0 fail / 0 warn；`grep lowpass.h`、`grep sysid_pose_from` 无残留。

---

## 变更 57 · 重构 `Robot_Control_Send_Vofa()`（纯整理，行为不变）

**动机**：函数内联组装 32 通道、逻辑分组不清晰，作者反馈"太乱了"。

**改动**（`imcalib/task/task_comm.c`，只重构，不改任何通道含义）

| 变化 | 说明 |
| --- | --- |
| 拆出 `Vofa_Fill_Status()` | ch0~2：在线掩码 / 解算有效 / 策略号 |
| 拆出 `Vofa_Fill_Leg()` | ch3~14：腿指令力矩 / 零点后角 / 腿长 / 腿摆角 |
| 拆出 `Vofa_Fill_Wheel()` | ch15~23：轮指令电流 / 转速 / 编码器 / 实际电流 / 温度 |
| 拆出 `Vofa_Fill_BusDiag()` | ch24~31：总线 TX 间隔/条数/丢帧 + RX 间隔/时刻拆分 |
| 主函数 | 分频 + SYSID 早退 + 依次调 4 个填充函数 + `Vofa_Send(dbg, 32u)` |
| 通道表注释 | 主函数上方新增 32 路逐列说明（列号 / 含义 / 单位 / 来源） |
| `static` 局部变量 | `rx_prev` / `tx_prev_whl` / `tx_prev_leg` 从主函数搬到 `Vofa_Fill_BusDiag()` 内部，作用域更小 |

**输入 / 输出 / 调用链**：与变更 54 完全一致，纯重构，无行为变化。
- 输入源：`motor_state` / `rl_control` / `leg_l/r` / `dji_motor_feedback` / `imu_state` / `DR16_Online()` / `Can_Bus_Tx_Pop()` / `Can_Bus_Tx_Drop_Count()` / `machine->dji_sign/dji_bus/dm_bus`
- 输出：`Vofa_Send(dbg, 32u)` → 当前 `VOFA_PORT` 串口

**核对**：`-DSYSID_ENABLE=1`（默认）与 `-DSYSID_ENABLE=0` 均 105 文件 0 fail / 0 warn。

---

## 变更 58 · 切回小机器（`MACHINE_DEFAULT` → `MACHINE_ID_LOCAL`，作者：测小机器 LQR）

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/machine_config.h` | `MACHINE_DEFAULT`：`MACHINE_ID_CHUANLIANTUI` → `MACHINE_ID_LOCAL` |
| `imcalib/user-lib/machine_config.c` | 未改；`machine` 初值 `&machine_table[MACHINE_DEFAULT]` 自动指向小机器表 |

**为什么只改这一行**：机器相关的量（型号/减速比/限幅/极性/零点/总线/腿几何/腿长区间）全部在两份表里，切换机器只切指针。

**切换后实际生效参数（大机器 → 小机器）**

| 项 | 大机器 `chuanliantui` | 小机器 `local-m2006-j4310` |
| --- | --- | --- |
| 轮型号/减速比/限幅 | M3508 / 15.5 / ±4.8 N·m | **M2006** / 36.0 / ±1.8 N·m |
| 腿电机 (DM) | J8009P：±π / 45 rad/s / 54 N·m | **J4310**：±π / 30 rad/s / 10 N·m |
| 腿力矩限幅 | 20 N·m | 10 N·m |
| 腿总线 | FDCAN1 ×4 | **FDCAN1 左腿 + FDCAN3 右腿** |
| 轮总线 | FDCAN3 | **FDCAN2** |
| DM 零点 | {0.476998, 1.974491, …} | {-0.03, -0.04, -0.038, -0.023} |
| 腿几何 lu / lg | 0.21 / 0.25 | 0.13087 / 0.15240 |
| 腿长区间 | 0.14 ~ 0.34 | 0.10 ~ 0.20 |

**输入 / 输出 / 调用链**
- 定义：`machine_config.h` 的 `MACHINE_DEFAULT` → `machine_config.c` 的 `machine` 指针 → 全工程只读 `machine->`
- 消费者：`dm.c`（极性/零点/量程/限幅/总线）、`dji.c`（型号→`per_raw`、减速比、总线）、`leg_solver.c`（lu/lg/腿长区间/phi0）、`lqr_balance.c`（`leg_len_min/max` 参与 LQR 投入判定与腿长目标夹取）、`rl_torque.c`（限幅）、`task_comm.c`（VOFA 换算）
- 运行时切换口：`Machine_Select(id)`（`main.c` 上电调用 `Machine_Select(MACHINE_DEFAULT)`）

**核对**：armcc 全量 105 文件 0 fail / 0 warn（默认配置）。

**已确认不需要改的**：**FDCAN 波特率不用动**。`can_bus.c:91-92` 的发送模板是 `BitRateSwitch=FDCAN_BRS_OFF` + `FDFormat=FDCAN_CLASSIC_CAN`，发出的全是**经典 CAN 帧**，只走仲裁段（nominal）时序；三路 FDCAN 的 nominal 都是 `24 MHz / (3×8) = 1 Mbps`（HSE 24 MHz 直供 FDCAN）。FDCAN1 上那个 4 Mbps 的 DataPrescaler 只影响 FD 数据段，对经典帧**不起作用**。（待台架：若小机器 J4310 曾被达妙上位机改成非 1 Mbps，则以电机实际波特率为准。）

**待台架 / 未决**
- LQR K 表拟合域 0.13~0.23 m，小机器腿长区间 0.10~0.20 m：站姿腿长低于 0.13 m 时增益为外推值。
- `lqr_balance.c:9` 的 `LQR_WHEEL_R = 0.04f` 硬编码，未进配置表；小机器轮径若不等于 0.04 m，速度估计会成比例偏。
- 测试模式（左上 + 右中）的姿态表仍是按大机器几何标的，小机器上不要进。

---

## 变更 59 · 关测试开关 + FDCAN1 数据段改回原值 + VOFA 换成 LQR 观测帧（作者：开始测小机器 LQR）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Sysid/sysid_config.h` | `SYSID_ENABLE`：1 → **0**（测试代码整块不参与编译，策略仲裁回到 LQR / 手动两路） |
| `Core/Src/fdcan.c` | FDCAN1 `DataPrescaler` 1→3、`DataTimeSeg1` 4→5、`DataTimeSeg2` 1→2 |
| `CtrBoard-H7_ALL.ioc` | 同上三行（与 CubeMX 保持同源，重新生成不会变回 4 Mbps） |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` 的 32 通道内容整块换成 LQR 观测；函数上方补通道表注释；删掉 `const pid_t *pid;` 与 `sysid_wheel_cmd_raw` 的 extern 引用 |

**为什么改 FDCAN1**：作者要求把大机器那次的改动还原。注：`can_bus.c:91-92` 发送模板是 `BRS_OFF + CLASSIC_CAN`，仲裁段 1 Mbps 不变，这 3 行只影响 FD 数据段（当前固件用不到）；改回去是为了与 CubeMX 配置、与两机器一致的原始状态对齐。

**VOFA 新帧（32 通道 / JustFloat / 200 Hz，`vofa_div < 5u` 分频不变）**

| 通道 | 含义 | 来源 |
| --- | --- | --- |
| ch0 | 在线掩码：IMU / 遥控 / 髋 4 / 轮 2 | `imu_state.online`、`DR16_Online()`、`motor_state.dm.online[]`、`motor_state.dji.online[]` |
| ch1 | 状态位：使能 / 跌倒 / 左腿有效 / 右腿有效 | `robot_state.*`、`leg_l/r.output.valid` |
| ch2 | 策略号 0 手动 / 1 LQR | `ctrl_strategy` |
| ch3~6 | 四髋位置（零点后 rad） | `motor_state.dm.pos_zero_rad[]` |
| ch7~10 | 左腿：大腿角 / 虚拟小腿角 / 虚拟腿摆角 / 腿长 | `leg_l.output.*` |
| ch11~14 | 右腿：同上 | `leg_r.output.*` |
| ch15~16 | 腿长目标（左/右 m） | `lqr_state.leg_len_tgt[]` |
| ch17~19 | 俯仰角 (rad) / 俯仰角速度 (rad/s) / 前进速度 (m/s) | `lqr_state.x[THB / DTHB / DS]` |
| ch20~23 | LQR 输出 (N·m)：左轮 / 右轮 / 左髋 / 右髋 | `lqr_state.u[WL / WR / BL / BR]` |
| ch24~25 | 轮转速 (rad/s) | `motor_state.dji.vel_rad_s[]` |
| ch26~27 | 轮实测电流 (A) | `dji_motor_feedback[].current_raw / 819.2` |
| ch28~31 | 髋力矩反馈 (N·m)：前左/后左/前右/后右 | `dm_motor_feedback[].trq_nm` |

**输入 / 输出 / 调用链**
- 输入：上面表里各来源（都在 `commTask` 之前由 `Dm_Parse/Dji_Parse/Motor_State_Update/Leg_State_Update` 刷好；LQR 的 `lqr_state` 由 `actuationTask` 的 LQR 分支刷新）
- 输出：`Vofa_Send(dbg, 32u)` → `VOFA_PORT`（当前 1 = USART1 @1152000）→ JustFloat
- 调用链：`comm_task_body()` → `Robot_Control_Send_Vofa()`；末尾两段 `Can_Bus_Tx_Pop` 清环保持不变
- `SYSID_ENABLE=0` 后：`task_actuation.c` 的 sysid 分支、`task_comm.c` 的 10 通道轮帧分支整块编译掉；`Sysid/*.c` 编成空单元

**核对**：默认（`SYSID_ENABLE=0`）与 `-DSYSID_ENABLE=1` 均 **105 文件 0 fail / 0 warn**。

**待台架**：小机器上先只看 ch7~14（手搬腿 → 大腿角/小腿角/摆角/腿长是否跟手、左右是否一致），确认后再进 LQR。

---

## 变更 60 · 小机器 LQR 批次 1：清测试残留、隔离时间戳、修复解算竞态与使能链

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/task_comm.c` | VOFA 恢复 2 分频 500 Hz；删除正常链路 CAN 完成环清空与腿速度自检；两腿 `Leg_Solve()` 用调度器锁保护；接入 DM 使能看门狗、故障码和 ch1 bit4~7 使能位 |
| `imcalib/task/inc/robot_control.h` | 删除无人消费的 `leg_debug_history_t` |
| `imcalib/user-lib/dr16.c/h` | 删除过时的三项接收诊断计数 |
| `imcalib/user-lib/can_bus.c/h` | TX 完成登记、完成中断、环形缓冲与统计 API 全部限制在 `SYSID_ENABLE=1`；正常固件直接入发送 FIFO |
| `imcalib/user-lib/dm.c/h`、`dji.c` | RX 纳秒时间戳仅在 sysid 构建启用；新增 DM 使能/故障判定和每台独立 100 ms 使能看门狗 |
| `imcalib/Algorithm/leg_balance.c/h` | 新增 `Leg_Balance_Reset()`，只清四个 PID 的历史与输出，不改参数 |
| `imcalib/task/task_actuation.c` | LQR 投入锁存成功时复位辅助 PID |

**输入 / 输出 / 调用链**
- 正常通信：`comm_task_body()` → `Leg_State_Update()` → `vTaskSuspendAll()` → 两次 `Leg_Solve()` → `xTaskResumeAll()`；高优先级 `actuationTask` 不再读到求解中途的 `valid=0`。
- DM 状态：反馈字节 0 高 4 位 → `Dm_Parse().err_raw` → `Dm_Is_Enabled()` / `Dm_Has_Fault()`；故障码 `0x8~0xE` 进入 `FAULT_MOTOR`。
- DM 看门狗：`Robot_Enable_Update()` 在 `motor_enabled=1` 时调用 `Dm_Enable_Watchdog()`；仅对在线且 `err_raw=0` 的电机按各自计时每 100 ms 重发 `DM_CMD_ENABLE`。
- LQR 投入：`LQR_Enable_Latch()` 返回 1 → `Leg_Balance_Reset()` → 首个控制周期从清零的 PID 历史开始。
- VOFA：`commTask 1 kHz` → 2 分频 → `Vofa_Send(dbg, 32)` 500 Hz；ch1 bit4~7 依次表示左前、左后、右前、右后 DM 的 `err_raw==1`。
- sysid 时间戳：`SYSID_ENABLE=1` 时保留 `Can_Bus_Transmit_Tagged()` → TX 完成回调 → 环形缓冲 → `Sysid_Mode_Run()` 出队；关闭时不登记、不启用 TX complete 中断，DM/DJI RX 中断也不读 `Mono_Ns_Get()`。

**核对**
- 未改 `dm_sign` / `dji_sign`、零点、MIT 量程、镜像、腿几何和 `MACHINE_DEFAULT`。
- VOFA 32 路下标未移动，只扩展 ch1 高 4 位；帧长仍为 132 B，500 Hz 约 66 kB/s。
- Keil AC5 按 `build/CtrBoard-H7_ALL/compile_commands.json` 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- 失能任一 J4310 后确认仍有反馈且 ch1 对应 bit 清零，100 ms 看门狗能重新使能。
- 人为触发 DM `0x8~0xE` 故障码，确认 `FAULT_MOTOR` 置位并停止输出。
- 反复进入 LQR，确认首拍辅助 PID 不再因旧历史产生冲击；竞态修复只完成代码与编译核对，实时行为待台架。

---

## 变更 61 · 小机器 LQR 批次 2：控制频率 500 Hz → 1 kHz

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/inc/robot_control.h` | 新增统一控制周期 `CTRL_DT=0.001f` |
| `imcalib/task/task_actuation.c`、`imcalib/Algorithm/rl_torque.c`、`imcalib/Sysid/sysid_mode.c` | 删除分散的 `0.002f` / `OUTPUT_DT`，统一引用 `CTRL_DT` |
| `CtrBoard-H7_ALL.ioc`、`Core/Src/tim.c` | TIM6 Prescaler 549 → 274，Period 保持 999，对应 275 MHz / 275 / 1000 = 1 kHz |
| `Core/Src/main.c`、`imcalib/user-lib/mono_ns.c` | TIM6 与单调时钟注释同步为 1 kHz |
| `imcalib/Algorithm/leg_balance.h` | 按作者确认，腿长/防劈叉/横滚三个辅助 PID 的 KD 分别由 25000/250/50 改为 0，先使用纯 P，待台架单独标定 D |
| `imcalib/Sysid/sysid_mode.c`、`sysid_log.c/h` | sysid 单周期同步为 1 kHz；心跳改 250 tick 保持 250 ms；`SYSID_TX_DIV=4` 保持 1 kHz commTask → 250 Hz 发送 |

**输入 / 输出 / 调用链**
- 时钟：TIM6 275 MHz → `(Prescaler+1)=275` → `(Period+1)=1000` → 1 kHz 中断 → `ctrl_tick_sem_handle` → `actuationTask`。
- 时间步：`CTRL_DT` → LQR 目标/状态/腿部力控、RL 虚拟关节 PID、sysid 激励时序，所有控制计算使用同一个 1 ms 周期。
- 辅助 PID：腿长、防劈叉、横滚误差 → `pid_calc()`；本批 `kd=0`，D 输出恒为 0，KP 与前馈不变。
- sysid：`Sysid_Mode_Run()` 1 kHz 采样；心跳 `1000×250 tick=250 ms`；发送泵仍由 `commTask 1 kHz / SYSID_TX_DIV 4 = 250 Hz`。

**总线负载估算与核对**
- 小机器腿总线每 1 ms 约 2 发 2 收，经典 CAN 1 Mbps 估算负载约 **52%**。
- 小机器轮总线每 1 ms 约 1 发 2 收，经典 CAN 1 Mbps 估算负载约 **39%**。
- `SYSID_ENABLE=1` 时台架观察 `Can_Bus_Tx_Drop_Count()`；正常固件可临时观察 `HAL_FDCAN_GetTxFifoFreeLevel()`，确认发送 FIFO 不持续归零。
- `SYSID_TX_DIV` 针对通信发送泵，不跟随控制频率翻倍；本批核对后保留 4。

**核对**
- TIM6 `.ioc` 与生成代码数值一致；控制相关硬编码 `0.002f` 已从指定链路清除。
- 未改机器选择、腿长、轮径、极性、零点、量程、镜像和 IMU 轴。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- 示波器或任务计数确认 TIM6/actuationTask 实际为 1 kHz，并检查是否出现信号量积压。
- 观察三路 FDCAN 发送 FIFO、sysid drop 计数和电机在线状态，确认 1 kHz 下无掉帧。
- 三个辅助 PID 当前无 D 阻尼；落地前按既定分通道顺序低限幅验证，D 项后续只能依据台架数据单独恢复。

---

## 变更 62 · 小机器 LQR 批次 3：调试开关、限幅、K 表腿长域与轮径入表

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h/c` | 新增全局 `lqr_debug`；默认符号保持现状；轮/髋/腿长 PID 三个输出开关；轮/髋运行时限幅；K 表域 0.13~0.23 m |
| `imcalib/Algorithm/leg_balance.c` | 关闭轮通道时轮输出为 0；关闭髋通道时 `Tp=0`；关闭腿长 PID 时足端力只留固定前馈；PID 始终继续计算 |
| `imcalib/user-lib/machine_config.h/c` | `machine_cfg_t` 新增独立轮半径 `wheel_r`；大机器 0.04 m（占位待实测），小机器 0.03 m（Leg2_v1 建模值） |

**`lqr_debug` 默认值与行为**

| 字段 | 默认值 | 行为 |
| --- | ---: | --- |
| `vel_leg_comp_sign` | `-1.0f` | `wheel_vel + sign×d_virtual_leg_angle - omg_pitch`；默认与改前完全相同，`+1` 仅供台架 A/B |
| `wheel_enable` | `1` | 关闭只把最终 DJI 输出置 0 |
| `hip_enable` | `1` | 关闭只把左右 `Tp` 置 0 |
| `len_pid_enable` | `1` | 关闭后两腿 `F` 只保留 `LEG_BALANCE_F_FEEDFORWARD`，腿长与横滚 PID 输出都不下发 |
| `trq_max_wheel` | `machine->dji_trq_clamp` | 小机器默认 1.8 N·m，替代原 1.5 N·m 宏 |
| `trq_max_hip` | `5.0f` | 替代原 2.0 N·m 宏，低于 J4310 10 N·m 上限 |

**输入 / 输出 / 调用链**
- 调试器 Watch → `lqr_debug` → `LQR_State_Update()` 的腿摆速度补偿、`LQR_Control_Update()` 一级限幅、`Leg_Balance_Compute()` 通道门与二级限幅；未增加 VOFA 通道。
- 腿长有效域：`max(machine->leg_len_min, 0.13)` 到 `min(machine->leg_len_max, 0.23)` → `LQR_Enable_Latch()` 投入判定与 `LQR_Target_Update()` 目标夹取。小机器实际为 **0.13~0.20 m**。
- 轮速度：DJI 输出轴角速度 × `machine->wheel_r` → 轮心线速度；轮径是机器表独立字段，没有叠加进腿长。

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-20** 明确确认小机器轮径从开源 `Leg2_v1` 查取，并确认速度补偿运行时 A/B 开关。
- 依据：`Leg2_v1/轮腿上交建模MATLAB/WBR_modeling.mlx` 的小机器参数组写明 `R_w_ac=0.03 m`；大机器活动参数组为 `0.04 m`。
- 实际写入：`machine_config.c` 大机器 `wheel_r=0.04f`（占位、待实测），小机器 `wheel_r=0.03f`；原 `lqr_balance.c` 固定宏 `0.04f` 删除。
- 符号：`vel_leg_comp_sign=-1.0f` 对应改前 `wheel_vel - d_virtual_leg_angle - omg_pitch`，默认行为不变；`+1.0f` 不作为当前物理结论，只允许台架比较。

**核对**
- `MACHINE_DEFAULT=MACHINE_ID_LOCAL` 不变；杆长、机器腿长表、极性、零点、MIT 量程、镜像、`+LEG_PI` 与 IMU 五个轴宏均未改。
- 开关关闭时 PID 仍更新，仅门控最终物理输出，重新开启不会因暂停计算产生额外历史跳变。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- 架空看 ch19，将 `vel_leg_comp_sign` 在 `-1/+1` 间切换，选择速度估计波动更小的一侧；确定后写死并删除字段。
- 小机器 K 表当前按 **0.04 m** 轮径生成，而机器表采用 **0.03 m**，轮通道尺度偏差约 25%；待用 `Leg2_v1` 的 `WBR_modeling.mlx` 以小机器参数重跑 K 表。
- 大机器 `wheel_r=0.04 m` 仍是占位值，必须实测后才能标定完成。
- `trq_max_hip=5.0 N·m`、各通道开关与 0.13~0.20 m 投入域均待按小机器台架顺序验证；IMU 轴本批未动。

---

## 变更 63 · 小机器 LQR 批次 4：主文档同步与 sysid 文档归档

| 文件 | 改动 |
| --- | --- |
| `md/AGENTS.md` | 文件树新增 `md/sysid/`；关键约束同步 1 kHz、机器表∩K 表域、VOFA 500 Hz；模块表改为 `simple-function` 并补 `lqr_debug` |
| `md/RL_OVERVIEW.md` | 控制频率改 1 kHz、100 Hz 推理改每 10 周期；删除旧 VOFA 通道复制表并指向 `VOFA_SEND.md`；时钟修为 550 MHz；D-Cache 状态改为已开启且 VOFA DMA 前 Clean |
| `md/LQR_PLAN.md` | 常量表同步运行时限幅、腿长域和机器轮径；频率清单标完成；新增 `lqr_debug` 用法；替换为小机器台架顺序；遗留项与双配置编译结果同步 |
| `md/IO_CHAINS.md` | DM `err_raw` 使能看门狗/故障链；机器配置表来源；VOFA 改为单一文档链接；LQR 链同步 1 kHz、轮径、腿长交集与调试门 |
| `md/VOFA_SEND.md`、`imcalib/user-lib/Vofa_send.h` | 按当前代码重写 32 路表，删除乱码；默认 `VOFA_PORT=1`、1152000、500 Hz 与 D-Cache Clean 同步；sysid 只保留目录指针 |
| `md/DBUS.md`、`md/UART_IDLE_DMA.md` | 通读保留；修正 DR16 接受 `len>=18`、UART9=DR16、UART7=HI229 的现行拓扑 |
| `md/sysid/` | 六份大机器测试文档移入该目录，文件头统一标注“大机器测试专用，`SYSID_ENABLE=1` 时生效”，交叉路径同步 |
| `md/hip-test-vofa.md` | 删除；旧 32 通道测试布局已被变更 59 替代，需要时从提交 `87c6628` 恢复 |

**输入 / 输出 / 调用链**
- 代码与配置作为输入 → `AGENTS / RL_OVERVIEW / LQR_PLAN / IO_CHAINS / VOFA_SEND` 分别提供规则、架构、控制器、I/O 与观测通道的单一入口。
- 正常 VOFA：`task_comm.c` 当前 32 路 → `VOFA_SEND.md`；不再在多个总览文档复制易过时的通道表。
- 大机器 sysid：`SYSID_ENABLE=1` → `imcalib/Sysid/` → `md/sysid/`；正常小机器 LQR 文档与大机器测试材料分目录。
- 文档移动后，仓库内旧 `md/<sysid文件>` 路径统一改为 `md/sysid/<文件>`；Markdown 相对链接检查无死链。

**核对**
- `CLAUDE.md` 与记忆文件未改；`md/sysid-change-map.md` 保留在 `md/` 根目录且继续作为全工程账本。
- `MACHINE_DEFAULT=MACHINE_ID_LOCAL`、物理极性、零点、量程、镜像与 IMU 轴均未因文档整理改动。
- `DBUS.md`、`UART_IDLE_DMA.md` 已对照当前 `dr16.c`、`hi229.c` 与中断入口核对。
- `hip-test-vofa.md` 已删除，可由 Git 恢复；其余六份文档为移动并保留内容。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- `LQR_PLAN.md` 的小机器顺序仍须逐项实测；IMU 轴、腿摆速度补偿符号、输出限幅和大机器轮径均未在文档中冒充完成。
- 大机器 sysid 文档虽已归档并标开关范围，但 1 kHz 改频后的采样/发送丢帧行为需在再次启用前重新核对。

---

## 变更 64 · 删除 VOFA 通道文档 + 各 md 对齐代码现状（作者：更新各 md，把 vofa 通道 md 都删了）

| 文件 | 改动 |
| --- | --- |
| `md/VOFA_SEND.md`、`md/sysid/vofa-channel-map.md`、`md/sysid/wheel-test-vofa.md` | **删除**。正常 32 路通道表只保留 `task_comm.c` 里 `Robot_Control_Send_Vofa()` 上方的注释；sysid 列定义以 `sysid_log.c::assemble_frame()` 为准；旧文可从提交 `24efef2` 恢复 |
| `md/AGENTS.md`、`md/CLAUDE.md`、`md/RL_OVERVIEW.md`、`md/IO_CHAINS.md` | 去掉指向三份通道 md 的链接与文件树条目；VOFA 描述统一为 JustFloat、500 Hz、"布局以代码为准"；模块表标注 sysid 发送泵未接 |
| `md/sysid/sysid-delivery.md` | 帧 33 列 → **37 列 / 152 B**（补列 33~36）；心跳状态码补 5；轮用例改成代码实际的 10 个 plateau run（±0.5~4 A，钳位 ±5 A）；`SYSID_ENABLE` 默认 0、`SYSID_PLAN` 默认 2、补 `SYSID_MODE`；测试限幅 10 Nm；VOFA+ 通道数 37；写明发送泵未接与 1 kHz 推帧问题；去掉 FireWater 说法；文件头去重 |
| `md/sysid/controller-spec-for-mujoco.md` | 日期改回 2026-09-20；PD 参数 25/300 → **10/0**；姿态表改为 45°/90°/135° × 2.40/2.60/2.80；斜坡改为 `Ramp_Update()` 斜率限幅（0.4/0.2 rad/s，t_pre 4.2 s + 保持 3 s）；右腿动作值按各自 `dof_pos`；限幅层级补 `SYSID_TRQ_LIMIT_NM`；帧列 35 → 37；行号同步 |
| `md/sysid/sysid-lower-machine-plan.md` | 数据流/时钟/风险中的 500 Hz → 1 kHz；§7.1 补当前带宽现状；§10.1 采样率说明；§17.1 步 4 改为"待接回"；文件头去重 |
| `md/sysid-change-map.md` | 本条；附录 B 时钟行 500 Hz → 1 kHz |

**核对时发现、本次未改（属代码，需作者授权后另开变更）**
- `imcalib/task/task_comm.c`：`SYSID_ENABLE=1` 时没有任何地方调用 `Sysid_Log_Send_Pump()`（提交 `48b087e` 有该调用，`87c6628` 改成 10 通道轮帧直发时去掉，之后小机器 LQR 批次里 10 通道分支也删了），也没有"测试模式停发 32 路"的早退。现状：快照只进环形缓冲，串口上仍是 32 路 LQR 帧。
- `Sysid_Mode_Run()` 每个 1 ms 周期推一帧，发送泵最多 250 Hz 取一帧，接回后 128 帧环形缓冲约 0.17 s 即溢出；152 B × 1 kHz = 152 kB/s 也超过 1152000 波特的 115 kB/s 线速。需要先定"每 N 拍推一帧"的抽取方案。
- 过期注释：`sysid_log.h` 顶部列表仍写"33 列 / @921600 / 每 2 个周期发一次"；`tools/sysid_export.py` docstring 仍写"31 列"；`sysid_mode.c:722` 注释写 1 rad/s（实际 0.4/0.2）；`SYSID_POSE_RAMP_RATE` 宏无人引用。
- `sysid_mode.c` 的 `#ifndef SYSID_PLAN` 兜底值 1 与 `sysid_config.h` 的 2 不一致（后者先包含，实际生效 2）。

**输入 / 输出 / 调用链**
- 正常 VOFA：`comm_task_body()` → `Robot_Control_Send_Vofa()`（2 分频）→ `Vofa_Send(dbg, 32)` → `VOFA_UART`（USART1 @1152000）JustFloat；通道含义见该函数上方注释。
- sysid 帧：`Sysid_Mode_Run()` → `Sysid_Log_Push()` → 环形缓冲 → （**缺调用**）`Sysid_Log_Send_Pump()` → `assemble_frame()` 37 列 → DMA。
- 文档入口：`AGENTS.md` 关键约束 → `task_comm.c` 注释 / `sysid_log.c`；训练端 → `sysid-delivery.md` §1.2 快速索引 + `controller-spec-for-mujoco.md` §6/§7。

**核对**
- `md/` 内 Markdown 链接无死链；`VOFA_SEND.md` / `vofa-channel-map` / `wheel-test-vofa` 仅在本账本历史条目中以文件名出现（保留作记录）。
- 未改任何代码、配置、极性、零点、量程、镜像与 `MACHINE_DEFAULT`；本次只动 md，未编译。

**待台架 / 待决定**
- 大机器再次启用 sysid 前：作者定抽取方案 → 接回发送泵 + 测试模式停发 32 路 → 台架看心跳行列 10/11（drop/busy）恒 0、`seq` 连续、VOFA+ 37 列对齐。

---

## 变更 65 · 小机器 LQR 参数对齐 Leg2_v1（作者：把 D 加回去、参数按 Leg2 来；腿长区间与投入下限按本机自标）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_balance.h` | 腿长/防劈叉/横滚 KD：0/0/0 → **50000/500/100**（Leg2_v1 `Code/Task/balance.h` 原值） |
| `imcalib/Algorithm/lqr_balance.c` | `lqr_debug.trq_max_hip` 默认：5.0 → `machine->dm_trq_clamp`（小机器 10 N·m） |
| `imcalib/Algorithm/lqr_balance.h` | `LQR_RC_YAW_MAX`：3.0 → **5.0 rad/s**（Leg2_v1 `app_rc.c`）；`LQR_K_LEN_MIN` 0.13 **不动** |
| `imcalib/user-lib/machine_config.c` | 小机器 `wheel_r`：0.03 → **0.04**（物理量，见下方单列）；`leg_len_min/max` 0.10/0.20 **不动** |
| `md/LQR_PLAN.md` | §一 加决策 9；§1.1/§2.4/§五/§八 数值同步；§六 ① 拆成 ①a~①d 四项符号/零点测法；新增 §十 与 Leg2_v1 参数对照表 |
| `md/AGENTS.md`、`md/RL_OVERVIEW.md` | 辅助 PID 描述同步 |

**为什么**
- KD：两边 PID 的 D 项都是 kd×(本拍误差−上拍误差)、不除 dt，周期同为 1 kHz，Leg2 原值就是同一物理阻尼。腿长环是 1000 N/m 弹簧顶机身，无阻尼会以约 3 Hz 弹跳；50000 折合约 50 N·s/m。之前减半是 500 Hz 时的换算，改 1 kHz 时作者先归零，现在不再需要换算。
- 髋限幅：Leg2 正常模式虚拟髋力矩不限幅，电机侧只受 MIT ±10 N·m；本机原 5 N·m 比 Leg2 严一倍，大扰动时会先饱和。
- 偏航量程：只影响满杆转向速度，按"参数一样"取 5.0。
- 轮径：见下。

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-20** 回复"其他的按你说的改"，确认小机器轮半径 0.03 → 0.04。
- 依据：Leg2_v1 `Code/User/app_config.h` 的 `WHEEL_R 0.04f`；`轮腿上交建模MATLAB/WBR_modeling.mlx` 生效参数组 `R_w_ac = 0.04`（机身 5.25 kg、轮 0.13463 kg，与 Leg2 `BODY_MASS`/`WHEEL_MASS` 一致）；现用 K 表就是按这组生成。
- 纠错：变更 62 引用的"mlx 小机器参数组 R_w_ac=0.03"是**被百分号注释掉**的另一台车参数组（机身 1.103 kg、半轮距 0.075），属 AI 误读，本次纠正。
- 实际写入：`machine_config.c` 小机器行 `wheel_r = 0.04f`；大机器 0.04 占位不变，仍待实测。
- 复核：卡尺量轮子直径应接近 80 mm；即便有出入，先用 0.04 复现 Leg2 行为，因为 K 表与速度估计必须用同一个值。

**作者保留本机自标（未改）**
- 腿长区间 `leg_len_min/max` = 0.10/0.20，LQR 投入下限 `LQR_K_LEN_MIN` = 0.13（Leg2 为 0.12~0.29、默认站姿 0.12）。提醒：使能时腿长须 ≥0.13，否则 `LQR_Enable_Latch()` 返回 0 只发零力矩。

**输入 / 输出 / 调用链**
- KD：`Leg_Balance_Init()` → `PID_struct_init(kd)` → `pid_calc()` 的 `dout = d×(err[NOW]−err[LAST])` → F / Tp；`Leg_Balance_Reset()` 投入时清历史，首拍 D 输出为 0，无冲击。
- 髋限幅：`LQR_Init()` → `lqr_debug.trq_max_hip` → `LQR_Control_Update()` 虚拟髋力矩夹取 + `Leg_Balance_Compute()` 四台 DM 力矩夹取；仍可在调试器 Watch 里临时压低。
- 偏航：`LQR_Target_Update()` → `target[LQR_X_DPHI] = axis × 5.0`。
- 轮径：`LQR_State_Update()` → `whl × machine->wheel_r` → `x[LQR_X_DS]`；与 K 表生成参数一致后，LQR_PLAN "轮径与 K 表待重跑"一项关闭。

**核对**
- 未改极性、零点、MIT 量程、镜像、`+LEG_PI`、IMU 轴宏与 `MACHINE_DEFAULT`。
- K 表核对：`lqr_gain_table.c` 与 `Leg2_v1/Code/Matlab/LQR_K_WBR.c` 除 include 行外逐字节一致。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。未链接、未上机。

**待台架（作者：等会测试并修改）**
- ①a 腿摆角零位 `leg_off_phi0`（小机器表现为大机器换算值 −0.13/−0.07，Leg2 为 0）：腿竖直看 ch9/ch13 应 ≈0。
- ①b IMU 轴与极性：本机 `euler_rad[0]` 装的是模块 Roll 通道，`gyro_rad_s[1]` 是模块 Y 轴角速度，角与角速度不在同一根轴；Leg2 两者同取第 1 路且抬头为正。抬头看 ch17/18，只改 `lqr_balance.c` 顶部五个 `LQR_IMU_*` 宏并核对翻倒检测；右倾看 `lqr_state.roll` 应为正；架空手转车体，两轮应出反向阻转力矩。
- ①c 轮速腿摆补偿 `vel_leg_comp_sign`：推导上复现 Leg2 应为 +1，架空推腿看 ch19 取波动小者。
- ①d 转向通道符号：右摇杆推右应右转，Leg2 对该通道取负号。
- D 项恢复后首次落地观察腿长是否抖动（Leg2 手册：腿部振荡就降 KD）。

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
| 时钟调用周期 | Tick 必须 ≥ 每 7.81 s 一次 | 目前唯一挂在 1 kHz TIM6 上，满足 |
| 时钟溢出 | `cyc×1000` 约 9 小时上限 | 单次实验远小于该时长 |
| 力矩记录点 | 现在仍是"命令值"，量化后回算在步 5 | 见计划 §10.2 |
| 腿几何已进配置表 | ~~切机器时 `robot_control.c:43-63` 必须手改~~ → 变更 20 后只改 `MACHINE_DEFAULT` | 已完成 |
| 腿偏置为换算值 | 大机器偏置按参考固件表换算，未经卷尺/角度计验证 | 待台架标定；现象离谱就重标 |
