# 输入输出链路总览

> 最后更新：2026-09-17
> 本文档记录各传感器/执行器的完整数据链路，从硬件到消费端。

---

## 1. IMU（HI229 姿态传感器）

```
HI229 模块 (UART7 921600bps)
    │  IDLE+DMA 接收
    ▼
hi229.c: HI229_Process()
    帧头搜索(0x5A 0xA5) → CRC 校验 → 解析 payload
    写入 hi229_data_t:
      eul[3]   raw 欧拉角 deg
      gyr[3]   raw 角速度 deg/s
      acc[3]   raw 加速度 G
      quat[4]  raw 四元数
      ts       模块时间戳 ms
      online   100ms 无帧→清零
    │
    │  HI229_Snapshot()
    ▼
task_imu.c: imu_task_body() @ 500Hz
    去重(ts) → 符号转换 → 单位转换 → 写入 imu_state:
      quat[4]       QUAT_SIGN × raw → 归一化
      euler_deg[3]  EUL_SIGN × raw
      euler_rad[3]  euler_deg × DEG2RAD
      gyro_rad_s[3] GYR_SIGN × raw × DEG2RAD
      online        归一化成功?
    │
    ▼
消费端:
  task_policy.c:
    imu_state.online       → 故障门控
    imu_state.gyro_rad_s   → obs.gyro
    imu_state.quat         → obs.gravity (quat_rotate_inv)
  task_comm.c:
    imu_state.online       → FAULT_IMU
    imu_state.euler_rad[0] → |pitch| 翻倒检测
```

**符号约定 (hi229.h):**

| 字段 | SIGN_X | SIGN_Y | SIGN_Z |
|------|:------:|:------:|:------:|
| quat | -1 | +1 | -1 |
| eul (RPY) | -1 | +1 | -1 |
| gyr | -1 | +1 | -1 |

---

## 2. DM 关节电机（达妙 J4310 MIT 协议）

```
DM J4310 × 4 (FDCAN1 左腿, FDCAN3 右腿)
    │  CAN 中断 @ 1000Hz
    ▼
dm.c: Dm_Read() — 中断回调
    8 字节 → 存入 dm_motor_feedback[i].raw_data
    置 raw_pending = 1
    │
    │  comm_task 调用
    ▼
dm.c: Dm_Parse()
    raw_data 解码:
      err_raw    = data[0]>>4
      motor_id   = data[0]&0x0F
      angle_raw  = data[1]<<8 | data[2]         (16bit)
      vel_raw    = data[3]<<4 | data[4]>>4      (12bit)
      trq_raw    = (data[4]&0xF)<<8 | data[5]   (12bit)
      temp_mos   = data[6]
      temp_rotor = data[7]
    if sign<0: angle/vel/trq 取反
    pos_rad   = uint_to_float(angle_raw, -PMAX, +PMAX, 16)   /* 纯解码角（只乘过反馈极性） */
    pos_zero_rad = pos_rad + machine->dm_zero[i]             /* 加零点偏置后的关节角 */
    vel_rad_s = uint_to_float(vel_raw, -VMAX, +VMAX, 12)
    trq_nm    = uint_to_float(trq_raw, -TMAX, +TMAX, 12)   /* PMAX/VMAX/TMAX 见 machine_config.h */
    Dm_Update_Angle() → angle_total (计圈)
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state.dm:
  pos_rad[i]     → 原始解码角（调试器 Watch 可用）
  pos_zero_rad[i] → leg.input.hip_f / hip_b (五连杆输入，零点后值)
  vel_rad_s[i]   → leg.input.d_hip_f / d_hip_b
  online[i]      → 故障检测
    │
    │  RL 推理 → rl_torque.c
    ▼
输出: torque_output_t → task_actuation.c
    torque.dm[0..3] → Dm_Send_Torque(torque.dm):
      if output_sign<0: command_torque = -torque[i]
      trq_raw = float_to_uint(command_torque, -10, +10, 12)
      Dm_Mit_Control(i, FIELD_MAX, FIELD_MAX, 0, 0, trq_raw)
        → 纯力矩模式 (kp=0, kd=0)
        → CAN 8 字节打包 → control_id
```

`err_raw` 同时参与使能状态：`0`=失能、`1`=使能、`0x8~0xE`=故障。`Robot_Enable_Update()` 在总使能期间调用 `Dm_Enable_Watchdog()`，对在线且 `err_raw=0` 的每台电机按独立 100 ms 计时重发 `DM_CMD_ENABLE`；`Dm_Has_Fault()` 并入 `FAULT_MOTOR`。

**电机映射:**

| 索引 | 位置 | CAN | feedback_id | control_id |
|:----:|------|-----|:-----------:|:----------:|
| 0 | F_LFT 左前髋 | FDCAN1 | 0x11 | 0x01 |
| 1 | B_LFT 左后髋 | FDCAN1 | 0x13 | 0x03 |
| 2 | F_RGT 右前髋 | FDCAN3 | 0x12 | 0x02 |
| 3 | B_RGT 右后髋 | FDCAN3 | 0x14 | 0x04 |

**极性不在驱动表里**：按机器存在 `machine_config.c` 的 `dm_sign[4]` / `dji_sign[2]`，每项是 `{反馈, 输出}` 一对。
**总线同理**：`dm_bus[4]`（1/2/3 = FDCANx）—— 大机器四台全在 FDCAN1，小机器左腿 1 / 右腿 3。

**关键函数:**

| 函数 | 作用 |
|------|------|
| Dm_Init() | 注册 CAN 回调 |
| Dm_Read() | 中断存 raw_data |
| Dm_Parse() | 解码 raw→物理量 |
| Dm_Is_Online() | 10ms 超时检测 |
| Dm_Is_Enabled() | `err_raw==1` 使能检测 |
| Dm_Has_Fault() | `err_raw` 在 `0x8~0xE` 的故障检测 |
| Dm_Enable_Watchdog() | 在线失能电机每 100ms 重发使能 |
| Dm_All_Enable() | 全部使能 |
| Dm_All_Disable() | 全部失能 |
| Dm_Send_Zero() | 零力矩 |
| Dm_Send_Torque() | 发送力矩数组 |

**ERR 状态码（不是故障位）**：`0` 失能 / `1` 使能 / `8` 过压 / `9` 欠压 / `A` 过流 / `B` MOS 过温 / `C` 线圈过温 / `D` 通信丢失 / `E` 过载。
判据是"落在 8~E 内"，不能写成 `err != 0`。

**满量程**：`PMAX`/`VMAX`/`TMAX` 是电机的量化刻度（出厂预设 ±12.5 / ±45 / ±54，可在上位机改），
必须与 `machine_config.c` 中当前机器的 `dm_*_max` 一致，否则角度与力矩整列都错。
核对方式：用达妙上位机读一次并与配置表比对（固件不做读参）。

---

## 3. DJI 轮电机（M2006 电流协议）

```
DJI M2006 × 2 (FDCAN2, 共享 control_id=0x200)
    │  CAN 中断
    ▼
dji.c: Dji_Read() — 中断回调
    8 字节 → 存入 dji_motor_feedback[i].raw_data
    置 raw_pending = 1
    │
    │  comm_task 调用
    ▼
dji.c: Dji_Parse()
    raw_data 解码:
      angle_raw   = data[0]<<8 | data[1]    (uint16)
      vel_raw     = data[2]<<8 | data[3]    (int16)
      current_raw = data[4]<<8 | data[5]    (int16)
      temp_raw    = data[6]                 (int8)
    if sign<0: angle=CPR-angle, vel=-vel, current=-current
    Dji_Circle_Calculate() → angle_total (计圈)
    Dji_Update_Physical():
      angle_rad       = count × RAD_PER_COUNT
      angle_total_rad = angle_total × RAD_PER_COUNT
      vel_rad_s       = rpm × RPM_TO_RAD_S
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state.dji:
  vel_rad_s[i]       → obs.joint_vel (轮速度)
                       → wheel_vel (RL 力矩计算)
  online[i]          → 故障检测
    │
    │  RL 推理 → 力矩输出
    ▼
输出: torque_output_t → task_actuation.c
    torque.dm[DM_MOTOR_*] → Dm_Send_Torque(torque.dm)
    torque.dji[DJI_MOTOR_*] → Dji_Send_Wheel_Torque(left_nm, right_nm)
      Dji_Torque_To_Current():
        M2006: raw = torque_nm / 0.00018, clamp ±10000
      wheel_current[0..3] → Dji_Send_Current(FDCAN2, 0x200, current)
        → 8 字节: 4×int16 大端打包
    Dji_All_Stop():
      → 4 通道全 0 电流
```

**电机映射:**

| 索引 | 位置 | CAN | feedback_id | control_id |
|:----:|------|-----|:-----------:|:----------:|
| 0 | 左轮 | FDCAN2 | 0x201 | 0x200 |
| 1 | 右轮 | FDCAN2 | 0x202 | 0x200 |

型号按机器取（`machine_config.c` 的 `dji_type`）：本机 M2006 / chuanliantui M3508。
表结构与 DM 完全一致（`motor_cfg_t`，见 `can_bus.h`）；极性同样在 `machine_config.c` 的 `dji_sign[2]`。

**关键函数:**

| 函数 | 作用 |
|------|------|
| Dji_Init() | 注册 CAN 回调 |
| Dji_Read() | 中断存 raw_data |
| Dji_Parse() | 解码 raw→物理量 |
| Dji_Circle_Calculate() | 编码器计圈 |
| Dji_Update_Physical() | 编码→弧度, rpm→rad/s |
| Dji_Torque_To_Current() | Nm→raw current |
| Dji_Send_Wheel_Torque() | 左右轮力矩发送 |
| Dji_All_Stop() | 零电流停机 |
| Dji_Is_Online() | 10ms 超时检测 |

---

## 4. DR16 遥控器（DBUS 协议）

```
DR16 接收机 (UART9 DBUS, 100kbps)
    │  IDLE+DMA 接收, 18 字节/帧
    ▼
dr16.c: DR16_Process()
    帧长校验(18) → DR16_Parse():
      ch0    = (buf[0..1] & 0x07FF) - 1024     摇杆右X
      ch1    = (buf[1..2] & 0x07FF) - 1024     摇杆右Y
      ch2    = (buf[2..4] & 0x07FF) - 1024     摇杆左X
      ch3    = (buf[4..5] & 0x07FF) - 1024     摇杆左Y
      s1     = (buf[5]>>4) & 0x0C >> 2         左拨杆
      s2     = (buf[5]>>4) & 0x03              右拨杆
      mx/my/mz = buf[6..11]                    鼠标轴
      ml/mr  = buf[12..13]                     鼠标键
      key    = buf[14..15]                     键盘
      wheel  = 1024 - buf[16..17]              左侧拨轮
    校验: ch0-3/wheel ∈ [-660,660], s1/s2 ∈ [1,3]
    写入 dr16 (dr16_t)
    │
    │  DR16_Snapshot()
    ▼
task_comm.c: Remote_Control_Update()
    DR16_Process() → DR16_Snapshot()
    │
    ├─ s1 != DOWN → rc_enable = 1
    ├─ ch3 → vx_cmd     (归一化 × REMOTE_COMMAND_SCALE)
    ├─ ch0 → yaw_cmd    (归一化 × REMOTE_COMMAND_SCALE)
    ├─ wheel → height_cmd (归一化 × REMOTE_COMMAND_SCALE)
    └─ s1 → mode
    │
    │  消费端
    ▼
task_comm.c:
  Robot_Fault_Update():
    !DR16_Online() → FAULT_RC
  Robot_Fallen_Update():
    |pitch|>1.4 → fallen=1, <1.0 → 回正
  Robot_Enable_Update():
    rc_enable && !fault && !fallen → robot_state.enabled
  Robot_Control_Output():
    !rc_enable → Dm_All_Disable + Dji_All_Stop (安全)

task_policy.c (手动遥操):
  ch3    → thigh 偏移 (×4.0 叠加 base_action)
  wheel  → shank 偏移 (×4.0 叠加 base_action)
  ch1    → wheel 速度 (×4.0 直接赋值, 宽死区100)
```

**dr16_t 字段:**

| 字段 | 来源 | 范围 | 说明 |
|------|------|:----:|------|
| ch0 | 摇杆右X | ±660 | yaw 指令 |
| ch1 | 摇杆右Y | ±660 | (未用) |
| ch2 | 摇杆左X | ±660 | (未用) |
| ch3 | 摇杆左Y | ±660 | 前进/后退 |
| wheel | 左侧拨轮 | ±660 | 高度/大腿偏移 |
| s1 | 左拨杆 | 1/2/3 | 使能控制 |
| s2 | 右拨杆 | 1/2/3 | 模式选择 |
| mx/my/mz | 鼠标 | int16 | (未用) |
| ml/mr | 鼠标键 | 0/1 | (未用) |
| key | 键盘 | uint16 | (未用) |
| online | 100ms 超时 | bool | 在线 |

**拨杆语义:**

| 拨杆 | 值 | 作用 |
|:----:|:--:|------|
| s1 DOWN | 2 | 失能 (rc_enable=0) |
| s1 MID | 3 | 使能 |
| s1 UP | 1 | 使能 |
| s2 | — | 模式选择 (未接线) |

**关键函数:**

| 函数 | 作用 |
|------|------|
| DR16_Init() | UART9+DMA 启动 |
| DR16_Process() | 解析一帧写入 dr16 |
| DR16_Online() | 100ms 超时检测 |
| DR16_Deadline() | 死区滤波 |
| DR16_Snapshot() | 返回 dr16 副本 |

---

## 5. 状态聚合层（commTask 写入）

三个聚合结构体，均为 commTask 写入、其他任务只读。

### motor_state — 电机状态

```
dm_motor_feedback[i] / dji_motor_feedback[i]  (驱动层解码)
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state_t motor_state:
    dm.pos_rad[i]       ← dm_motor_feedback[i].pos_rad       （原始解码角）
    dm.pos_zero_rad[i]  ← dm_motor_feedback[i].pos_zero_rad  （零点后值）
    dm.vel_rad_s[i]     ← dm_motor_feedback[i].vel_rad_s
    dm.trq_nm[i]        ← dm_motor_feedback[i].trq_nm
    dm.last_rx_tick[i]  ← dm_motor_feedback[i].last_rx_tick
    dm.online[i]        ← Dm_Is_Online(i)
    dji.angle_rad[i]       ← dji_motor_feedback[i].angle_rad
    dji.angle_total_rad[i] ← dji_motor_feedback[i].angle_total_rad
    dji.vel_rad_s[i]       ← dji_motor_feedback[i].vel_rad_s
    dji.current_raw[i]     ← dji_motor_feedback[i].current_raw
    dji.last_rx_tick[i]    ← dji_motor_feedback[i].last_rx_tick
    dji.online[i]          ← Dji_Is_Online(i)
    timestamp_ms           ← HAL_GetTick()
    updated                = 1

消费端:
  task_policy.c: dm.pos_zero_rad/d.vel_rad_s → obs.joint_pos/vel
  task_policy.c: dji.vel_rad_s → obs.joint_vel (轮)
  task_comm.c:   dm/dji.online → FAULT_MOTOR
  task_comm.c:   dm.pos_zero_rad/vel_rad_s → leg.input (髋关节映射，零点已在 dm.c 叠加)
  task_actuation.c: dji.vel_rad_s → wheel_vel → RL_Torque_Compute
```

### input_command — 遥控指令

```
DR16 遥控器 (ch0/ch3/wheel/s1)
    │
    │  Remote_Control_Update() @ task_comm.c
    ▼
input_command_t input_command:
    vx_cmd     = ch3 / 660 × REMOTE_COMMAND_SCALE
    yaw_cmd    = ch0 / 660 × REMOTE_COMMAND_SCALE
    height_cmd = wheel / 660 × REMOTE_COMMAND_SCALE
    mode       = s1

消费端:
  task_policy.c: → obs.command[6..8]
  task_policy.c: → Manual_Action_Apply (手动测试)
```

### robot_state — 机器人状态

```
DR16 / IMU / 故障状态
    │
    │  task_comm.c 各子函数写入
    ▼
robot_state_t robot_state:
    rc_enable    ← Remote_Control_Update()
                   s1 != DOWN && DR16_Online()
    fallen       ← Robot_Fallen_Update()
                   |pitch|>1.4 → 1, <1.0 → 0
    motor_enabled← Robot_Enable_Update()
                   rc_enable && ctrl_fault==0 && !fallen

消费端:
  task_actuation.c:
    !rc_enable     → Dm_All_Disable + Dji_All_Stop
    !motor_enabled → Dm_Send_Zero (空转)
    motor_enabled  → RL_Torque_Compute (正常推理)
  task_policy.c:
    motor_enabled  → 锁存 base_action
    !motor_enabled → Action_State_Clear
```

---

## 6. 五连杆腿部解算（leg_solver）

```
dm.pos_zero_rad[0..3] (DM 电机编码器，零点后值)
    │
    │  Leg_State_Update() @ task_comm.c
    ▼
leg_l / leg_r (leg_state_t):
    input.hip_f   = dm.pos_zero_rad[F] + π                   前髋角 (零点已在 dm.c 叠加, +π 几何偏置)
    input.hip_b   = dm.pos_zero_rad[B]                       后髋角 (零点已在 dm.c 叠加)
    input.d_hip_f = dm.vel_rad_s[F]                           前髋速度
    input.d_hip_b = dm.vel_rad_s[B]                           后髋速度
    │
    │  Leg_Solve() @ task_comm.c
    ▼
三层求解:

① Leg_Solve_Geometry — 闭链几何
    qf = mirror × hip_f, qb = mirror × hip_b
    A = (lu·cos qf, lu·sin qf)    前杆端点
    B = (lu·cos qb, lu·sin qb)    后杆端点
    求 P 点 (两圆交点) → phi_a, phi_b
    输出:
      thigh_angle          = Leg_Wrap(qf)                        大腿角(前髋上连杆, 去镜像后与 hip_f 一致)
      virtual_leg_length   = |OP|                           虚拟腿长
      virtual_leg_angle    = π/2 - atan2(y_p,x_p) + offset_phi0  虚拟腿摆角
      virtual_shank_angle  = mirror × (phi_a - qf - π/2)   虚拟小腿角

② Leg_Solve_Velocity — 速度雅可比
    leg_jac[2][2]       → d_virtual_leg_length, d_virtual_leg_angle
    vshank_jac[2]       → d_virtual_shank_angle

③ Leg_Solve_Force_Map — 力矩映射
    force_map = leg_jac 转置
    Leg_Force_Map_Forward(force, torque) → tau_f, tau_b
    │
    │  消费端
    ▼
task_policy.c:
  virtual_shank_angle    → obs.joint_pos (虚拟小腿)
  d_virtual_shank_angle  → obs.joint_vel (虚拟小腿)
  thigh_angle            → 手动遥操偏移基准 (base_action)
rl_torque.c:
  virtual_shank_angle    → q[1,4] (RL 关节位置)
  d_virtual_shank_angle  → qd[1,4] (RL 关节速度)
  thigh_angle            → q[0,3] (RL 关节位置)
  vshank_jac             → 力矩分解 tau_f/tau_b
  输出: torque_output_t {dm[4], dji[2]}
task_comm.c:
  virtual_leg_length, virtual_leg_angle, virtual_shank_angle → VOFA 调试
```

**配置参数（`machine_config.c`）：**

`lu`、`lg`、`dm_zero`、`offset_phi0` 和机器腿长区间均已进入机器表；`robot_control.c` 初始化时只读取当前 `machine`。

| 参数 | 左腿 | 右腿 | 说明 |
|------|:----:|:----:|------|
| lu | 0.13087 | 0.13087 | 上杆长 (m) |
| lg | 0.15240 | 0.15240 | 下杆长 (m) |
| dm_zero | 见 machine_config.c | 见 machine_config.c | 电机零点 (rad)，dm.c 解码时叠加 |
| offset_phi0 | -0.13 | -0.07 | 方向角偏置 (rad) |
| mirror | 1 | 1 | 镜像系数 |

**输出字段:**

| 字段 | 含义 | 用途 |
|------|------|------|
| thigh_angle | 大腿角 (前髋上连杆, qf) | RL obs + PD + VOFA |
| virtual_leg_length | 虚拟腿长 \|OP\| | VOFA 调试 |
| virtual_leg_angle | 虚拟腿摆角 (相对竖直) | LQR |
| virtual_shank_angle | 虚拟小腿角 (小腿相对大腿) | RL obs + PD |
| d_virtual_leg_length | 虚拟腿长速度 | 调试 |
| d_virtual_leg_angle | 虚拟腿摆角速度 | LQR |
| d_virtual_shank_angle | 虚拟小腿角速度 | RL obs |
| vshank_jac[2] | 虚拟小腿雅可比 | RL 力矩分解 |
| force_map[2][2] | 力矩映射矩阵 | 力矩输出 |

**VOFA 通道：**正常控制的 32 路布局与 500 Hz 发送参数统一见 [VOFA_SEND.md](VOFA_SEND.md)。

---

## 7. RL 观测层（policyTask）

```
imu_state (IMU 姿态)
    gyro_rad_s[3]   → obs[0-2] × gyro_scale
    quat[4]         → obs[3-5]  quat_rotate_inv → gravity 投影
input_command (遥控)
    vx/yaw/height   → obs[6-8] × command_scale
leg_l / leg_r (腿部状态)
    input.hip_f                  → obs[9,11]  - dof_pos (大腿角偏置)
    output.virtual_shank_angle   → obs[10,12] - dof_pos (虚拟小腿角偏置)
joint_vel[6] (关节速度)
    leg_l.input.d_hip_f              → obs[13]
    leg_l.output.d_virtual_shank_angle → obs[14]
    dji.vel_rad_s[WHEEL_LFT]         → obs[15]
    leg_r.input.d_hip_f              → obs[16]
    leg_r.output.d_virtual_shank_angle → obs[17]
    dji.vel_rad_s[WHEEL_RGT]         → obs[18]
last_action[6]                    → obs[19-24]
    │
    │  RL_Observation_Build() @ task_policy.c
    ▼
rl_observation_state_t:
    obs[25]                      当前观测帧
    history[125]                 5帧历史 (5×25, 循环左移)
    last_action[6]               上步动作
    │
    │  CubeAI 推理
    ▼
网络输入: obs[25] + history[125] + last_action[6] = 156 维
网络输出: action[6] → 6 维动作
    │
    │  RL_Torque_Compute() → rl_torque.c
    ▼
力矩分解 (见 rl_torque 链路)
```

**观测维度:**

| 索引 | 字段 | 来源 | 缩放 |
|:----:|------|------|------|
| 0-2 | gyro | imu_state.gyro_rad_s | gyro_scale |
| 3-5 | gravity | quat_rotate_inv(quat) | — |
| 6-8 | command | input_command.vx/yaw/height | command_scale |
| 9 | l_thigh | leg_l.input.hip_f - dof_pos[0] | — |
| 10 | l_shank | leg_l.output.virtual_shank_angle - dof_pos[1] | — |
| 11 | r_thigh | leg_r.input.hip_f - dof_pos[2] | — |
| 12 | r_shank | leg_r.output.virtual_shank_angle - dof_pos[3] | — |
| 13 | l_thigh_vel | leg_l.input.d_hip_f | joint_vel_scale |
| 14 | l_shank_vel | leg_l.output.d_virtual_shank_angle | joint_vel_scale |
| 15 | l_wheel_vel | dji.vel_rad_s[WHEEL_LFT] | joint_vel_scale |
| 16 | r_thigh_vel | leg_r.input.d_hip_f | joint_vel_scale |
| 17 | r_shank_vel | leg_r.output.d_virtual_shank_angle | joint_vel_scale |
| 18 | r_wheel_vel | dji.vel_rad_s[WHEEL_RGT] | joint_vel_scale |
| 19-24 | last_action | 上步动作 | — |

---

## 全局数据流

```
┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
│  HI229   │  │  DM ×4   │  │  DJI ×2  │  │   DR16   │
│  UART7   │  │ FDCAN1/3 │  │  FDCAN2  │  │  UART9   │
└────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘
     │             │             │             │
     ▼             ▼             ▼             ▼
 imu_state    motor_state_t (聚合)        input_command_t
              dm.pos_rad/vel/trq          robot_state_t
              dji.vel_rad_s/online
                   │
           ┌───────┴───────┐
           ▼               ▼
     leg_l/leg_r     ┌───────────┐
     Leg_Solve()     │  RL 推理  │
     五连杆解算      │ CubeAI    │
           │         └─────┬─────┘
           │               │
           ▼               ▼
     ┌───────────┐  ┌────────────┐
     │   LQR     │  │ rl_torque  │
     │ 平衡控制  │  │ 力矩分解   │
     └─────┬─────┘  └─────┬──────┘
           │               │
           ▼               ▼
      ┌─────────────────────────┐
      │     task_actuation      │
      │  DM/DJI 力矩下发        │
      └─────────────────────────┘
```

---

## 8. LQR 平衡链路（task_actuation 内，@1kHz）

只在左拨杆中位时激活。全部计算在 `actuationTask` 里完成，只读其它任务的共享状态。

```
imu_state (pitch/roll/yaw/gyro)    leg_l / leg_r (Leg_Solve 输出)
motor_state.dji.vel_rad_s          DR16_Snapshot()
        │                                  │
        ▼                                  ▼
  LQR_State_Update()  ←──────────  LQR_Target_Update()
  x[10] 状态组装 + 速度运动学 + 位移积分     target[10] + 腿长目标
        │                                  │
        └──────────────┬───────────────────┘
                       ▼
              LQR_Control_Update()
              腿长变化>0.5mm → LQR_K_WBR(h_l,h_r) 求 40 个增益
              u[i] = Σ K[i][j]·(target[j] − x[j])   → [T_wl,T_wr,T_bl,T_br]
                       │
                       ▼
              Leg_Balance_Compute()
              腿长PID + 防劈叉PID + 横滚PID → 足端力 F
              Leg_Force_Map_Forward(&leg, F, Tp) → 前/后髋力矩
              lqr_debug 通道门 + 限幅 → torque_output_t
                       │
                       ▼
        Dm_Send_Torque() + Dji_Send_Wheel_Torque()
```

**状态索引**：`[s, ds, φ, dφ, θ_ll, dθ_ll, θ_lr, dθ_lr, θ_b, dθ_b]`，与数学建模一致；φ（偏航角）不参与控制，只控角速度。
**腿摆角世界系**：`−virtual_leg_angle + pitch`；**角速度**同理 `−d_virtual_leg_angle + omg_pitch`。
（本工程解算腿角前摆为正，数学模型 θ_ll 前摆为负，**整体取反后再加 pitch**；髋扭矩同步取反，详见 [LQR_PLAN.md](LQR_PLAN.md) §3.1）
**速度**：`ω_轮·machine->wheel_r + L·dθ·cosθ + dL·sinθ` 后接一阶低通（α=0.3）；腿摆速度补偿符号由 `lqr_debug.vel_leg_comp_sign` 暂作台架 A/B，默认 −1 保持现状。
**腿长限制**：机器表工作区间与 K 表拟合域 0.13~0.23 m 的交集；小机器为 0.13~0.20 m。
**调试门**：`lqr_debug` 可分别关闭轮、髋、腿长 PID 的最终输出并调整限幅；关闭通道时 PID 仍持续计算。
**符号责任**：反馈极性按 `feedback_sign` 在驱动解码时统一到机体坐标；输出极性按 `output_sign` 在驱动下发时统一处理（`dm.c` / `dji.c`），调用方不要取反。详见 [LQR_PLAN.md](LQR_PLAN.md)。
