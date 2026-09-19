# 控制律复刻说明书（MuJoCo 仿真端）

> 目的：让 MuJoCo 端一字不差地复现下位机的虚拟关节 PD 位置跟踪控制律。
> 适用固件：`imcalib/Sysid/`（`SYSID_ENABLE=1`, `SYSID_MODE=SYSID_MODE_POSE`）
> 最后更新：2026-09-22

---

## 1. 整体链路总览

```
目标轨迹 (sysid_pose_runs[] 斜坡+保持)
  │
  │  sysid_mode.c: 斜坡插值 → act_buf[] → RL_Torque_Compute()
  ▼
虚拟关节 PD 控制器 (rl_torque.c)
  │  pid_calc() 对 6 个虚拟关节分别计算
  │  输出: tau_v[0..5] (虚拟关节力矩)
  ▼
雅可比映射 (rl_torque.c:217-220)
  │  τ_前髋 = τ_大腿 + τ_小腿 × vshank_jac[1]
  │  τ_后髋 = τ_小腿 × vshank_jac[0]
  ▼
4 台 DM 腿电机力矩 (已限幅 ±dm_trq_clamp)
  │
  │  Dm_Send_Torque() → Dm_Mit_Control()
  │  MIT 帧: kp=0, kd=0, 只有力矩字段有效
  ▼
CAN 总线 → DM 电机执行
```

**调用链（位置扫描模式）：**

| 步骤 | 函数 | 文件:行 |
|------|------|---------|
| 1. 500Hz 节拍 | `Sysid_Mode_Run()` | `imcalib/Sysid/sysid_mode.c:572` |
| 2. 斜坡插值得到目标角 | `ramp = t / t_pre` | `imcalib/Sysid/sysid_mode.c:717-725` |
| 3. 目标角→动作值 | `act_buf = (target - dof_pos) × 2.0` | `imcalib/Sysid/sysid_mode.c:731-734` |
| 4. 虚拟关节 PD + 雅可比映射 | `RL_Torque_Compute()` | `imcalib/Algorithm/rl_torque.c:124` |
| 5. PID 计算 | `pid_calc()` | `imcalib/user-lib/pid.c:42` |
| 6. 力矩下发 | `Dm_Send_Torque()` | `imcalib/user-lib/dm.c:291` |
| 7. MIT 编码 | `Dm_Mit_Control()` | `imcalib/user-lib/dm.c:187` |

---

## 2. PD 控制律精确离散形式

### 2.1 PID 实现（`imcalib/user-lib/pid.c:42-91`）

本工程使用 **位置式 PID**（`POSITION_PID`），D 项**不除 dt**：

```
e[NOW] = set - get
if angle_wrap:  e[NOW] = wrap180(e[NOW])          // [-π, π] 最短路径
if max_err ≠ 0 and |e[NOW]| > max_err:  return 0  // 越界保护

e_now  = Deadband_Soften(e[NOW],  deadband)        // 软化死区
e_last = Deadband_Soften(e[LAST], deadband)

pout = p × e_now
iout += i × e_now × dt                              // 积分项累加
dout = d × (e[NOW] − e[LAST])                       // 注意: 不除 dt!

abs_limit(iout, IntegralLimit)                       // 积分限幅
pos_out = pout + iout + dout
abs_limit(pos_out, MaxOutput)                        // 总输出限幅
```

**关键特性：**
- **D 项等效阻尼** = `d × dt`（因为 `dout = d × Δe ≈ d × dt × de/dt`，所以物理阻尼系数 = d × 控制周期）
- **angle_wrap** 时误差取最短路径：`wrap180(set - get)`（`pid.c:34-38`）
- **deadband** 处理（`pid.c:25-31`）：`|e| > deadband` 时 `e_out = e - sign(e)×deadband`；否则 `e_out = 0`
- **max_err** 越界直接返回 0（`pid.c:48-49`）

### 2.2 位置扫描的 PID 参数

`sysid_mode.c:543-564` 初始化 `sysid_pose_param`，然后 `RL_Torque_State_Init()` 将其写入 PID 控制器：

| 参数 | 大腿 (VJ_THIGH) | 小腿 (VJ_SHANK) | 轮 (VJ_WHEEL) | 来源 |
|------|-----------------|-----------------|---------------|------|
| kp | **25.0** | **25.0** | 0.0 | `sysid_mode.c:59` `SYSID_POSE_KP` |
| ki | 0.0 | 0.0 | 0.0 | `sysid_mode.c:545` 继承自 `RL_MODEL_STABLE` |
| kd | **300.0** | **300.0** | 0.0 | `sysid_mode.c:60` `SYSID_POSE_KD` |
| MaxOutput | **1000.0** | **1000.0** | **1000.0** | `rl_torque.c:98-109` |
| IntegralLimit | 0.0 | 0.0 | 0.0 | `rl_torque.c:98-109` |
| deadband | 0.0 | 0.0 | 0.0 | `pid.c:133`（`pid_param_init` 不设则为 0） |
| max_err | 0.0 | 0.0 | 0.0 | 同上 |
| angle_wrap | **开 (1)** | **开 (1)** | 关 (0) | `rl_torque.c:111-114` |
| pid_mode | POSITION_PID | POSITION_PID | POSITION_PID | `rl_torque.c:98-109` |

**控制周期 dt = 0.002 s**（500 Hz），硬编码在 `rl_torque.c:204/211` 的 `pid_calc(..., 0.002f)` 调用中。

### 2.3 一句话公式

对大腿和虚拟小腿（i ∈ {0, 1, 3, 4}）：

```
τ_v[i] = kp × wrap180(q_des[i] − q[i]) + kd × (e[i][k] − e[i][k−1])
```

其中：
- `kp = 25.0`，`kd = 300.0`
- `e[i] = wrap180(q_des[i] − q[i])`（角度环绕到 [-π, π]）
- D 项 = `kd × (e[k] − e[k−1])`，**不除 dt**
- 等效连续阻尼 = `kd × dt = 300 × 0.002 = 0.6 Nm·s/rad`
- 无积分项（ki=0），无死区，无 max_err 门限
- 输出限幅 ±1000.0 Nm（实际被 `dm_trq_clamp = 20.0` 先截断，见 §4.3）

轮子（i ∈ {2, 5}）在位置扫描模式下增益全零，不出力矩。

---

## 3. 虚拟关节定义与索引顺序

### 3.1 索引枚举（`rl_torque.c:13-21`，文件内部私有）

```c
enum {
    VJ_L_THIGH  = 0,   // 左大腿
    VJ_L_SHANK  = 1,   // 左虚拟小腿
    VJ_L_WHEEL  = 2,   // 左轮
    VJ_R_THIGH  = 3,   // 右大腿
    VJ_R_SHANK  = 4,   // 右虚拟小腿
    VJ_R_WHEEL  = 5,   // 右轮
};
```

**交错布局**：左腿 (0,1,2) → 右腿 (3,4,5)。

### 3.2 每个虚拟关节的物理量

| 虚拟关节 | 当前值 q[i] | 来源 | 定义 |
|----------|------------|------|------|
| VJ_L_THIGH (0) | `leg_l.output.thigh_angle` | `rl_torque.c:171` | `wrap(qf)`，qf = 前髋角（镜像后） |
| VJ_L_SHANK (1) | `leg_l.output.virtual_shank_angle` | `rl_torque.c:172` | `wrap(mirror × wrap(φa − qf − π/2))`，小腿相对大腿角 |
| VJ_L_WHEEL (2) | 固定 0.0 | `rl_torque.c:173` | 位置扫描不用轮 |
| VJ_R_THIGH (3) | `leg_r.output.thigh_angle` | `rl_torque.c:174` | 同左腿定义 |
| VJ_R_SHANK (4) | `leg_r.output.virtual_shank_angle` | `rl_torque.c:175` | 同左腿定义 |
| VJ_R_WHEEL (5) | 固定 0.0 | `rl_torque.c:176` | 位置扫描不用轮 |

**速度（用于轮子的速度环，位置扫描不使用）：**

| 虚拟关节 | 速度 qd[i] | 来源 |
|----------|-----------|------|
| VJ_L_THIGH | `leg_l.input.d_hip_f` | `rl_torque.c:177` |
| VJ_L_SHANK | `leg_l.output.d_virtual_shank_angle` | `rl_torque.c:178` |
| VJ_L_WHEEL | `wheel_vel[0]` | `rl_torque.c:179` |
| VJ_R_THIGH | `leg_r.input.d_hip_f` | `rl_torque.c:180` |
| VJ_R_SHANK | `leg_r.output.d_virtual_shank_angle` | `rl_torque.c:181` |
| VJ_R_WHEEL | `wheel_vel[1]` | `rl_torque.c:182` |

### 3.3 虚拟关节物理量定义（`leg_solver.c`）

**大腿角**（`leg_solver.c:99-100`）：

```
thigh_angle = wrap(qf)
qf = mirror × hip_f      // mirror = 1（左右腿均是，见 robot_control.c:47/52）
```

**虚拟小腿角**（`leg_solver.c:103-104`）：

```
vs_raw = wrap(φa − qf − π/2)
virtual_shank_angle = wrap(mirror × vs_raw)
```

- `φa` = 前杆绝对角度（闭链几何中间量，`leg_solver.c:82`）
- `qf` = 前髋角（镜像后）
- 含义：小腿相对于大腿的角度
- `mirror = 1` 时无取反效果

---

## 4. 虚拟关节力矩 → 电机力矩的映射

### 4.1 映射公式（`rl_torque.c:217-220`）

```
τ_前左髋 = τ_v[0] + τ_v[1] × vshank_jac[1]    // 大腿力矩 + 小腿力矩×前髋雅可比
τ_后左髋 = τ_v[1] × vshank_jac[0]               // 小腿力矩×后髋雅可比
τ_前右髋 = τ_v[3] + τ_v[4] × vshank_jac[1]    // 右腿同理
τ_后右髋 = τ_v[4] × vshank_jac[0]
```

代码对应（`rl_torque.c:217-220`）：
```c
tau_f[0] = tau_v[VJ_L_THIGH] + tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[1];
tau_b[0] = tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[0];
tau_f[1] = tau_v[VJ_R_THIGH] + tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[1];
tau_b[1] = tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[0];
```

### 4.2 vshank_jac 的含义（`leg_solver.c:152-158`）

```
d(virtual_shank_angle)/dt = vshank_jac[0] × dqb/dt + vshank_jac[1] × dqf/dt
```

| 分量 | 公式 | 含义 |
|------|------|------|
| `vshank_jac[0]` | `lu × sin(qb − φb) / (lg × sin(φa − φb))` | 虚拟小腿角对**后髋角**的偏导 |
| `vshank_jac[1]` | `−lu × sin(qf − φb) / (lg × sin(φa − φb)) − 1.0` | 虚拟小腿角对**前髋角**的偏导 |

代码（`leg_solver.c:153-157`）：
```c
jac_a = cache->lu * sin_bb / (cache->lg * sin_ab);           // vshank_jac[0]
jac_b = -cache->lu * sin_fb / (cache->lg * sin_ab) - 1.0f;  // vshank_jac[1]
```

其中 `sin_ab = sin(φa − φb)`，`sin_bb = sin(qb − φb)`，`sin_fb = sin(qf − φb)`。

**注意**：`vshank_jac` 是姿态相关的（随腿的构型实时变化），`leg_solver.c` 每个控制周期重新计算。左右腿各有一套，但代码中 `mirror = 1`（`robot_control.c:47/52`），左右共用同一套公式。

### 4.3 电机力矩限幅（`rl_torque.c:223-227`）

```c
leg_limit = machine->dm_trq_clamp;   // chuanliantui = 20.0 Nm（machine_config.c:13）
torque->dm[0] = clampf(tau_f[0], -leg_limit, leg_limit);  // 前左
torque->dm[1] = clampf(tau_b[0], -leg_limit, leg_limit);  // 后左
torque->dm[2] = clampf(tau_f[1], -leg_limit, leg_limit);  // 前右
torque->dm[3] = clampf(tau_b[1], -leg_limit, leg_limit);  // 后右
```

轮子力矩（位置扫描模式下为 0）：
```c
wheel_limit = machine->dji_trq_clamp;  // chuanliantui = 4.8 Nm
torque->dji[0] = clampf(tau_v[2], -wheel_limit, wheel_limit);  // 左轮
torque->dji[1] = clampf(tau_v[5], -wheel_limit, wheel_limit);  // 右轮
```

**限幅层级**（从内到外）：
1. PID MaxOutput = 1000.0 Nm（`pid.c:69`）—— 远大于实际需要
2. `dm_trq_clamp` = 20.0 Nm（`rl_torque.c:224`）—— **实际生效的软件限幅**
3. MIT 力矩满量程 = ±54 Nm（`dm.c:308`，`machine->dm_trq_max`）—— 硬件编码限幅

---

## 5. 电机侧的透明性

DM 电机在本工程中**只使用 MIT 帧的力矩字段**，位置环和速度环完全在下位机 MCU 内。

**证据（`dm.c:291-316`）：**

`Dm_Send_Torque()` 对每台电机调用 `Dm_Mit_Control()`，传入参数为：

```c
Dm_Mit_Control(i, DM_MIT_FIELD_MAX,   // angle_raw = 4095（位置场填满值，电机忽略）
                   DM_MIT_FIELD_MAX,   // vel_raw = 4095（速度场填满值，电机忽略）
                   0u,                 // kp_raw = 0（位置增益 = 0）
                   0u,                 // kd_raw = 0（速度增益 = 0）
                   trq_raw);           // trq_raw = 编码后的力矩值
```

来源：`dm.c:309-310`

- `DM_MIT_FIELD_MAX = 0x0FFF = 4095`（`dm.h:27`）
- `kp_raw = 0`、`kd_raw = 0`：电机驱动板不参与任何 PD 计算
- 力矩编码：`trq_raw = Dm_Float_To_Uint(command_torque, -dm_trq_max, dm_trq_max, 12)`（`dm.c:307-308`）

**结论**：仿真端只要复刻我们下位机的 PD 控制器即可，不需要在电机模型里再加任何位置/速度反馈。DM 电机等效为一个纯力矩执行器。

---

## 6. 轨迹激励（位置扫描模式）

### 6.1 模式概述

位置扫描模式（`SYSID_MODE = SYSID_MODE_POSE`）的目标是：让虚拟关节 PD 控制器跟踪一条预设的姿态轨迹，记录"指令力矩 + 实测角度"，供训练端做 real2sim 比对。

**与力矩激励模式的区别**：力矩模式直接下发力矩脉冲记录角度响应；位置模式通过 PD 控制器跟踪目标角，记录跟踪所需的力矩。

### 6.2 姿态表（`sysid_mode.c:213-217`）

```c
static const sysid_run_t sysid_pose_runs[] = {
    POSE_RUN(-0.15f, 2.50f), POSE_RUN(0.00f, 2.50f), POSE_RUN(0.15f, 2.50f),
    POSE_RUN(-0.15f, 2.80f), POSE_RUN(0.00f, 2.80f), POSE_RUN(0.15f, 2.80f),
    POSE_RUN(-0.15f, 3.10f), POSE_RUN(0.00f, 3.10f), POSE_RUN(0.15f, 3.10f),
};
```

共 **9 个 run**，每个 run = `{大腿角, 虚拟小腿角}`（rad）：

| run | 大腿角 (rad) | 虚拟小腿角 (rad) |
|-----|-------------|-----------------|
| 0 | −0.15 | 2.50 |
| 1 | 0.00 | 2.50 |
| 2 | +0.15 | 2.50 |
| 3 | −0.15 | 2.80 |
| 4 | 0.00 | 2.80 |
| 5 | +0.15 | 2.80 |
| 6 | −0.15 | 3.10 |
| 7 | 0.00 | 3.10 |
| 8 | +0.15 | 3.10 |

**两条腿走同一目标**（`sysid_mode.c:733-734`）：`act_buf[3] = act_buf[0]`，`act_buf[4] = act_buf[1]`。

### 6.3 每个 run 的时序（`sysid_mode.c:57-58`）

```
SYSID_POSE_RAMP_S = 0.5 s    // 从上一姿态线性滑到目标的时间
SYSID_POSE_HOLD_S = 2.0 s    // 到位后保持的时间（准静态取数）
```

每个 run 总时长 = 0.5 + 2.0 = **2.5 s**（`sysid_run_dur()` at `sysid_mode.c:271`）。

### 6.4 斜坡实现（`sysid_mode.c:710-725`）

```c
// 首次 run: 斜坡起点取当前实测角
if (tick_in_run == 0 && !sysid_pose_ready) {
    sysid_pose_prev[0] = leg_l.output.thigh_angle;        // 当前大腿角
    sysid_pose_prev[1] = leg_l.output.virtual_shank_angle; // 当前虚拟小腿角
    sysid_pose_ready = 1;
}

ramp = 1.0f;
if (t_pre > 0 && t < t_pre)
    ramp = t / t_pre;                           // 线性斜坡: 0 → 1

thigh_t = prev_thigh + (target_thigh - prev_thigh) * ramp;  // 线性插值
shank_t = prev_shank + (target_shank - prev_shank) * ramp;
```

- `ramp = t / t_pre`：从 **0 线性增长到 1**，持续 `t_pre = 0.5 s`
- `t ≥ t_pre` 后 `ramp = 1.0`，即保持目标角（`t_excite = 2.0 s`）
- **斜坡起点**：第一个 run 取当前实测角；后续 run 取上一个 run 的最终目标角（`sysid_mode.c:756-757`）

### 6.5 目标角→动作值的转换（`sysid_mode.c:731-734`）

```c
act_buf[0] = (thigh_t - dof_pos[0]) * 2.0f;   // 左大腿
act_buf[1] = (shank_t - dof_pos[1]) * 2.0f;   // 左小腿
act_buf[3] = act_buf[0];                        // 右大腿（同值）
act_buf[4] = act_buf[1];                        // 右小腿（同值）
```

这里 `× 2.0` 是因为 `RL_Torque_Compute` 内部做 `pos_ref = act × RL_TQ_POS_SCALE`（`rl_torque.c:190-193`），其中 `RL_TQ_POS_SCALE = 0.5`（`rl_torque.c:7`）。所以：

```
pos_ref = act × 0.5 = (target - dof_pos) × 2.0 × 0.5 = target - dof_pos
最终 PD 目标 = pos_ref + dof_pos = target
```

**静息位 dof_pos**（来自 `RL_MODEL_STABLE`，`rl_torque.c:77`）：

| 索引 | dof_pos | 含义 |
|------|---------|------|
| 0 (左大腿) | −0.23 rad | |
| 1 (左小腿) | −0.65 rad | |
| 2 (左轮) | 0.0 | |
| 3 (右大腿) | +0.23 rad | |
| 4 (右小腿) | +0.65 rad | |
| 5 (右轮) | 0.0 | |

### 6.6 帧内目标角记录（列 33/34）

在 `SYSID_MODE_POSE` 模式下，帧长扩展为 **35 列**（`SYSID_LOG_FRAME_N = 35`，`sysid_log.h:18`），比力矩模式多 2 列：

| 列 | 名称 | 含义 |
|----|------|------|
| 33 | `pose_tgt[0]` | 当前大腿目标角 (rad)，左右腿同值 |
| 34 | `pose_tgt[1]` | 当前虚拟小腿目标角 (rad)，左右腿同值 |

来源：`sysid_log.c:120-121`，值来自 `sysid_pose_prev[]`（`sysid_mode.c:823-824`）。

> **注意**：列 33/34 仅在 `SYSID_MODE=SYSID_MODE_POSE` 时有定义。`md/vofa-channel-map.md` 和 `md/sysid-delivery.md` 描述的是力矩模式（`SYSID_MODE_TORQUE`）的 33 列布局，不包含这两列。训练端应以本节为准。

---

## 7. 数据记录接口

### 7.1 记录方式

测试模式下，每 2 ms（500 Hz）由 `Sysid_Mode_Run()` 产生一帧快照（`sysid_snap_t`），推入环形缓冲（128 帧），由 `commTask`（1 kHz）以 **250 Hz**（`SYSID_TX_DIV=4`）通过 VOFA 串口（JustFloat 协议，921600 8N1）DMA 发出。

### 7.2 位置扫描模式的帧布局（35 列）

| 列 | 名称 | 单位 | 含义 |
|----|------|------|------|
| 0 | kind | - | 行类型（1=腿, 3=轮, 5=标记/心跳） |
| 1 | seq | - | 全局递增序号，跳号=丢帧 |
| 2 | phase_or_event | - | 正=段号, 0=心跳, −1/−2/−3=事件 |
| 3/4 | t_cmd hi/lo | - | **输入时间戳**：力矩命令上 CAN 总线时刻 |
| 5~8 | tau (前左/后左/前右/后右) | Nm | **指令力矩**（PD 输出，已限幅） |
| 9~12 | pos (同上顺序) | rad | **实测髋角**（零点后值） |
| 13~16 | vel (同上顺序) | rad/s | 髋角速度 |
| 17/18 | leg_length_L / leg_pitch_L | m / rad | 左腿长 / 左腿摆角 |
| 19/20 | leg_length_R / leg_pitch_R | m / rad | 右腿长 / 右腿摆角 |
| 21/22 | t_rx hi/lo | - | 轮反馈到达时刻 |
| 23/24 | t_leg hi/lo | - | **输出时间戳**：DM 反馈到达时刻 |
| 25~28 | d_len_L / d_pitch_L / d_len_R / d_pitch_R | m/s, rad/s | 腿长/摆角的解算导数 |
| 29 | clamp_cnt | - | 被限幅的电机数 |
| 30 | preload_n | N | 预压沿腿力（位置扫描模式下为 0） |
| 31/32 | whl/leg_drop_cnt | - | 总线发送丢帧累计 |
| **33** | **pose_tgt[0]** | **rad** | **大腿目标角** |
| **34** | **pose_tgt[1]** | **rad** | **虚拟小腿目标角** |

**训练端可用的输入输出配对**：
- **输入** = 列 5~8（PD 控制器算出的指令力矩）+ 列 33/34（目标角）
- **输出** = 列 9~12（实测髋角）+ 列 17~20（腿长/摆角）+ 列 25~28（导数）

时间戳还原：`ns = 列_hi × 1048576 + 列_lo`。

### 7.3 相关文档索引

| 文档 | 内容 |
|------|------|
| `md/vofa-channel-map.md` | 力矩模式（33 列）的逐列对照表 |
| `md/sysid-delivery.md` | 交付总说明：坐标系定义、单位、用例清单、SOP |

> 以上两份文档描述的是力矩模式（`SYSID_MODE_TORQUE`）的帧布局。位置扫描模式多了列 33/34（目标角），其余列含义相同。

---

## 8. 必须点明的两个坑

### 8.1 气弹簧

真机腿上安装有气弹簧，它提供一个沿腿方向的、随姿态变化的力（约 150 N 量级）。这个力通过腿的雅可比投影到两个髋电机上，产生一个**随腿长/腿摆角变化的外力矩**。

当前固件**不做气弹簧补偿**（`md/sysid-change-map.md` 变更 41/43 说明）。因此：
- MuJoCo 里如果没有气弹簧等效力，位置跟踪曲线会有一个**系统性的偏置**——PD 控制器算出的力矩会比真机小（因为真机有一部分力矩在对抗气弹簧）
- 建议：在仿真里加上等效的沿腿弹簧力，或在比对时先排除气弹簧的影响（例如比较"力矩变化量"而非"力矩绝对值"）

### 8.2 未建模摩擦/间隙/传动弹性

DM 电机的减速传动机构存在：
- **库仑摩擦 + 粘性摩擦**：表现为跟踪误差中的常值偏置和速度相关偏置
- **传动间隙（backlash）**：换向时角度突变
- **传动弹性**：高频激励下力矩到角度的相位滞后

这些效应在位置跟踪中表现为**小误差和相位滞后**——正是需要在仿真里调的参数。建议先在 MuJoCo 里加 Coulomb/viscous friction 和 backlash，再用 sysid 数据拟合参数值。

---

## 附录 A：chuanliantui 大机器关键参数快照

| 参数 | 值 | 来源 |
|------|-----|------|
| 杆长 lu / lg | 0.21 / 0.25 m | `machine_config.c:23-24` |
| 腿长工作区间 | 0.14 ~ 0.34 m | `machine_config.c:25-26` |
| dm_trq_clamp（腿力矩限幅） | 20.0 Nm | `machine_config.c:13` |
| dji_trq_clamp（轮力矩限幅） | 4.8 Nm | `machine_config.c:9` |
| dm_trq_max（MIT 力矩满量程） | 54.0 Nm | `machine_config.c:12` |
| dm_pos_max（MIT 位置满量程） | ±π rad | `machine_config.c:10` |
| dm_vel_max（MIT 速度满量程） | ±45 rad/s | `machine_config.c:11` |
| mirror（左右腿） | 1 | `robot_control.c:47/52` |
| 控制频率 | 500 Hz (dt = 0.002 s) | `task_actuation.c:12` |
| SYSID_POSE_KP | 25.0 | `sysid_mode.c:59` |
| SYSID_POSE_KD | 300.0 | `sysid_mode.c:60` |
| RL_TQ_POS_SCALE | 0.5 | `rl_torque.c:7` |
| PID MaxOutput | 1000.0 | `rl_torque.c:98` |
| PID IntegralLimit | 0.0 | `rl_torque.c:98` |

**极性与零点**：属台架标定项，见 `md/sysid-delivery.md` §3.2。训练端直接使用帧中记录的值即可，不要再取反或叠加偏置。
