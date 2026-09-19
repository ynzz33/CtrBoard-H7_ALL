# RL 部署总览 — 代码链路 & 进度

## 当前重构约定

- 用户后续自行调整遥控映射；临时 action 测试代码已删除，不再加入额外测试控制链路。
- 任务代码迁至 `imcalib/task/`；IMU、策略、执行、通信各自独立文件。
- `task/inc/robot_control.h` 统一声明共享状态和跨任务接口，不再保留分散的 `task_*.h`。
- Keil 与 eIDE 已更新任务源目录和 `imcalib/task/inc` include 路径。
- 已阅读姿态解算、`leg_solver`、观测构建；`rl_policy` 留待结合训练同学的模型参数共同检查。
- `rl_torque` 保持动作、PID、雅可比和电机输出的直接链路。PID 直接逐个 `PID_struct_init`，不使用参数指针同步、循环初始化或隐式重配。
- 当前不加入气弹簧补偿、斜率限制或额外控制策略。
- 多次计算保持单一中间量和短表达式；注释简短。循环索引在 `for` 内定义。

> 最后更新：2026-09-17
> 参考实车：XYEGA_RM2026_WheelLeg_Infatry_RLdeploy（复旦 EGA 2026 国赛上场版）

---

## 一、整体思路

轮腿平衡步兵的 RL 控制本质是一个 **sim-to-real** 流程：

1. **训练端**（IsaacLab / MuJoCo）产出 ONNX 策略模型
2. **板端**（STM32H723 + CubeAI）实时推理，输出 6 维动作
3. **执行层**把动作经 PD + 雅可比映射为 6 个电机力矩

整个链路在 500Hz 控制环里跑，RL 推理锁频 100Hz（每 5 个周期推理一次，其余复用上次动作）。

**数据流一句话**：
```
IMU(四元数+陀螺仪) + 电机编码器(关节角) + DJI轮速 + 遥控指令
    → 五连杆运动学 → 25维观测 → 5帧历史堆叠(125维)
    → CubeAI推理 → 6维动作 → PID → 雅可比映射 → 6个电机力矩
```

## 极性与坐标定义（已确认，禁止擅自修改）

### HI229 IMU

| 数据 | X | Y | Z | 说明 |
|------|---|---|---|------|
| 加速度 `accel_g` | -1 | +1 | -1 | 原始加速度映射到机体坐标；静止重力方向为负 |
| 角速度 `gyro_dps` | -1 | +1 | -1 | 映射后用于 RL 角速度观测 |
| 欧拉角 | -1 | +1 | -1 | 对应 Roll / Pitch / Yaw |
| 四元数虚部 | -1 | +1 | -1 | 对应 qx / qy / qz，实部 qw 不变 |

- HI229 姿态链路直接采用已映射的参考四元数；原始加速度不参与当前四元数生成。加速度极性只影响 `accel_g`、`accel_normed` 与调试输出，未来若启用加速度融合时必须重新复核。
- 投影重力使用 `g_body = R^T * [0,0,-1]`，只由输出四元数计算，不跟随原始加速度极性直接翻转。当前已验证水平静止、俯仰、横滚和偏航行为正确，禁止额外取反。

### DM 髋电机与五连杆

- DM 左前、左后髋 `feedback_sign=+1`；右前、右后髋 `feedback_sign=-1`。驱动层先统一右侧角度、角速度和力矩反馈到逻辑机体坐标。
- 几何输入中，前髋 `hip_f = DM 反馈角 + π + offset_f`，后髋 `hip_b = DM 反馈角 + offset_b`。左右腿均使用相同前后映射，`config.mirror=+1`，禁止在五连杆层再次镜像或交换电机。
- `l0` 增大表示伸腿；`phi0` 前摆为正、后摆为负；`virtual_shank = wrap(phi_a - qf - π/2)`，采用相对大腿的 EGA 虚拟小腿定义。
- 逻辑髋电机力矩到实体输出时，右侧 DM 只在 `dm.c` 驱动边界取反一次；禁止在解算器、测试模式或任务层再次取反。

### DJI 轮电机与遥控

- DJI 左轮 `feedback_sign=+1`，右轮 `feedback_sign=-1`；轮速和多圈角已验证：机器人前进方向为正，后退方向为负。
- 轮电机输出极性由 `dji.c` 按配置表的 `output_sign` 统一处理（与 `feedback_sign` 同号）；调用方不要再取反
- 遥控接收方向已验证；普通 RL 指令当前统一缩放到 `±0.1` 用于安全测试，最终策略缩放以训练参数确认后再更新。

以上定义已经过当前机械安装的腿部输入、腿长、腿角、虚拟小腿、速度/雅可比、虚拟力矩映射和实体运动联调确认。若物理行为再次异常，先检查反馈/下发链路和报文状态，不得直接改几何符号。

---

## 二、任务架构

5 个 FreeRTOS 任务，职责分明：

| 任务 | 频率 | 节拍方式 | 职责 |
|------|------|----------|------|
| `actuationTask` | 500Hz | TIM6 信号量（硬实时） | 策略仲裁（LQR / 手动遥操）→ 力矩计算 → CAN 下发 |
| `policyTask` | 100Hz | osDelay | 观测构建 → CubeAI 推理 → 写 action_state |
| `imuTask` | 500Hz | osDelay(2ms) | HI229 新帧解析 → 姿态更新 → 写 imu_state |
| `commTask` | 1kHz | osDelay | DM/DJI/DR16 解析 → 状态更新 → 在线检测 → 故障位 → VOFA |
| `defaultTask` | - | - | USB 初始化（保留） |

**单写者模型**：每个共享状态只有一个任务写，32bit 对齐 float 在 M7 上读写原子，无需锁。

`freertos.c` 只维护任务初始化、循环和节拍等待。共享状态和公共控制接口位于 `robot_control.c/h`，各 `task_*.c` 实现对应任务的单周期逻辑。

**数据流向**：
```
[ISR] FDCAN → dm/dji raw_pending
[ISR] UART  → hi229_rx.flag / dbus_rx.flag
[ISR] TIM6  → ctrl_tick_sem (500Hz 信号量)

imuTask     → imu_state {quat, eul, gyr, acc, online}
commTask    → motor_state/leg_state; ctrl_fault; VOFA
policyTask  → action_state {a[6], last_ok_tick, updated}
actuationTask → torque_output_t → DM/DJI 力矩 → CAN
```

---

## 三、RL 部署链路详解

### 3.1 五连杆运动学 (`leg_solver`)

**输入**：左右各 2 个 DM 髋电机的角度 + 角速度（hip_f=前髋，hip_b=后髋）。前髋的几何零位比 DM 反馈零位多 π，使"前后上连杆反向水平"的最短腿姿态对应虚拟腿竖直。腿长和腿角由前后髋共同决定，不能把两台电机简称为"大腿/小腿电机"。

**计算**：
- 闭链几何求解足端 P → 腿长 l0、腿摆角 phi0（前摆为正，含零点偏置）
- 大腿角 `thigh_angle = Leg_Wrap(qf)`（前髋上连杆角，去镜像后与 hip_f 一致）
- 虚拟小腿角 `virtual_shank = wrap(phi_a - qf - π/2)`（相对前髋电机）
- 雅可比：`point_jac`（足端直角坐标）、`leg_jac`（极坐标）、`vshank_jac`（虚拟小腿）、`force_map`（力域转换）

实现分为闭链几何、速度与雅可比、力矩映射三层，`Leg_Solve()` 仅负责按顺序调用三层。

**根因修复（2026-09-17）**：`thigh_angle` 原来错误使用 `phi_a + π/2`（下连杆绝对角），后髋运动会干扰大腿角。修正为 `Leg_Wrap(cache->qf)`，即前髋上连杆角，只有前髋动才改变。已实机验证。

**极性**：DM/DJI 驱动反馈层统一到机体坐标系，右前髋、右后髋和右轮的物理角度/速度取反；五连杆输入不再重复做右腿镜像。现有 `config.mirror` 保持 +1，后续清理前不得设置为 -1。

**输出**：`leg_output_t` 含 thigh_angle/l0/phi0/virtual_shank/各雅可比/force_map/valid

VOFA 当前 32 通道（上限 32）用于全链路诊断：`dbg[0]` 为在线掩码，`dbg[1]` 为解算有效掩码（1=左腿, 2=右腿, 3=两腿），`dbg[2]` 为控制策略，`dbg[3..20]` 为 6 个虚拟关节 PID（当前/目标/输出，含两个轮），`dbg[21..24]` 为四台腿电机下发力矩，`dbg[25..26]` 为轮子下发力矩，`dbg[27..28]` 为腿长，`dbg[29..31]` 为小腿雅可比。通道布局详见 `md/VOFA_SEND.md`。

DM 反馈层已对右侧电机取反（`feedback_sign`），力矩下发按 `output_sign` 在 `dm.c` 边界取反，使逻辑侧正力矩与左右实体电机的正运动方向一致。

旧 F/T 与手动 action 测试入口已删除，执行链只保留策略动作→虚拟关节 PID→雅可比→实际电机。

### 3.2 观测构建 (`rl_observation`)

25 维观测 + 5 帧历史 = 125 维输入

| 索引 | 内容 | 缩放 |
|------|------|------|
| 0-2 | 陀螺仪角速度 (rad/s) | × gyro_scale（待配） |
| 3-5 | 投影重力 (机体坐标系) | × 1.0 |
| 6-8 | 指令 [vx, yaw_rate, height] | × command_scale（待配） |
| 9-12 | 关节角度偏差 (4 腿关节 - 中位) | × 1.0 |
| 13-18 | 关节角速度 (6 维含轮子) | × joint_vel_scale（待配） |
| 19-24 | 上步动作 (6 维) | × 1.0 |

投影重力：`g_body = R^T * [0,0,-1]^T`，用四元数旋转计算。

历史堆叠：循环左移，`[t-4, t-3, t-2, t-1, t]` 五帧。

**当前阻塞**：`rl_control.param.configured` 未置 1，`obs_dof_pos`/`command_scale`/`gyro_scale`/`joint_vel_scale` 全部为零。需训练同学提供参数后填入。

### 3.3 推理 (`rl_policy`)

4 个 CubeAI 模型，按策略切换：

| 策略 | 模型 | 用途 |
|------|------|------|
| Stable | stable.onnx | 站立平衡 |
| MiniRecover/Spin | pin.onnx | 小陀螺 |
| Upstairs | upstairs.onnx | 上楼梯/行走 |
| Jump | jump.onnx | 跳跃 |

每个模型：双输入 `[obs(25), history(125)]` → 单输出 `[action(6)]`。

静态分配激活缓存（`AI_ALIGNED(4)`），无堆内存。CRC 外设时钟需使能。

**当前状态**：`ctrl_task_body()` 构建观测但**不调用 `RL_Policy_Run()`**，手动遥操模式直接把遥控器偏移当 action 赋给 `action_state`。待观测参数配齐后再开启推理。

### 3.4 力矩执行 (`rl_torque`)

6 维动作 → 6 个电机力矩的完整转换：

```
1. 动作直接使用 (无额外限幅)
2. PD 控制:
   腿关节(4维): pos_ref = act × 0.5 + dof_pos
   轮子(2维):   vel_ref = act × 20.0
   tau_v = Kp×(pos_ref - q)，关节开启角度环绕
   轮子: tau_v = PID(vel_current, vel_ref)
3. 虚拟→实际 (vshank_jac):
   tau_front_hip = tau_thigh + tau_shank × vshank_jac[1]
   tau_rear_hip  = tau_shank × vshank_jac[0]
4. 输出限幅:
   腿 ±5Nm, 轮 ±5Nm (Jump 轮 ±4Nm)
```

力矩输出结构 `torque_output_t` 将 DM 与 DJI 分离（已重构）：
- `dm[DM_MOTOR_NUM]` — 4 个髋关节力矩，按 DM 驱动索引（F_LFT/B_LFT/F_RGT/B_RGT）
- `dji[DJI_MOTOR_NUM]` — 2 个轮子力矩，按 DJI 驱动索引（WHEEL_LFT/WHEEL_RGT）
- 右轮极性在 `dji.c` 驱动边界按 `output_sign` 处理，调用方不再取反（`tau_v[VJ_R_WHEEL]` 直接传）
- `task_actuation.c` 直接调用 `Dm_Send_Torque(torque.dm)` + `Dji_Send_Wheel_Torque(torque.dji[0], torque.dji[1])`，无手动索引映射

PID 参数按模型存表，具体数值以 `RL_Torque_Param_Init()` 为准。控制器在总初始化和模型切换时逐个调用 `PID_struct_init`，不做运行时参数同步。当前不包含气弹簧补偿和输出斜率限制。

**当前 PID 参数快照（以代码为准）：**

| 模型 | 关节 Kp | 关节 Kd | 轮子 Kp | 轮子 Ki | 轮子 Kd |
|------|---------|---------|---------|---------|---------|
| Stable | 3.5 | 0 | 8.0 | 0 | 0 |
| Pin | 2.0 | 0 | 5.0 | 0 | 0 |
| Jump | 2.0 | 0 | 5.0 | 0 | 0 |

**已修正**：DJI 力矩常数的减速比因子已修——`per_raw` 按 `machine->dji_gear_ratio` 缩放（见 `dji.c` 的 `Dji_Torque_To_Current`）。剩余：**Kt 绝对值仍待台架实测**（悬臂挂砝码/弹簧秤法）。型号/刻度/减速比已集中到 `imcalib/user-lib/machine_config.h`。

### 3.5 遥控映射 (`task_policy`)

当前为**手动遥操模式**（RL 推理未启用），遥控器直接控制关节偏移：

| 通道 | 输入 | 映射 |
|------|------|------|
| ch3（左Y） | 大腿偏移 | ×4.0 叠加到 base_action |
| wheel（拨轮） | 小腿偏移 | ×4.0 叠加到 base_action |
| ch1（右Y） | 轮子速度 | ×4.0 直接赋值（宽死区100） |
| ch0（右X） | yaw 指令 | ×REMOTE_COMMAND_SCALE → obs |

使能边沿锁存当前关节角为 base_action，后续摇杆在此基础上偏移。

### 3.6 使能与安全 (`task_comm`)

**使能状态机**：
- 遥控 s1 中/上 = 使能请求
- s1 下 / 任一故障 / 翻倒 = 失能
- 使能请求 + 动作不新鲜(>100ms) → FAULT_ACTION → 失能

**故障位**：`FAULT_IMU | FAULT_RC | FAULT_MOTOR | FAULT_CAN | FAULT_ACTION`

**翻倒**：|pitch| > 1.4rad 置 fallen，< 1.0rad 回正（回差）

**总开关**：`torque_output_enabled`（当前测试初始化为 1）

`torque_output_enabled=0` 时执行任务保持 DJI 零电流和 DM 零力矩；遥控、动作、IMU、CAN、电机在线与翻倒保护仍有效。

### 3.7 LQR 平衡模式

actuationTask 里新增了策略仲裁：**左拨杆中位 = LQR 平衡，上位 = 手动遥操/RL，下位 = 失能**。

LQR 链路（`lqr_balance.c` + `leg_balance.c`）与 RL 链路完全解耦，只在 `task_actuation.c` 的分支处交汇，两条链路互不 include。完整设计、参数来源、台架验证顺序与遗留项见 **[LQR_PLAN.md](LQR_PLAN.md)**。

要点速记：
- LQR 是**第一套真正能站的自动控制器**（RL 推理尚未启用）
- 遥控在 LQR 模式下换语义：右摇杆 X=转向，右摇杆 Y=前后速度，拨轮=升降
- LQR 模式不检查 `base_action_locked`，改查 `imu_state.online && leg_l.valid && leg_r.valid`
- LQR 不满足条件时直接零力矩，**不自动降级**到别的策略
- LQR 的腿长/横滚/防劈叉 PID 的 KD 是按 500Hz 折算过的，改频率要同步改

---

## 四、当前进度 — 已完成 vs 待完成

### 当前完成情况

| 模块 | 文件 | 状态 |
|------|------|------|
| FDCAN 总线 | can_bus.c/h | ✅ 路由注册 + 批量接收 + bus-off 恢复 + RX 看门狗 |
| DM 电机 | dm.c/h | ✅ MIT 协议 + 解码 + 在线检测 + 多圈计数 |
| 机器配置表 | machine_config.c/h | ✅ 新增，两份表 + 运行时切换（M3508+J8009P / M2006+J4310） |
| 单调 ns 时钟 | mono_ns.c/h | ✅ 新增，DWT CYCCNT + 500Hz 周期扩展 |
| DJI 轮电机 | dji.c/h | ✅ 电流控制 + 解码 + 在线检测 + 减速比修正 |
| UART 底层 | uart_idle.c/h | ✅ IDLE+DMA Circular |
| DR16 遥控 | dr16.c/h | ✅ 解析 + 实测正常 |
| HI229 IMU | hi229.c/h | ✅ 通信 + 数据提取 |
| 姿态解算 | Attitude_Algorithm.c/h | ✅ Mahony + HI229 融合 |
| Vofa 调试 | Vofa_send.c/h | ✅ FireWater DMA 发送 |
| 五连杆 | leg_solver.c/h | ✅ 几何/腿长/腿角/虚拟小腿/雅可比/force_map/极性，含 thigh_angle 根因修复，已上机验证 |
| RL 观测 | rl_observation.c/h | ✅ 代码完成；🟡 缩放参数未配置（param.configured=0） |
| CubeAI 推理 | rl_policy.c/h | ✅ 4 模型初始化 + 运行 + 维度静态检查；🟡 推理未在任务中调用 |
| 力矩执行 | rl_torque.c/h | ✅ PID + 雅可比映射 + DM/DJI 分离输出 + 轮子 PID；已上机验证 |
| 任务框架 | task/robot_control.c + task_*.c | ✅ 4 任务体、共享状态、使能机、故障门、VOFA 32ch（上限 32）；已上机验证 |
| 遥控映射 | task_policy.c | ✅ ch3→大腿/wheel→小腿/ch1→轮子，手动遥操模式 |
| 力矩下发 | task_actuation.c | ✅ torque_output_t 直接下发 DM+DJI，无手动映射 |
| 离线测试 | tests/offline_test.c | ✅ 纯算法数值验证 |

### 待实测 / 待配置

| 项目 | 位置 | 说明 | 优先级 |
|------|------|------|--------|
| **观测缩放参数** | `rl_control.param` | gyro_scale/command_scale/joint_vel_scale/obs_dof_pos 需训练同学提供 | 🔴 P0 |
| **开启 RL 推理** | `task_policy.c` | param 配齐后加 `RL_Policy_Run()` 调用 | 🔴 P0 |
| **DJI 力矩常数** | `dji.h DJI_NM_FULL_*` + `dji_gear_ratio` | ✅ 减速比因子已修正；Kt 绝对值待实测（悬臂挂砝码法） | 🟢 P2 |
| **遥控缩放** | `task_policy.c` | MANUAL_ACTION_SCALE 和 REMOTE_COMMAND_SCALE 需与训练侧对齐 | 🟡 P1 |
| **模型切换状态机** | 未实现 | stable/pin/upstairs/jump 切换条件待定义 | 🟡 P1 |
| **跌倒恢复** | 未实现 | 只有翻倒标志，无自动起身 FSM | 🟡 P2 |
| **轮子电机毛刺** | DJI M2006 | 即使无 D 项也抖，可能需死区或低通滤波 | 🟡 P2 |

### ⚠️ 与参考实车的差异

| 项目 | 参考实车 | 本项目 | 影响 |
|------|----------|--------|------|
| IMU | BMI088 板载 SPI | HI229 外挂串口 | 姿态源不同，四元数约定可能需调整 |
| 髋电机 | DM8009P (54Nm) | DM J4310 (10Nm) | 力矩限幅不同，需确认是否够用 |
| 轮电机 | M3508 (16.33减速比) | M2006 (36减速比) | 减速比因子已按实机 gear_ratio 缩放；Kt 绝对值待实测 |
| 功率控制 | 超电 + RLS 自适应 | 无 | 暂无功率限制 |
| 大型自起 | LargeRecover FSM | 无 | 只有翻倒标志，无自动恢复 |
| 小陀螺动作延迟 | Spin 模式延迟 1 周期 | 无 | Spin 策略效果可能不同 |
| ToF 跳跃触发 | ToF 测距触发跳跃 | 无 | Jump 策略手动触发 |
| D-Cache | 开启 + Clean 处理 | 关闭 | 需注意 CubeAI 是否自动开启 |
| FPU Error | 关闭 | 未确认 | 需在 CubeMX 中关闭 |

---

## 五、实测打开顺序

Leg_Solve 当前已完成以下验证：

```text
输入极性与前髋 +π 几何零位
→ 腿长与虚拟腿摆角
→ 虚拟小腿角
→ thigh_angle 根因修复 (phi_a → qf)
→ 速度与解析雅可比
→ 虚拟 F/T 到实际髋电机力矩
→ 左右腿实体输出极性
→ 轮子极性与减速比修正
→ torque_output_t DM/DJI 分离重构
→ 低力矩上机运动（手动遥操模式）
```

后续 RL 整链路仍按以下顺序进行，任何一步失败则停止：

```
① 观测参数配置 (param.configured = 1)
   → 与训练同学对齐 gyro_scale/command_scale/joint_vel_scale/obs_dof_pos

② 开启 RL 推理 (RL_Policy_Run)
   → 确认 CubeAI 4 模型初始化成功，action 6 维 finite

③ DJI 力矩常数实测
   → 悬臂挂砝码法确认 NM_PER_RAW

④ 低力矩 RL 闭环
   → 先用小 Kp 验证力矩方向正确

⑤ 模型切换状态机
   → 定义 stable/pin/upstairs/jump 切换条件

⑥ 实机验证
   → 逐步提高增益，验证平衡/行走/小陀螺/跳跃
```

---

## 六、踩坑备忘（来自参考实车）

1. **CubeMX FPU Error**：关掉，否则 CubeAI 跑着跑着就进异常中断
2. **CubeMX 时间配置**：每次点开 CubeAI 栏目后弹窗选 **No**，否则时钟被改
3. **syscalls.c 被删**：CubeAI 启用后重新 generate 会删此文件，需提前重命名备份
4. **D-Cache 与 DMA**：CubeAI 可能自动开启 D-Cache，导致 DMA 读旧数据
5. **电机偏置**：必须在 SolidWorks 中测量，实机零点→策略零点的偏置角
6. **欠压保护**：8009P 最好用 V3 版本（12V 以下才进保护），J4310 需确认保护电压
7. **DM/DJI 混用隐患**：不要把 DM 和 DJI 混合成同一个 motor 数组，索引混淆会导致力矩写错电机（已踩坑，已重构为 `torque_output_t` 分离）

---

## 七、关键约束速查

- **时钟**：HSE 24MHz → PLL → SYSCLK 240MHz
- **FDCAN**：1Mbps = Prescaler=12, Seg1=17, Seg2=2
- **BMI088**（本项目未使用，用 HI229）：驱动输出已是 rad/s 和 g
- **Mahony**：无 acc_trust 门控，无输出限幅
- **串口**：IDLE+DMA Circular，不使用 Resync
- **推理频率**：100Hz（每 5 个 500Hz 周期推理一次）
- **观测维度**：25 + 125(历史) = 150
- **动作维度**：6（左大腿, 左虚拟小腿, 左轮, 右大腿, 右虚拟小腿, 右轮）
- **物理通道**：DM×4（左前/左后/右前/右后髋） + DJI×2（左轮/右轮）
- **力矩限幅**：腿 ±5Nm，轮 ±5Nm（Jump 轮 ±4Nm）
- **DJI 减速比**：M2006 = 36，反馈 rpm 是转子转速，÷36 才是输出轴
