# RL 部署总览 — 代码链路 & 进度

> 最后更新：2026-09-11
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
    → CubeAI推理 → 6维动作 → PD → 雅可比映射 → 气弹簧补偿 → 6个电机力矩
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
- 轮电机输出电流极性与反馈极性分层管理，后续修改轮子输出前必须单独台架确认，不得依据反馈符号重复取反。
- 遥控接收方向已验证；普通 RL 指令当前统一缩放到 `±0.1` 用于安全测试，最终策略缩放以训练参数确认后再更新。

以上定义已经过当前机械安装的腿部输入、腿长、腿角、虚拟小腿、速度/雅可比、虚拟力矩映射和实体运动联调确认。若物理行为再次异常，先检查反馈/下发链路和报文状态，不得直接改几何符号。

---

## 二、任务架构

5 个 FreeRTOS 任务，职责分明：

| 任务 | 频率 | 节拍方式 | 职责 |
|------|------|----------|------|
| `actuationTask` | 500Hz | TIM6 信号量（硬实时） | 读取已更新状态 → 力矩计算 → CAN 下发 |
| `policyTask` | 100Hz | osDelay | 观测构建 → CubeAI 推理 → 写 action_state |
| `imuTask` | 500Hz | osDelay(2ms) | HI229 新帧解析 → 姿态更新 → 写 imu_state |
| `commTask` | 1kHz | osDelay | DM/DJI/DR16 解析 → 状态更新 → 在线检测 → 故障位 → VOFA |
| `defaultTask` | - | - | USB 初始化（保留） |

**单写者模型**：每个共享状态只有一个任务写，32bit 对齐 float 在 M7 上读写原子，无需锁。

`freertos.c` 只维护任务初始化、循环和节拍等待；`robot_tasks.c` 的各 `*_task_body()` 每次只执行一个任务周期。

**数据流向**：
```
[ISR] FDCAN → dm/dji raw_pending
[ISR] UART  → hi229_rx.flag / dbus_rx.flag
[ISR] TIM6  → ctrl_tick_sem (500Hz 信号量)

imuTask     → imu_state {quat, eul, gyr, acc, online}
commTask    → command_state/motor_state/leg_state; ctrl_fault; VOFA
policyTask  → action_state {a[6], last_ok_tick, updated}
commTask    → motor_state/leg_state → actuationTask → 力矩 → CAN
```

---

## 三、RL 部署链路详解

### 3.1 五连杆运动学 (`leg_solver`)

**输入**：左右各 2 个 DM 髋电机的角度 + 角速度（hip_f=前髋，hip_b=后髋）。前髋的几何零位比 DM 反馈零位多 π，使“前后上连杆反向水平”的最短腿姿态对应虚拟腿竖直。腿长和腿角由前后髋共同决定，不能把两台电机简称为“大腿/小腿电机”。

**计算**：
- 闭链几何求解足端 P → 腿长 l0、腿摆角 phi0（前摆为正，含零点偏置）
- 虚拟小腿角 `virtual_shank = wrap(phi_a - qf - π/2)`（相对前髋电机）
- 雅可比：`point_jac`（足端直角坐标）、`leg_jac`（极坐标）、`vshank_jac`（虚拟小腿）、`force_map`（力域转换）

实现分为闭链几何、速度与雅可比、力矩映射三层，`Leg_Solve()` 仅负责按顺序调用三层。

**极性**：DM/DJI 驱动反馈层统一到机体坐标系，右前髋、右后髋和右轮的物理角度/速度取反；五连杆输入不再重复做右腿镜像。现有 `config.mirror` 保持 +1，后续清理前不得设置为 -1。

**输出**：`leg_output_t` 含 l0/phi0/virtual_shank/各雅可比/force_map/valid

VOFA 当前 32 通道用于 IMU、投影重力和轮速验证：`dbg[0..2]` 陀螺仪，`dbg[3..5]` 为翻转极性后的加速度，`dbg[6..9]` 四元数，`dbg[10..12]` 投影重力，`dbg[13..16]` 左右轮速度/多圈角，`dbg[17..19]` 小量缩放后的遥控指令，`dbg[20..23]` 拨杆和轮电流，`dbg[24..25]` IMU/重力有效位，`dbg[26..28]` 欧拉角，`dbg[29]` 投影重力模长，`dbg[30]` 四元数模长，`dbg[31]` 为电机在线与使能掩码。

DM 反馈层已对右侧电机取反，力矩下发同步取反，使逻辑侧正力矩与左右实体电机的正运动方向一致。

力矩映射台架测试：左拨杆上位进入测试，右拨杆上位选左腿、右拨杆中位选右腿，右拨杆下位不输出；左拨杆下位立即失能。`ch3` 映射径向力 F，`ch0` 映射切向力矩 T，测试范围和电机力矩均做严格限幅。仅当 `torque_output_enabled=1` 时下发。

### 3.2 观测构建 (`rl_observation`)

25 维观测 + 5 帧历史 = 125 维输入

| 索引 | 内容 | 缩放 |
|------|------|------|
| 0-2 | 陀螺仪角速度 (rad/s) | × gyro_scale (0.25) |
| 3-5 | 投影重力 (机体坐标系) | × 1.0 |
| 6-8 | 指令 [vx, yaw_rate, height] | × command_scale |
| 9-12 | 关节角度偏差 (4 腿关节 - 中位) | × 1.0 |
| 13-18 | 关节角速度 (6 维含轮子) | × joint_vel_scale (0.05) |
| 19-24 | 上步动作 (6 维) | × 1.0 |

投影重力：`g_body = R^T * [0,0,-1]^T`，用四元数旋转计算。

历史堆叠：循环左移，`[t-4, t-3, t-2, t-1, t]` 五帧。

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

### 3.4 力矩执行 (`rl_torque`)

6 维动作 → 6 个电机力矩的完整转换：

```
1. 动作限幅 ±100
2. PD 控制:
   腿关节(4维): pos_ref = act × 0.5
   轮子(2维):   vel_ref = act × 10
   tau_v = Kp×(pos_ref + dof_pos - q) + Kd×(vel_ref - qd)，限幅 ±1000
3. 虚拟→实际:
   tau_thigh = tau_v[thigh] + tau_v[vs]×jac[1]
   tau_shank = tau_v[vs]×jac[0]
4. 力域转换 + 气弹簧:
   反解到 (F, T) 域 → F -= mirror×gas×l0 → 映射回关节
5. 输出限幅:
   腿 ±35Nm, 轮 ±5Nm (Jump ±4Nm)
6. 斜率限制 (max_step, 0=关)
```

**PD 参数按模型存表**（与训练一致）：

| 策略 | Kp(腿) | Kd(腿) | dof_pos | 气弹簧(左/右) |
|------|--------|--------|---------|---------------|
| Stable | 15 | 1.0 | [-0.23, -0.65, 0, 0.23, 0.65, 0] | 0 / 0 |
| Pin | 10 | 1.0 | 同上 | 0 / 0 |
| Jump | 6 | 0.5 | [0.2, 0.4, 0, -0.2, -0.4, 0] | 0 / 0 |

### 3.5 使能与安全 (`robot_tasks`)

**使能状态机**：
- 遥控 s1 中/上 = 使能请求
- s1 下 / 任一故障 / 翻倒 = 失能
- 使能请求 + 动作不新鲜(>100ms) → FAULT_ACTION → 失能

**故障位**：`FAULT_IMU | FAULT_RC | FAULT_MOTOR | FAULT_CAN | FAULT_ACTION`

**翻倒**：|pitch| > 1.4rad 置 fallen，< 1.0rad 回正（回差）

**总开关**：`torque_output_enabled`（当前测试初始化为 1）

测试模式下 `torque_output_enabled=0` 时允许 DM 使能但保持零力矩；此时 `FAULT_ACTION` 不阻止使能，其余故障和翻倒保护仍有效。

---

## 四、当前进度 — 已完成 vs 待完成

### ✅ 已完成（代码写好，编译通过）

| 模块 | 文件 | 状态 |
|------|------|------|
| FDCAN 总线 | can_bus.c/h | ✅ 路由注册 + 批量接收 + bus-off 恢复 + RX 看门狗 |
| DM 电机 | dm.c/h | ✅ MIT 协议 + 解码 + 在线检测 + 多圈计数 |
| DJI 轮电机 | dji.c/h | ✅ 电流控制 + 解码 + 在线检测 |
| UART 底层 | uart_idle.c/h | ✅ IDLE+DMA Circular |
| DR16 遥控 | dr16.c/h | ✅ 解析（待实测映射） |
| HI229 IMU | hi229.c/h | ✅ 通信 + 数据提取 |
| 姿态解算 | Attitude_Algorithm.c/h | ✅ Mahony + HI229 融合 |
| Vofa 调试 | Vofa_send.c/h | ✅ FireWater DMA 发送 |
| 五连杆 | leg_solver.c/h | ✅ 几何、腿长、腿角、虚拟小腿、速度/雅可比、force_map、实际电机力矩极性及实体运动均已上机验证 |
| RL 观测 | rl_observation.c/h | 🟡 腿部字段、IMU、投影重力和轮速反馈已确认；缩放和观测参数仍待整体配置 |
| CubeAI 推理 | rl_policy.c/h | ✅ 4 模型初始化 + 运行 + 维度静态检查 |
| 力矩执行 | rl_torque.c/h | ✅ PD + 雅可比映射 + 气弹簧 + 限幅 + 斜率限制 |
| 任务框架 | robot_tasks.c/h | ✅ 4 任务体 + 使能机 + 故障门 + VOFA 32 通道 |
| 离线测试 | tests/offline_test.c | ✅ 纯算法数值验证 |

### ❌ 待实测 / 待配置

| 项目 | 位置 | 说明 | 优先级 |
|------|------|------|--------|
| **电机映射** | `leg_map_l/r` | 左前/后髋=DM0/DM1，右前/后髋=DM2/DM3，已完成台架确认 | ✅ |
| **五连杆参数** | `leg_l/r.config` | lu/lg/offset_f/offset_b 已用于当前机械并完成解算验证 | ✅ |
| **电机偏置** | `leg_config.offset_f/b` | 已完成几何零位校准；前髋几何输入包含 `+π` | ✅ |
| **观测参数** | `rl_control.param` | obs_dof_pos/command_scale/gyro_scale/joint_vel_scale 尚未完成整体验证 | 🔴 P0 |
| **气弹簧** | `rl_torque_param_t.gas_spring` | 实测标定当前为 0（参考值 370.1） | 🟡 P1 |
| **DJI 力矩常数** | `dji.h DJI_NM_PER_RAW_*` | Kt/满量程电流实测核对 | 🟡 P1 |
| **DR16 接收** | `dr16.c/h` | 数据帧接收与解析已实测正常 | ✅ |
| **DR16 指令缩放** | `commTask` | 通道映射已接通，物理缩放仍待确认 | 🟡 P1 |
| **投影重力** | `rl_observation.c/h` | 水平、俯仰、横滚和偏航 VOFA 实测通过 | ✅ |
| **轮速与轮子反馈极性** | `dji.c/h` / `RL_Control_Update_Observation` | 前进为正、后退为负，轮速和多圈角实测通过 | ✅ |
| **torque_output_enabled** | robot_tasks.c | 当前测试初始化为 1，低力矩输出链已完成上机运动确认 | ✅ |

### ⚠️ 与参考实车的差异

| 项目 | 参考实车 | 本项目 | 影响 |
|------|----------|--------|------|
| IMU | BMI088 板载 SPI | HI229 外挂串口 | 姿态源不同，四元数约定可能需调整 |
| 髋电机 | DM8009P (54Nm) | DM J4310 (10Nm) | 力矩限幅不同，需确认是否够用 |
| 轮电机 | M3508 (16.33减速比) | M2006 | 力矩常数不同，Kt 待实测 |
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
→ 速度与解析雅可比
→ 虚拟 F/T 到实际髋电机力矩
→ 左右腿实体输出极性
→ 低力矩上机运动
```

后续 RL 整链路仍按以下顺序进行，任何一步失败则停止：

```
① 电机映射 (leg_map_l/r.configured = 1)
   → 确认 DM/DJI 电机 CAN 反馈正常，在线检测通过

② 五连杆有效 (leg_l/r.config.configured = 1)
   → 确认 Leg_Solve 返回 valid=1，l0/phi0 合理

③ 投影重力与轮速极性
   → VOFA 确认重力方向、左右轮速度方向

④ 观测参数有效 (rl_control.param.configured = 1)
   → 确认 obs 25 维全 finite，history_ready=1

⑤ 推理有效
   → 确认 CubeAI 4 模型初始化成功，action 6 维 finite

⑥ 低力矩输出 (torque_output_enabled = 1)
   → 先用小 Kp/Kd 验证力矩方向正确

⑦ 实机验证
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
- **物理通道**：[L大腿, L小腿, R大腿, R小腿, L轮, R轮] = DM×4 + DJI×2
