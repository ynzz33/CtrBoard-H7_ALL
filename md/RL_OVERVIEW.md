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

**输入**：左右各 2 个 DM 髋电机的角度 + 角速度（hip_f=前髋，hip_b=后髋）。腿长和腿角由前后髋共同决定，不能把两台电机简称为“大腿/小腿电机”。

**计算**：
- 闭链几何求解足端 P → 腿长 l0、腿角 phi0
- 虚拟小腿角 `virtual_shank = wrap(phi_a - qf - π/2)`（相对前髋电机）
- 雅可比：`point_jac`（足端直角坐标）、`leg_jac`（极坐标）、`vshank_jac`（虚拟小腿）、`force_map`（力域转换）

**极性**：DM/DJI 驱动反馈层统一到机体坐标系，右前髋、右后髋和右轮的物理角度/速度取反；五连杆输入不再重复做右腿镜像。现有 `config.mirror` 保持 +1，后续清理前不得设置为 -1。

**输出**：`leg_output_t` 含 l0/phi0/virtual_shank/各雅可比/force_map/valid

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

**总开关**：`torque_output_enabled`（默认 0，实测时置 1）

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
| 五连杆 | leg_solver.c/h | ✅ 闭链求解 + 雅可比 + 镜像 + 门控 |
| RL 观测 | rl_observation.c/h | ✅ 25 维构建 + 5 帧历史 + 参数门控 |
| CubeAI 推理 | rl_policy.c/h | ✅ 4 模型初始化 + 运行 + 维度静态检查 |
| 力矩执行 | rl_torque.c/h | ✅ PD + 雅可比映射 + 气弹簧 + 限幅 + 斜率限制 |
| 任务框架 | robot_tasks.c/h | ✅ 4 任务体 + 使能机 + 故障门 + VOFA 32 通道 |
| 离线测试 | tests/offline_test.c | ✅ 纯算法数值验证 |

### ❌ 待实测 / 待配置

| 项目 | 位置 | 说明 | 优先级 |
|------|------|------|--------|
| **电机映射** | `leg_map_l/r` | 左前/后髋=DM0/DM1，右前/后髋=DM2/DM3 已登记；几何参数仍未配置 | 🟡 P0 |
| **五连杆参数** | `leg_l/r.config` | lu/lg/offset_f/offset_b，需 SolidWorks 测量 | 🔴 P0 |
| **电机偏置** | `leg_config.offset_f/b` | 实机零点→策略零点的偏置角 | 🔴 P0 |
| **观测参数** | `rl_control.param` | obs_dof_pos/command_scale/gyro_scale/joint_vel_scale | 🔴 P0 |
| **气弹簧** | `rl_torque_param_t.gas_spring` | 实测标定当前为 0（参考值 370.1） | 🟡 P1 |
| **DJI 力矩常数** | `dji.h DJI_NM_PER_RAW_*` | Kt/满量程电流实测核对 | 🟡 P1 |
| **DR16 指令映射** | commTask | ch3→vx、ch0→yaw、wheel→height；±20 死区、±660 限幅，s2 上位允许高度指令；物理单位待 RL 对接 | 🟡 P1 |
| **轮速符号** | `RL_Control_Update_Observation` | 观测侧轮速取反方向待实测 | 🟡 P1 |
| **torque_output_enabled** | robot_tasks.c | 全局下发开关，实测时置 1 | 🔴 P0 |

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

严格按此顺序，任何一步失败则停止：

```
① 电机映射 (leg_map_l/r.configured = 1)
   → 确认 DM/DJI 电机 CAN 反馈正常，在线检测通过

② 五连杆有效 (leg_l/r.config.configured = 1)
   → 确认 Leg_Solve 返回 valid=1，l0/phi0 合理

③ 观测有效 (rl_control.param.configured = 1)
   → 确认 obs 25 维全 finite，history_ready=1

④ 推理有效
   → 确认 CubeAI 4 模型初始化成功，action 6 维 finite

⑤ 低力矩输出 (torque_output_enabled = 1)
   → 先用小 Kp/Kd 验证力矩方向正确

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
