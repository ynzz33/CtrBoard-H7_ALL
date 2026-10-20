# 结构优化计划：倒地恢复优先，整机结构体后置

## 目标与冻结边界

本计划先完成倒地恢复链重构，再做整机结构体审计。目标是减少模块、长期状态和重复判断；不改变控制行为。

冻结项：

- 不改物理标定、极性、零点、几何、MIT量程和坐标定义。
- 不改倒地、倒置、触地、AIR和三态判据。
- 不改普通起立、翻身自救的阶段顺序、PID参数、轨迹、限幅、超时、重试、补偿与轮组零输出。
- 不改控制周期、积分/滤波推进次数、最终输出口和任务优先级。

## 当前链路与问题

当前链路已经是：

```text
fallen_detection → control_condition → ground_recovery → torque
```

`ground_recovery` 已经持有普通起立、倒置翻身、阶段机、目标、PID、映射、补偿和临时候选；`standup.c/h` 已删除。动作状态分为生命周期、普通起立路径、翻身路径和执行器历史，避免字段扁平堆叠但不引入访问宏。

已完成的安全删减：

- `control_condition_t` 删除重复姿态字段，改读 `fallen_detection_t`。
- `ground_recovery_ctx_t` 删除 `need`、`pose`、`upright`、`ready_block`、`angle_speed_cmd`、`prepare`、`raw_dm`、跨函数临时 `force/tp`。
- 删除无引用状态枚举和无效 `allow_restart` 参数。

## 目标动作层结构

重构后只保留两个恢复相关模块：

```text
fallen_detection.c/h
    只检测有效性、普通异常、倒置和upright

ground_recovery.c/h
    只处理已确认 FALLEN
    ├─ 普通起立阶段：EXTEND / REAR / RETRACT / SWING
    ├─ 倒置翻身阶段：SETTLE / TUCK / FLIP
    ├─ 共用 PID、目标、补偿、映射与失败重试
    └─ 输出 PREPARE / BALANCE / STOP
```

`ground_recovery.c/h` 是唯一公开动作接口，`control_compute` 继续只调用一个动作入口：

```c
route = Ground_Recovery_Update(&ground_recovery, &frame->imu,
                               &frame->left, &frame->right,
                               &frame->compensation, permit,
                               MACHINE_LQR_DT, torque);
```

## 实际状态结构

只保留一个恢复动作状态和一个参数结构，不新增普通起立/翻身各自的全局实例。

```c
typedef struct {
    ground_recovery_lifecycle_t lifecycle;
    ground_recovery_normal_t normal;
    ground_recovery_self_right_t self_right;
    ground_recovery_actuator_t actuator;
} ground_recovery_ctx_t;
```

以下量改为函数局部变量，不保存跨拍：

```text
angle_speed_target
base_dm[4]
prepared torque_output_t
本拍 force[2] / tp[2]
ready_block
```

`force / tp` 当前用于 PID 计算、恢复限幅和映射之间的短距离传递；迁移时合并为一次 `Ground_Recovery_Compute_Output()`，在同一函数中先算、限幅、映射，避免再存入状态结构。

## 函数结构

当前 `standup.c` 的复杂点不在 PID 数量，而在 `Standup_Stage_Update()` 同时处理失败重试、普通起立、倒置翻身、超时和阶段交接；`Standup_Target_Update()` 与 `Standup_PID_Calculate()` 又分散同一阶段的目标和控制量。迁移后不保留同义包装函数，收敛为以下函数结构：

```c
uint8_t Ground_Recovery_Update(...);      /* 唯一公开入口 */

static uint8_t Ground_Recovery_Advance(...);
static uint8_t Ground_Recovery_Normal_Advance(...);
static uint8_t Ground_Recovery_Self_Right_Advance(...);
static uint8_t Ground_Recovery_Compute_Output(...);
static uint8_t Ground_Recovery_Map_Output(...);
```

每个函数的边界固定如下：

| 函数 | 保留的逻辑 | 合并/删除的旧函数 | 不改变的行为 |
| --- | --- | --- | --- |
| `Ground_Recovery_Update` | 输入有效性、调用顺序、PREPARE/BALANCE/STOP 输出 | 原 `Standup_Update` | 调用次数、失败清零与交接时机 |
| `Ground_Recovery_Advance` | 失败重试、倒置优先、普通/翻身分派 | 原 `Standup_Stage_Update` 的公共部分 | retry_wait、最大重试、permit 门控 |
| `Ground_Recovery_Normal_Advance` | EXTEND/REAR/RETRACT/SWING 的超时、到位和目标 | 原普通阶段判断与 `Standup_Target_Update` 的普通分支 | 阶段顺序、rear 连续角路径、stable_time |
| `Ground_Recovery_Self_Right_Advance` | SETTLE/TUCK/FLIP 的目标、扫腿、卡滞与回正交接 | `Standup_Recovery_Enter/Ready/Retry/Stage/Target` | 扫腿速率、stall、回正窗口、翻身重试 |
| `Ground_Recovery_Compute_Output` | 原长度 PID、摆角串级 PID、support 渐入和翻身力限幅 | `Standup_Target_Update` 的控制目标部分与 `Standup_PID_Calculate` | PID 历史预置、Ramp、F/Tp 公式与限幅 |
| `Ground_Recovery_Map_Output` | `Leg_Force_Map_Forward`、补偿、最终候选发布 | `Standup_Torque_Output` | 映射、弹簧/重力叠加、失败不发布部分结果 |

具体简化原则：

```text
1. 阶段函数直接写对应 phase 的目标，取消“先生成目标、再由另一大函数按 phase 找目标”的往返。
2. force / tp / base_dm / prepared 全部为本拍局部变量，从 Compute_Output 直接传给 Map_Output。
3. rear 连续角、flip 扫描和 PID/斜坡仍保留跨拍状态；不能为了短函数改成每拍重新计算。
4. 普通与翻身只分阶段逻辑，不复制 PID、补偿、映射或最终输出。
5. 不保留仅转调、未读取参数或仅为“以后可能扩展”而存在的函数。
```

迁移后的公开调用不变：

```c
route = Ground_Recovery_Update(&ground_recovery, &frame->imu,
                               &frame->left, &frame->right,
                               &frame->compensation, permit,
                               MACHINE_LQR_DT, torque);
```

行为对照顺序必须与原代码一致：

```text
输入有效性
→ 失败/重试
→ 倒置优先与阶段推进
→ PREPARE/BALANCE/STOP 判断
→ 目标与 PID
→ 翻身力限幅
→ 映射/补偿
→ 候选发布
```

## 实施结果

1. 已在 `ground_recovery.c/h` 中迁入 `standup.c/h` 的状态、参数与实现，并更新初始化、选路和工程源登记。
2. 已将 `force/tp` 收进一次输出计算，删除旧文件及所有旧公开符号。
3. 已完成子结构分组；控制语句顺序未重排。普通起立、倒置翻身、失败重试及DONE→LQR仍需台架继续确认。

## 后置：整机结构体优化

倒地恢复重构完成后再按风险由低到高处理：

1. 删除 `lqr_state.u_col / u_hip_col`，当前只写无读。
2. 删除 `robot_state.rc_enable`，当前只写无读。
3. 删除 `feedback_metadata_t` 及 `robot_feedback` 的无人消费元数据链。
4. 将完整 `air_debug` 收紧为当前 VOFA 实际读取的遥测结构。
5. 逐项审查反馈快照的时间/原始电流副本；驱动原始反馈与跨任务快照不合并。

每一项都单独全量编译，检查 eIDE sourceList、compile_commands、objlist 和当前 VOFA 通道。
