#include "robot_control.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include "machine_config.h"
#include "pid.h"

#include <string.h>

/*
 * 策略任务两条路径 (rl_control.infer_enable 切换, 默认 0):
 *   0 手动遥操: 摇杆偏移当动作 (旧路径, 对照用)
 *   1 推理:     RL 投入 (左上 + 右中 + 使能) → 预热 RL_WARMUP_STEPS 步零动作 (PD + 历史照跑)
 *               → networkzn1 推理; 未投入 / 观测无效 / 推理失败 → 零动作且 rl_ready=0 (零力矩)
 * 观测关节 = 固件角经机器表 .rl 映射到训练关节; 发布的动作 = 训练动作乘同一 sign 回固件关节
 */

#define MANUAL_ACTION_SCALE 4.0f

static uint8_t base_locked;
static uint8_t was_enabled;
static float base_action[RL_ACTION_SIZE];
static uint16_t warmup_cnt;     /* 预热计数 */

/* 检查电机 */
static uint8_t RL_Motors_Online(void)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i])
        {
            return 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i])
        {
            return 0u;
        }
    }
    return 1u;
}

/* 固件关节 → 训练关节: q_t = sign × wrap(q − zero), 速度同符号; 映射未配置返回 0 */
static uint8_t RL_Joint_Map(float joint_pos[4], float joint_vel[6])
{
    const rl_map_t *map = &machine->rl;
    float qd[6];

    if (!map->configured)
    {
        return 0u;
    }
    joint_pos[0] = (float)map->sign[0] * Angle_Wrap_180(leg_l.output.thigh_angle - map->zero[0]);
    joint_pos[1] = (float)map->sign[1] * Angle_Wrap_180(leg_l.output.virtual_shank_angle - map->zero[1]);
    joint_pos[2] = (float)map->sign[3] * Angle_Wrap_180(leg_r.output.thigh_angle - map->zero[2]);
    joint_pos[3] = (float)map->sign[4] * Angle_Wrap_180(leg_r.output.virtual_shank_angle - map->zero[3]);
    qd[0] = leg_l.input.d_hip_f;
    qd[1] = leg_l.output.d_virtual_shank_angle;
    qd[2] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    qd[3] = leg_r.input.d_hip_f;
    qd[4] = leg_r.output.d_virtual_shank_angle;
    qd[5] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    for (uint8_t i = 0u; i < 6u; i++)
    {
        joint_vel[i] = (float)map->sign[i] * qd[i];
    }
    return 1u;
}

/* 构建观测 + 推历史; 源无效或映射未配置 → 观测清零返回 0 */
static uint8_t RL_Control_Update_Observation(const float command[3])
{
    float joint_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float joint_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t source_valid;

    source_valid = (uint8_t)(imu_state.online && leg_l.output.valid
        && leg_r.output.valid && RL_Motors_Online()
        && RL_Joint_Map(joint_pos, joint_vel));
    if (!RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu_state.gyro_rad_s, imu_state.quat, command,
        joint_pos, joint_vel, source_valid))
    {
        return 0u;
    }
    RL_Observation_Update_History(&rl_control.observation);
    return rl_control.observation.history_ready;
}

/* 发布动作 (固件关节空间) */
static void RL_Action_Publish(const float action[RL_ACTION_SIZE],
                              uint8_t manual_locked, uint8_t rl_ready)
{
    memcpy(action_state.a, action, sizeof(action_state.a));
    action_state.updated = 1u;
    action_state.last_ok_tick = HAL_GetTick();
    action_state.base_action_locked = manual_locked;
    action_state.rl_ready = rl_ready;
}

/*
 * 使能边沿立刻锁存当前角度作为 base_action
 * 不再等待200ms，因为锁存前不输出力矩，腿不会偏移
 */
static void Manual_Lock_On_Enable(void)
{
    const rl_torque_param_t *param;

    if (robot_state.motor_enabled && !was_enabled)
    {
        if (leg_l.output.valid && leg_r.output.valid)
        {
            param = &rl_control.torque_param[rl_control.policy.selected_model];
            base_action[0] = (leg_l.input.hip_f - param->dof_pos[0]) * 2.0f;
            base_action[1] = (leg_l.output.virtual_shank_angle - param->dof_pos[1]) * 2.0f;
            base_action[2] = 0.0f;
            base_action[3] = (leg_r.input.hip_f - param->dof_pos[3]) * 2.0f;
            base_action[4] = (leg_r.output.virtual_shank_angle - param->dof_pos[4]) * 2.0f;
            base_action[5] = 0.0f;
            base_locked = 1u;
        }
        else
        {
            base_locked = 0u;
        }
    }
    else if (!robot_state.motor_enabled)
    {
        base_locked = 0u;
    }
    was_enabled = robot_state.motor_enabled;
}

/*
 * 统一遥控处理: 更新 input_command + 手动偏移叠加
 * 指令由 commTask 的 Rc_Command_Update() 解算, 此处只读
 */
static void Remote_Command_Apply(float action[RL_ACTION_SIZE])
{
    float stick_thigh;
    float stick_shank;
    float stick_wheel;

    if (!rc_command.online)
    {
        return;
    }

    /* RL obs 指令 */
    input_command.vx_cmd     = rc_command.vel * REMOTE_COMMAND_SCALE;
    input_command.yaw_cmd    = rc_command.yaw * REMOTE_COMMAND_SCALE;
    input_command.height_cmd = rc_command.len * REMOTE_COMMAND_SCALE;

    /* 手动偏移叠加 */
    if (base_locked)
    {
        stick_thigh = rc_command.ang * MANUAL_ACTION_SCALE;
        stick_shank = rc_command.len * MANUAL_ACTION_SCALE;
        stick_wheel = rc_command.vel * MANUAL_ACTION_SCALE;
        action[0] = base_action[0] + stick_thigh;
        action[1] = base_action[1] + stick_shank;
        action[2] = stick_wheel;
        action[3] = base_action[3] + stick_thigh;
        action[4] = base_action[4] + stick_shank;
        action[5] = stick_wheel;
    }
}

/* 遥控 → 策略指令: 前进 / 转向 / 高度 (范围 RL_CMD_*; 转向右推为负, 同 LQR) */
static void RL_Command_From_Rc(float command[3])
{
    command[0] = rc_command.vel * RL_CMD_VX_MAX;
    command[1] = -rc_command.yaw * RL_CMD_YAW_MAX;
    command[2] = RL_CMD_HEIGHT_MIN
               + (rc_command.len + 1.0f) * 0.5f * (RL_CMD_HEIGHT_MAX - RL_CMD_HEIGHT_MIN);
    input_command.vx_cmd = command[0];
    input_command.yaw_cmd = command[1];
    input_command.height_cmd = command[2];
}

/* 动作裁剪 (RL_ACTION_CLIP = 0 不裁) */
static void RL_Action_Clip(float action[RL_ACTION_SIZE])
{
    float limit = RL_ACTION_CLIP;

    if (limit <= 0.0f)
    {
        return;
    }
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = clampf(action[i], -limit, limit);
    }
}

/* 推理路径单周期 */
static void RL_Infer_Body(void)
{
    const rl_map_t *map = &machine->rl;
    float command[3];
    float action_t[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   /* 训练空间 */
    float action[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};     /* 固件空间 */

    /* 未投入: 清历史, 发零动作保持新鲜 (LQR 挡也走这里) */
    if (!output_task_rl_engaged())
    {
        warmup_cnt = 0u;
        rl_control.infer_phase = 0u;
        RL_Observation_Reset(&rl_control.observation);
        RL_Action_Publish(action, 0u, 0u);
        return;
    }

    RL_Command_From_Rc(command);
    if (!RL_Control_Update_Observation(command))
    {
        /* 观测无效: 重新预热, 零力矩 */
        warmup_cnt = 0u;
        rl_control.infer_phase = 0u;
        RL_Action_Publish(action, 0u, 0u);
        return;
    }

    if (warmup_cnt < RL_WARMUP_STEPS)
    {
        warmup_cnt++;
        rl_control.infer_phase = 1u;
    }
    else
    {
        rl_control.infer_phase = 2u;
        if (!RL_Policy_Run(&rl_control.policy, &rl_control.observation, action_t))
        {
            RL_Action_Publish(action, 0u, 0u);   /* 推理失败 */
            return;
        }
        RL_Action_Clip(action_t);
    }

    RL_Observation_Set_Last_Action(&rl_control.observation, action_t);
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = (float)map->sign[i] * action_t[i];   /* 训练 → 固件 */
    }
    RL_Action_Publish(action, 0u, 1u);
}

/* 策略初始化 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}

/* 策略单周期 */
void ctrl_task_body(void)
{
    float action[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float command[3];

    if (rl_control.infer_enable)
    {
        RL_Infer_Body();
        return;
    }

    /* 手动遥操 (旧路径) */
    Manual_Lock_On_Enable();
    Remote_Command_Apply(action);
    command[0] = input_command.vx_cmd;
    command[1] = input_command.yaw_cmd;
    command[2] = input_command.height_cmd;
    (void)RL_Control_Update_Observation(command);
    RL_Observation_Set_Last_Action(&rl_control.observation, action);
    RL_Action_Publish(action, base_locked, 0u);
}
