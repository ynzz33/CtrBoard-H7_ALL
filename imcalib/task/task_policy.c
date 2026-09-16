#include "robot_control.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include "dr16.h"

#include <string.h>

#define MANUAL_ACTION_SCALE 6.0f

static uint8_t base_locked;
static uint8_t was_enabled;
static float base_action[RL_ACTION_SIZE];

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

/* 构建观测 */
static uint8_t RL_Control_Update_Observation(void)
{
    float command[3];
    float joint_pos[4];
    float joint_vel[6];
    uint8_t source_valid;

    command[0] = input_command.vx_cmd;
    command[1] = input_command.yaw_cmd;
    command[2] = input_command.height_cmd;
    joint_pos[0] = leg_l.input.hip_f;
    joint_pos[1] = leg_l.output.virtual_shank;
    joint_pos[2] = leg_r.input.hip_f;
    joint_pos[3] = leg_r.output.virtual_shank;
    joint_vel[0] = leg_l.input.d_hip_f;
    joint_vel[1] = leg_l.output.d_virtual_shank;
    joint_vel[2] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    joint_vel[3] = leg_r.input.d_hip_f;
    joint_vel[4] = leg_r.output.d_virtual_shank;
    joint_vel[5] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    source_valid = (uint8_t)(imu_state.online && leg_l.output.valid
        && leg_r.output.valid && RL_Motors_Online());
    if (!RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu_state.input.gyro_rad_s, imu_state.output.quat, command,
        joint_pos, joint_vel, source_valid))
    {
        return 0u;
    }
    RL_Observation_Update_History(&rl_control.observation);
    return rl_control.observation.history_ready;
}

/* 摇杆归一化 */
static float Manual_Axis(int16_t raw)
{
    int16_t value;

    value = DR16_Deadline(raw, 20u);
    if (value > DR16_CH_LIMIT)
    {
        value = DR16_CH_LIMIT;
    }
    else if (value < -DR16_CH_LIMIT)
    {
        value = -DR16_CH_LIMIT;
    }
    return (float)value / (float)DR16_CH_LIMIT;
}

/*
 * 使能边沿立刻锁存当前角度作为 base_action
 * 不再等待200ms，因为锁存前不输出力矩，腿不会偏移
 */
static void Manual_Lock_On_Enable(void)
{
    const rl_torque_param_t *param;

    if (robot_state.enabled && !was_enabled)
    {
        /* 使能边沿：立刻锁存 */
        if (leg_l.output.valid && leg_r.output.valid)
        {
            param = &rl_control.torque_param[rl_control.policy.selected_model];
            base_action[0] = (leg_l.input.hip_f - param->dof_pos[0]) * 2.0f;
            base_action[1] = (leg_l.output.virtual_shank - param->dof_pos[1]) * 2.0f;
            base_action[2] = 0.0f;
            base_action[3] = (leg_r.input.hip_f - param->dof_pos[3]) * 2.0f;
            base_action[4] = (leg_r.output.virtual_shank - param->dof_pos[4]) * 2.0f;
            base_action[5] = 0.0f;
            base_locked = 1u;
        }
        else
        {
            base_locked = 0u;
        }
    }
    else if (!robot_state.enabled)
    {
        base_locked = 0u;
    }
    was_enabled = robot_state.enabled;
}

/* 摇杆偏移叠加 */
static void Manual_Action_Apply(float action[RL_ACTION_SIZE])
{
    dr16_t remote;
    float stick_thigh;
    float stick_shank;

    if (!base_locked)
    {
        return;
    }
    DR16_Process();
    remote = DR16_Snapshot();
    if (!remote.online)
    {
        return;
    }
    stick_thigh = Manual_Axis(remote.ch3) * MANUAL_ACTION_SCALE;
    stick_shank = Manual_Axis(remote.wheel) * MANUAL_ACTION_SCALE;
    action[0] = base_action[0] + stick_thigh;
    action[1] = base_action[1] + stick_shank;
    action[3] = base_action[3] + stick_thigh;
    action[4] = base_action[4] + stick_shank;
}

/* 策略初始化 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}

/* 策略单周期 */
void ctrl_task_body(void)
{
    float action[RL_ACTION_SIZE] = {0};

    RL_Control_Update_Observation();
    Manual_Lock_On_Enable();

    memcpy(action, base_action, sizeof(action));
    Manual_Action_Apply(action);

    RL_Observation_Set_Last_Action(&rl_control.observation, action);
    memcpy(action_state.a, action, sizeof(action_state.a));
    action_state.updated = 1u;
    action_state.last_ok_tick = HAL_GetTick();
    action_state.base_action_locked = base_locked;
}