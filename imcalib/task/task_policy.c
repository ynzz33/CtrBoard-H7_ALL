#include "robot_control.h"
#include "rl_observation.h"
#include "rl_policy.h"

#include <string.h>

#define MANUAL_ACTION_SCALE 4.0f

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
    joint_pos[1] = leg_l.output.virtual_shank_angle;
    joint_pos[2] = leg_r.input.hip_f;
    joint_pos[3] = leg_r.output.virtual_shank_angle;
    joint_vel[0] = leg_l.input.d_hip_f;
    joint_vel[1] = leg_l.output.d_virtual_shank_angle;
    joint_vel[2] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    joint_vel[3] = leg_r.input.d_hip_f;
    joint_vel[4] = leg_r.output.d_virtual_shank_angle;
    joint_vel[5] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    source_valid = (uint8_t)(imu_state.online && leg_l.output.valid
        && leg_r.output.valid && RL_Motors_Online());
    if (!RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu_state.gyro_rad_s, imu_state.quat, command,
        joint_pos, joint_vel, source_valid))
    {
        return 0u;
    }
    RL_Observation_Update_History(&rl_control.observation);
    return rl_control.observation.history_ready;
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

/* 策略初始化 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}

/* 策略单周期 */
void ctrl_task_body(void)
{
    float action[RL_ACTION_SIZE] = {0};

    Manual_Lock_On_Enable();
    Remote_Command_Apply(action);
    RL_Control_Update_Observation();

    RL_Observation_Set_Last_Action(&rl_control.observation, action);
    memcpy(action_state.a, action, sizeof(action_state.a));
    action_state.updated = 1u;
    action_state.last_ok_tick = HAL_GetTick();
    action_state.base_action_locked = base_locked;
}