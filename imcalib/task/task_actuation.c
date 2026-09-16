#include "robot_control.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"

/*① 力矩运算: 门控 + RL_Torque_Compute → torque[6]
    返回 1 = 有效力矩, 0 = 安全停机*/
static uint8_t Motor_Torque_Compute(float torque[RL_TQ_NUM])
{
    float wheel_vel[2];
    uint8_t action_ready;

    memset(torque, 0, sizeof(float) * RL_TQ_NUM);

    if (!robot_state.rc_enable || !robot_state.motor_enabled)
    {
        return 0u;
    }

    action_ready = action_state.updated
        && (HAL_GetTick() - action_state.last_ok_tick) < 100u
        && action_state.base_action_locked;
    if (!action_ready || !torque_output_enabled)
    {
        return 0u;
    }

    wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    if (!RL_Torque_Compute(&leg_l, &leg_r,
        &rl_control.torque_param[rl_control.policy.selected_model],
        wheel_vel, action_state.a, &rl_control.torque_state, torque))
    {
        return 0u;
    }

    /* 调试: 只输出左腿 */
    torque[RL_TQ_R_THIGH] = 0.0f;
    torque[RL_TQ_R_SHANK] = 0.0f;
    torque[RL_TQ_L_WHEEL] = 0.0f;
    torque[RL_TQ_R_WHEEL] = 0.0f;

    return 1u;
}

/*力矩输出: 调试模式 — 只存储不发送*/
static void Motor_Torque_Output(const float torque[RL_TQ_NUM])
{
    (void)torque;
    output_debug_dm_sent = 0u;
    output_debug_dji_sent = 0u;
    (void)Dm_Send_Zero();
    (void)Dji_All_Stop();
}

/* 输出初始化 */
void output_task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/* 输出单周期 */
void output_task_body(void)
{
    float torque[RL_TQ_NUM];

    if (Motor_Torque_Compute(torque))
    {
        Motor_Torque_Output(torque);
    }
    else
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
    }
}
