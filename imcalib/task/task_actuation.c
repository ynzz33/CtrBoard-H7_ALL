#include "robot_control.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"

static void Motor_Output_Update(void)
{
    float torque[RL_TQ_NUM] = {0.0f};
    float wheel_vel[2];
    HAL_StatusTypeDef dm_status;
    uint8_t action_ready;

    if (!robot_state.rc_enable)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dji_All_Stop();
        (void)Dm_Send_Zero();
        return;
    }
    action_ready = action_state.updated
        && (HAL_GetTick() - action_state.last_ok_tick) < 100u
        && action_state.base_action_locked;
    if (robot_state.enabled && action_ready)
    {
        wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
        wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
        (void)RL_Torque_Compute(&leg_l, &leg_r,
            &rl_control.torque_param[rl_control.policy.selected_model],
            wheel_vel, action_state.a, &rl_control.torque_state, torque);
        torque[RL_TQ_R_THIGH] = 0.0f;
        torque[RL_TQ_R_SHANK] = 0.0f;
        torque[RL_TQ_L_WHEEL] = 0.0f;
        torque[RL_TQ_R_WHEEL] = 0.0f;
    }
    if (!robot_state.enabled || !torque_output_enabled)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dji_All_Stop();
        (void)Dm_Send_Zero();
        return;
    }
    dm_status = Dm_Send_Torque(torque);
    output_debug_dm_sent = (uint8_t)(dm_status == HAL_OK);
    output_debug_dji_sent = 0u;
}

/* 输出初始化 */
void output_task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/* 输出单周期 */
void output_task_body(void)
{
    Motor_Output_Update();
}
