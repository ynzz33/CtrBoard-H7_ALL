#include "robot_control.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"

/* 输出初始化 */
void output_task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/* 输出单周期 */
void output_task_body(void)
{
    torque_output_t torque;
    float wheel_vel[2];

    if (robot_state.rc_enable && robot_state.motor_enabled
        && leg_l.output.valid && leg_r.output.valid
        && action_state.base_action_locked)
    {
        wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
        wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
        (void)RL_Torque_Compute(&leg_l, &leg_r,
            &rl_control.torque_param[rl_control.policy.selected_model],
            wheel_vel, action_state.a, &rl_control.torque_state, &torque);

        output_debug_dm_sent = Dm_Send_Torque(torque.dm);
        output_debug_dji_sent = (uint8_t)Dji_Send_Wheel_Torque(
            torque.dji[DJI_MOTOR_WHEEL_LFT], torque.dji[DJI_MOTOR_WHEEL_RGT]);
    }
    else
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
    }
}
