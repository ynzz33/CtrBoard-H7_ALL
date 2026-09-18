#include "robot_control.h"
#include "dm.h"
#include "dji.h"
#include "dr16.h"
#include "tim.h"
#include "../Sysid/sysid_config.h"   /* 相对路径: 不依赖包含路径 */
#if SYSID_ENABLE
#include "../Sysid/sysid_mode.h"
#endif

/* 控制周期 (500Hz) */
#define OUTPUT_DT 0.002f

static uint8_t lqr_running;

/* 输出初始化 */
void output_task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/* 下发: 总输出关闭时只发零力矩, 其余计算照常 */
static void output_send(const torque_output_t *torque)
{
    if (!torque_output_enabled)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
        return;
    }
    output_debug_dm_sent = Dm_Send_Torque(torque->dm);
    output_debug_dji_sent = (uint8_t)Dji_Send_Wheel_Torque(
        torque->dji[DJI_MOTOR_WHEEL_LFT], torque->dji[DJI_MOTOR_WHEEL_RGT]);
}

/* LQR 平衡链路: 状态估计 → 状态反馈 → 腿部力控 → 下发 */
static void output_task_lqr(const dr16_t *remote)
{
    torque_output_t torque;
    float wheel_vel[2];

    wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];

    (void)LQR_Target_Update(&lqr_state, remote, OUTPUT_DT);

    if (LQR_State_Update(&lqr_state, &imu_state, &leg_l, &leg_r, wheel_vel, OUTPUT_DT))
    {
        LQR_Control_Update(&lqr_state);
        if (Leg_Balance_Compute(&leg_balance, &lqr_state, &leg_l, &leg_r,
                                OUTPUT_DT, &torque))
        {
            output_send(&torque);
            return;
        }
    }

    /* 任一步失败: 零力矩 */
    output_debug_dm_sent = 0u;
    output_debug_dji_sent = 0u;
    (void)Dm_Send_Zero();
    (void)Dji_All_Stop();
}

/* 输出单周期 */
void output_task_body(void)
{
    torque_output_t torque;
    float wheel_vel[2];
    dr16_t remote;

    remote = DR16_Snapshot();

    /* 左拨杆: 上位 = 手动遥操/RL, 中位 = LQR, 下位 = 失能; 左中 + 右中 = 测试模式 */
#if SYSID_ENABLE
    if (remote.online && remote.s1 == DR16_SW_MID && remote.s2 == DR16_SW_MID)
    {
        ctrl_strategy = CTRL_STRATEGY_SYSID;
    }
    else
#endif
    if (remote.online && remote.s1 == DR16_SW_MID)
    {
        ctrl_strategy = CTRL_STRATEGY_LQR;
    }
    else
    {
        ctrl_strategy = CTRL_STRATEGY_MANUAL;
    }

#if SYSID_ENABLE
    if (ctrl_strategy == CTRL_STRATEGY_SYSID)
    {
        lqr_running = 0u;
        Sysid_Mode_Run();
        return;
    }
#endif

    if (ctrl_strategy == CTRL_STRATEGY_LQR)
    {
        if (!(robot_state.motor_enabled && imu_state.online
              && leg_l.output.valid && leg_r.output.valid))
        {
            lqr_running = 0u;
        }
        else if (!lqr_running)
        {
            /* 使能边沿: 腿长锁到当前实测, 且必须在 K 表有效域内才投入 */
            lqr_running = LQR_Enable_Latch(&lqr_state, &leg_l, &leg_r);
        }

        if (lqr_running)
        {
            output_task_lqr(&remote);
        }
        else
        {
            output_debug_dm_sent = 0u;
            output_debug_dji_sent = 0u;
            (void)Dm_Send_Zero();
            (void)Dji_All_Stop();
        }
        return;
    }

    lqr_running = 0u;

    if (robot_state.rc_enable && robot_state.motor_enabled
        && leg_l.output.valid && leg_r.output.valid
        && action_state.base_action_locked)
    {
        wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
        wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
        (void)RL_Torque_Compute(&leg_l, &leg_r,
            &rl_control.torque_param[rl_control.policy.selected_model],
            wheel_vel, action_state.a, &rl_control.torque_state, &torque);

        output_send(&torque);
    }
    else
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
    }
}
