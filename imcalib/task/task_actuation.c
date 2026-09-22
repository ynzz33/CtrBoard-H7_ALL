#include "robot_control.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"
#if 0   /* 测试模式暂不接入仲裁 (sysid_mode.c 自带 Dm_Send_*, 接入时改走 output_dispatch) */
#include "../Sysid/sysid_mode.h"
#endif

/*
 * 输出任务三层结构 (作者 2026-09-22 定: 解算只算, 分发唯一):
 *   1 估计层  LQR_State_Update()            每拍必算, 不看挡位
 *   2 求解层  solve_*()                      按策略算 torque_output_t, 只写 torque 不碰驱动
 *   3 分发层  output_dispatch()              全文件唯一的 Dm_Send / Dji_Send 调用点
 * 改输出行为 (总开关 / 限幅 / 斜坡 / 极性) 只动 output_dispatch(); 改控制律只动 solve_*()
 */

static uint8_t lqr_running;     /* 已投入 */
static uint8_t lqr_manual;      /* 手动腿测 */

/* 输出初始化 */
void output_task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/* LQR / 手动腿测是否已投入 (供 VOFA) */
uint8_t output_task_lqr_engaged(void)
{
    return lqr_running;
}

/* ================= 3 分发层: 唯一下发点 ================= */
/* valid=0 或总输出关 → 零力矩; 否则原样下发 (各路限幅已在控制器内做, 极性在驱动边界做) */
static void output_dispatch(const torque_output_t *torque)
{
    if (!torque->valid || !torque_output_enabled)
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
        // (void)Dm_Send_Zero();
        // (void)Dji_All_Stop();
}

/* ================= 模式与投入 ================= */
/* 左拨杆 → 模式: 上 = RL, 中 = LQR, 下 / 离线 = 失能 (右拨杆中位 = 投入, 在 output_task_body 判) */
static ctrl_strategy_t strategy_from_remote(const rc_command_t *cmd)
{
    if (!cmd->online)
    {
        return CTRL_STRATEGY_DISABLE;
    }
    switch (cmd->s1)
    {
    case DR16_SW_MID:
        return CTRL_STRATEGY_LQR;
    case DR16_SW_UP:
        return CTRL_STRATEGY_MANUAL;
    default:
        return CTRL_STRATEGY_DISABLE;
    }
}

/* LQR / 手动腿测投入锁存: 前提齐全时使能沿锁腿长投入, 前提丢失退出; 返回是否已投入 */
static uint8_t lqr_engage_update(uint8_t manual)
{
    uint8_t ready;

    if (manual != lqr_manual)
    {
        lqr_running = 0u;   /* 模式切换 */
        lqr_manual = manual;
    }

    ready = (uint8_t)(robot_state.motor_enabled
                      && (manual || imu_state.online)
                      && leg_l.output.valid && leg_r.output.valid);
    if (!ready)
    {
        lqr_running = 0u;
    }
    else if (!lqr_running)
    {
        /* 使能沿: 锁腿长目标 (不查实测腿长, 同 Leg2) */
        lqr_running = LQR_Enable_Latch(&lqr_state, &leg_l, &leg_r, manual);
        if (lqr_running)
        {
            Leg_Balance_Reset(&leg_balance);
        }
    }
    return lqr_running;
}

/* LQR 退出 / 未投入: 清观测值 */
static void lqr_idle(void)
{
    lqr_running = 0u;
    Leg_Balance_Reset(&leg_balance);
}

/* ================= 2 求解层: 只写 torque, 不下发 ================= */
/* LQR 平衡: 目标 → 状态反馈 → 腿部力控 */
static void solve_lqr(torque_output_t *torque)
{
    (void)LQR_Target_Update(&lqr_state, &rc_command, CTRL_DT, 0u);
    if (!lqr_state.valid)
    {
        return;
    }
    LQR_Control_Update(&lqr_state);
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &leg_l, &leg_r,
                                        CTRL_DT, torque);
}

/* 手动腿测: 摇杆 → 腿长/摆角目标 → PID → 力域映射 (轮零) */
static void solve_lqr_manual(torque_output_t *torque)
{
    (void)LQR_Target_Update(&lqr_state, &rc_command, CTRL_DT, 1u);
    torque->valid = Leg_Balance_Manual(&leg_balance, &lqr_state, &leg_l, &leg_r,
                                       CTRL_DT, torque);
}

/* RL: 动作 → 力矩; 前提: 遥控使能 + 电机使能 + 两腿有效 + 基准动作已锁 */
static void solve_rl(const float wheel_vel[2], torque_output_t *torque)
{
    if (!(robot_state.rc_enable && robot_state.motor_enabled
          && leg_l.output.valid && leg_r.output.valid
          && action_state.base_action_locked))
    {
        return;
    }
    (void)RL_Torque_Compute(&leg_l, &leg_r,
        &rl_control.torque_param[rl_control.policy.selected_model],
        wheel_vel, action_state.a, &rl_control.torque_state, torque);
    torque->valid = 1u;
}

/* ================= 主体: 估计 → 求解 → 分发 ================= */
void output_task_body(void)
{
    float wheel_vel[2];
    ctrl_strategy_t strategy;
    torque_output_t torque;

    wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];

    /* 1 估计: 每拍必算 (同 RL 观测) */
    (void)LQR_State_Update(&lqr_state, &imu_state, &leg_l, &leg_r, wheel_vel, CTRL_DT);

    /* 2 求解: torque 默认全零 valid=0, 只有走通的分支才置 valid */
    Torque_Output_Clear(&torque);
    strategy = strategy_from_remote(&rc_command);
    ctrl_strategy = strategy;

    switch (strategy)
    {
#if 0   /* 测试模式暂不接入仲裁 */
    case CTRL_STRATEGY_SYSID:
        lqr_running = 0u;
        Sysid_Mode_Run();
        return;                     /* sysid 自己下发, 不走 dispatch */
#endif

    case CTRL_STRATEGY_LQR:
    case CTRL_STRATEGY_LQR_MANUAL:
    {
        uint8_t manual = (uint8_t)(strategy == CTRL_STRATEGY_LQR_MANUAL);
        if (rc_command.s2 == DR16_SW_MID && lqr_engage_update(manual))
        {
            if (manual)
            {
                solve_lqr_manual(&torque);
            }
            else
            {
                solve_lqr(&torque);
            }
        }
        else
        {
            lqr_idle();
        }
        break;
    }

    case CTRL_STRATEGY_MANUAL:
        lqr_running = 0u;
        if (rc_command.s2 == DR16_SW_MID)
        {
            solve_rl(wheel_vel, &torque);
        }
        break;

    case CTRL_STRATEGY_DISABLE:
    default:
        lqr_idle();
        break;
    }

    /* 3 分发: 唯一出口 */
    output_dispatch(&torque);
}
