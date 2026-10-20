/*
 * 文件职责：1kHz执行拍的固定编排。
 * 输入为反馈快照、遥控和策略任务发布的RL动作；输出经唯一输出层发送到电机。
 * 关键边界：本文件只安排“策略准备→控制运算→最终输出”，不保存控制公式。
 */
#include "robot_control.h"
#include "robot_state.h"
#include "machine_config.h"
#include "dm.h"
#include "tim.h"
#include "mono_ns.h"
#include "ground_recovery.h"
#include "control_strategy.h"
#include "control_compute.h"
#include "control_output.h"

static control_frame_t control_frame;
volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];

uint8_t Robot_Control_Enable_Allowed(void)
{
    uint8_t recovery;

    if (rc_command.mode.strategy == CTRL_STRATEGY_RL && rl_control.recovery.phase == RL_RECOVERY_FAILED) { return 0u; }
    recovery = (uint8_t)(ground_recovery_control.lifecycle.enabled && ground_recovery_control.lifecycle.recovery_enabled && rc_command.mode.strategy == CTRL_STRATEGY_LQR && rc_command.mode.engage);
    recovery |= (uint8_t)(machine->rl.configured && rc_command.mode.strategy == CTRL_STRATEGY_RL && rc_command.mode.engage);
    return (uint8_t)(rc_command.mode.rc_enable && rc_command.online && ctrl_fault == FAULT_NONE && (!robot_state.fallen || recovery));
}

/* 读取一次已发布反馈，形成整个执行拍唯一使用的快照。 */
static void Control_Frame_Read(void)
{
    static robot_feedback_t feedback;

    (void)Robot_Feedback_Read(&feedback);
    control_frame.landed = 0u;

    taskENTER_CRITICAL();
    control_frame.imu = feedback.imu;
    control_frame.left = feedback.left;
    control_frame.right = feedback.right;
    control_frame.rc = feedback.rc;
    control_frame.height = feedback.height;
    control_frame.height_valid = feedback.height_valid;
    control_frame.fallen = robot_state.fallen;
    control_frame.wheel_vel[0] = feedback.motor.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    control_frame.wheel_vel[1] = feedback.motor.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    control_frame.joint_ready = 1u;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        control_frame.motor_pos[i] = feedback.motor.dm.pos_continuous_rad[i];
        control_frame.motor_vel[i] = feedback.motor.dm.vel_rad_s[i];
        control_frame.joint_torque[i] = feedback.motor.dm.trq_nm[i];
        control_frame.joint_ready &= (uint8_t)(feedback.motor.dm.online[i] && Dm_Is_Enabled(i));
    }
    control_frame.motor_valid = feedback.motor.valid;
    control_frame.drive = (uint8_t)(Robot_Control_Enable_Allowed() && control_frame.imu.online && control_frame.imu.pitch_world_valid && feedback.motor.valid && robot_state.motor_enabled && torque_output_enabled);
    if (control_frame.rc.mode.strategy == CTRL_STRATEGY_RL)
    {
        control_frame.drive &= (uint8_t)(machine->rl.configured && rl_control.policy.ready);
    }
    else
    {
        control_frame.drive &= (uint8_t)(control_frame.left.output.valid && control_frame.right.output.valid);
    }
    control_frame.normal = (uint8_t)(control_frame.drive && !control_frame.fallen);
    control_frame.recovery = (uint8_t)(control_frame.drive && ground_recovery_control.lifecycle.enabled && control_frame.rc.mode.strategy == CTRL_STRATEGY_LQR && control_frame.rc.mode.engage);
    taskEXIT_CRITICAL();
}

void output_task_init(void)
{
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t output_task_lqr_engaged(void)
{
    return Control_Compute_LQR_Engaged();
}

uint8_t output_task_rl_engaged(void)
{
    return Control_Compute_RL_Engaged();
}

/* 同一actuationTask内同步执行，候选力矩不会跨拍排队。 */
void output_task_body(void)
{
    ctrl_strategy_t strategy;
    torque_output_t torque;
    compensation_debug_t pending_compensation;
    uint8_t entered_fallen;
    uint8_t dispatch;
#if CONTROL_TIME_VOFA_ENABLE
    static uint32_t previous_start_us;
    static uint32_t previous_output_us;
    uint32_t start_us;
    uint32_t period_us;
    uint32_t run_us;

    start_us = (uint32_t)(Mono_Ns_Get() / 1000u);
    period_us = control_time_debug.sequence != 0u ? start_us - previous_start_us : 0u;
    previous_start_us = start_us;
#endif

    Control_Frame_Read();
    strategy = Control_Strategy_Prepare(&control_frame, Control_Compute_LQR_Engaged(), &entered_fallen);
    Control_Compute_Run(&control_frame, strategy, entered_fallen, &torque, &dispatch, &pending_compensation);
    Control_Output_Publish_Air_Debug();
    if (dispatch)
    {
        Control_Output_Apply(&control_frame, strategy, &torque, &pending_compensation);
#if CONTROL_TIME_VOFA_ENABLE
        control_time_debug.output_period_us = control_time_debug.output_sequence != 0u ? start_us - previous_output_us : 0u;
        previous_output_us = start_us;
        control_time_debug.output_sequence++;
#endif
    }
#if CONTROL_TIME_VOFA_ENABLE
    run_us = (uint32_t)(Mono_Ns_Get() / 1000u) - start_us;
    if (control_time_debug.sequence != 0u)
    {
        if (control_time_debug.min_period_us == 0u || period_us < control_time_debug.min_period_us)
        {
            control_time_debug.min_period_us = period_us;
        }
        if (period_us > control_time_debug.max_period_us)
        {
            control_time_debug.max_period_us = period_us;
        }
    }
    control_time_debug.period_us = period_us;
    control_time_debug.run_us = run_us;
    control_time_debug.sequence++;
#endif
}
