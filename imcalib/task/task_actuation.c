/*
 * 每个基础执行拍读取反馈快照，依次做状态/模型准备、候选力矩计算和最终发送。
 * control_frame保存本拍输入与许可；dispatch决定本拍是否调用输出层，不表示发送成功。
 */
#include "robot_control.h"
#include "robot_state.h"
#include "machine_config.h"
#include "dm.h"
#include "tim.h"
#include "mono_ns.h"
#include "fallen_process.h"
#include "control_strategy.h"
#include "control_compute.h"
#include "control_output.h"

static control_frame_t control_frame;
volatile float output_dm_torque_nm[DM_MOTOR_NUM];
volatile float output_wheel_torque_nm[DJI_MOTOR_NUM];

/* 返回通信任务能否请求电机使能：遥控使能/在线、无故障且满足当前策略限制。
 * LQR保留进入倒地检测/自起的入口；RL失败禁止使能，实际力矩还需最终输出检查。 */
uint8_t Robot_Control_Enable_Allowed(void)
{
    uint8_t recovery;

    if (rc_command.mode.strategy == CTRL_STRATEGY_RL && rl_control.recovery.phase == RL_RECOVERY_FAILED) { return 0u; }
    recovery = (uint8_t)(fallen_process_control.lifecycle.enabled && fallen_process_control.lifecycle.recovery_enabled && rc_command.mode.strategy == CTRL_STRATEGY_LQR && rc_command.mode.engage);
    recovery |= (uint8_t)(rc_command.mode.strategy == CTRL_STRATEGY_RL && rc_command.mode.engage);
    return (uint8_t)(rc_command.mode.rc_enable && rc_command.online && ctrl_fault == FAULT_NONE && (rc_command.mode.strategy == CTRL_STRATEGY_LQR || !robot_state.fallen || recovery));
}

/* 复制最新反馈到本拍control_frame，计算基础、非倒地和自起三种出力许可。
 * 电机位置rad、速度rad/s、反馈力矩N·m；joint_ready要求四髋在线且报告已使能。 */
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
    control_frame.drive_permit = (uint8_t)(Robot_Control_Enable_Allowed() && control_frame.imu.online && control_frame.imu.pitch_world_valid && feedback.motor.valid && robot_state.motor_enabled && torque_output_enabled);
    if (control_frame.rc.mode.strategy == CTRL_STRATEGY_RL)
    {
        control_frame.drive_permit &= rl_control.policy.ready;
    }
    else
    {
        control_frame.drive_permit &= (uint8_t)(control_frame.left.output.valid && control_frame.right.output.valid);
    }
    control_frame.nonfallen_drive_permit = (uint8_t)(control_frame.drive_permit && !control_frame.fallen);
    control_frame.recovery_permit = (uint8_t)(control_frame.drive_permit && fallen_process_control.lifecycle.enabled && control_frame.rc.mode.strategy == CTRL_STRATEGY_LQR && control_frame.rc.mode.engage);
    taskEXIT_CRITICAL();
}

/* 启动TIM6中断，由定时器释放执行任务及分频策略任务的节拍；启动失败进入错误处理。 */
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

/* 完成一个执行拍：读快照→准备状态/模型→算候选力矩→按dispatch发送。
 * torque是待检查候选；pending_compensation随最终应用结果发布，尚未发送时不当作已执行。 */
void output_task_body(void)
{
    ctrl_strategy_t strategy; /* 本拍选中的控制策略。 */
    torque_output_t torque; /* 计算得到的四髋/双轮候选力矩，尚未经过最终发送检查。 */
    compensation_debug_t pending_compensation; /* 与候选配套的诊断，输出层按实际提交结果发布。 */
    uint8_t entered_fallen; /* 本拍刚进入公共FALLEN，触发自起历史复位。 */
    uint8_t dispatch; /* 1调用输出层，0本拍不发送；不是发送成功标志。 */
#if CONTROL_TIME_VOFA_ENABLE
    static uint32_t previous_start_us;
    static uint32_t previous_output_us;
    uint32_t start_us;
    uint32_t period_us;
    uint32_t run_us;

    /* 可选时序诊断：记录函数起点，统计的是调用间隔，不是CAN物理发送间隔。 */
    start_us = (uint32_t)(Mono_Ns_Get() / 1000u);
    period_us = control_time_debug.sequence != 0u ? start_us - previous_start_us : 0u;
    previous_start_us = start_us;
#endif

    /* 1. 固定本拍反馈和基础出力许可，后续计算使用同一份输入。 */
    Control_Frame_Read();
    /* 2. 先更新恢复状态/补偿缓存/公共选路，再生成候选力矩。 */
    strategy = Control_Strategy_Prepare(&control_frame, &entered_fallen);
    Control_Compute_Run(&control_frame, strategy, entered_fallen, &torque, &dispatch, &pending_compensation);
    /* 3. RL分频可跳过发送；进入输出层后仍会复查即时故障并决定正常力矩或发零。 */
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
