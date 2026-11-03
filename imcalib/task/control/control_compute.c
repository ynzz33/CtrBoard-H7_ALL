/*
 * 把执行拍输入转换为候选电机力矩：独立台阶、补偿测试、LQR平衡/自起/AIR或RL动作。
 * 管理控制器投入历史和RL分频；torque有效性与dispatch分别表示候选可用和本拍需要发送。
 */
#include "control_compute.h"
#include "control_strategy.h"
#include "robot_state.h"
#include "latency_state.h"
#include "output_capture.h"
#include "mono_ns.h"
#include "machine_config.h"
#include "fallen_process.h"
#include "slip.h"
#include "air_detection.h"
#include "control_condition.h"
#include "air_process.h"
#include "control_detection.h"
#include "stair_process.h"
#include <string.h>

static slip_state_t slip_control;
static uint8_t lqr_running;
static uint8_t rl_engaged;
static uint8_t rl_wait_ticks;
static uint8_t stair_owned_previous; /* 只记录进入/退出边沿，用于重建普通平衡历史 */
static rl_motor_map_state_t rl_motor_map; /* 用本拍IMU/腿/轮速组装LQR状态，接入AIR控制选择，再融合轮速和IMU加速度。
 * reset_velocity要求重新建立速度先验；任一步失败都撤销相应状态有效性。 */
static void Control_State_Update(control_frame_t *frame, uint8_t reset_velocity)
{
    if (!LQR_State_Update(&lqr_state, &frame->imu, &frame->left, &frame->right, frame->wheel_vel))
    {
        Slip_Reset(&slip_control);
        lqr_state.air_control = 0u;
        lqr_state.air_release = 0u;
        air_detection.applied = 0u;
        control_condition.valid = 0u;
        return;
    }
    Air_Process_Update(frame);
    if (reset_velocity || !slip_control.initialized)
    {
        Slip_Init(&slip_control, lqr_state.ds_raw, lqr_state.a_fwd);
    }
    else
    {
        (void)Slip_Update_Contact(&slip_control, lqr_state.ds_raw, lqr_state.a_fwd, MACHINE_LQR_DT,
                                  (uint8_t)(!lqr_state.air_control || !air_detection.flight));
    }
    if (!slip_control.valid)
    {
        lqr_state.valid = 0u;
        Slip_Reset(&slip_control);
        return;
    }
    LQR_Velocity_Apply(&lqr_state, slip_control.velocity, MACHINE_LQR_DT);
}

/* 检查反馈/状态许可，首次投入时锁存LQR基准并清腿控制历史，返回本拍是否投入。
 * 自起回正后的recovered只消费一次，用于重新初始化状态和速度历史。 */
static uint8_t LQR_Engage_Update(control_frame_t *frame)
{
    uint8_t ready;
    lqr_debug_t saved_debug;

    ready = (uint8_t)(lqr_state.valid && (control_condition.condition == CONTROL_FALLEN ? frame->nonfallen_drive_permit : (frame->drive_permit && control_condition.valid)));
    if (!ready)
    {
        lqr_running = 0u;
    }
    else if (!lqr_running)
    {
        if (fallen_process_control.lifecycle.recovered)
        {
            saved_debug = lqr_debug;
            LQR_Init(&lqr_state);
            lqr_debug = saved_debug;
            Control_State_Update(frame, 1u);
            fallen_process_control.lifecycle.recovered = 0u;
        }
        lqr_running = LQR_Enable_Latch(&lqr_state, &frame->rc);
        if (lqr_running)
        {
            Leg_Balance_Reset(&leg_balance);
        }
    }
    return lqr_running;
}

/* 退出LQR投入，清防翻和腿控制历史并撤销AIR控制应用标志；不重置AIR检测器。 */
static void LQR_Idle(void)
{
    Motion_Guard_Reset(&motion_guard);
    lqr_running = 0u;
    Leg_Balance_Reset(&leg_balance);
    lqr_state.air_control = 0u;
    lqr_state.air_release = 0u;
    air_detection.applied = 0u;
}

/* 台阶进入/退出各一次：清普通融合/位移/航向/腿控历史，不清任何自起预算。 */
static void Control_Stair_Balance_Reset(void)
{
    lqr_debug_t saved = lqr_debug;
    LQR_Idle();
    Slip_Reset(&slip_control);
    LQR_Init(&lqr_state);
    lqr_debug = saved;
    lqr_state.valid = 0u;
}

/* 台阶只有独立F/Tp候选；WAIT/RECHECK不运行普通估计或控制。 */
static void Compute_Stair(const control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    const stair_control_param_t control = {
        .length_pd = machine->lqr.pd.leg_len,
        .angle_pos = { .kp = fallen_process_param.angle_pos_kp, .kd = fallen_process_param.angle_pos_kd, .max_output = fallen_process_param.angle_speed_max },
        .angle_speed = { .kp = fallen_process_param.angle_speed_kp, .kd = fallen_process_param.angle_speed_kd, .max_output = machine->hip.dm_trq_clamp },
        .length_rate = fallen_process_param.length_rate,
        .angle_rate = fallen_process_param.angle_rate,
        .support_time = fallen_process_param.support_time,
        .support_angle_range = machine->recovery.fallen_support_angle_range,
        .leg_trim = machine->lqr.reference.leg_trim,
        .support_force = machine->lqr.roll_height.support_force,
    };
    const stair_process_input_t input = {
        .leg = {&frame->left, &frame->right},
        .control = &control,
        .compensation = &frame->compensation,
        .hip_limit = machine->hip.dm_trq_clamp,
        .now_ms = HAL_GetTick(),
        .permit = (uint8_t)(frame->drive_permit && frame->joint_ready && frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage),
    };
    Stair_Process_Update(&stair_process, &machine->stair.process, &input, MACHINE_LQR_DT, torque);
    if (torque->valid) { *debug = stair_process.compensation; }
}

/* 以四髋零基础力矩为输入，叠加选定弹簧/世界姿态重力补偿，轮候选保持零。
 * 非倒地出力许可不成立时保留无效候选，最终输出层负责发零。 */
static void Compute_Compensation(const control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    const float base_dm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (!frame->nonfallen_drive_permit)
    {
        return;
    }
    torque->valid = Leg_Compensation_Apply(&frame->compensation.spring, &frame->compensation.gravity, base_dm, machine->hip.dm_trq_clamp, torque->dm, debug);
}

/* 更新遥控目标并计算十态LQR摆矩/轮矩，再交leg_balance生成腿轴向力和四髋输出。
 * AIR时速度/偏航请求置零；只有全部计算成功才标记候选有效。 */
static void Compute_LQR(const control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    rc_command_t command;

    command = frame->rc;
    if (lqr_state.air_control)
    {
        command.velocity_target = 0.0f;
        command.yaw_rate_target = 0.0f;
    }
    if (!LQR_Target_Update(&lqr_state, &command))
    {
        return;
    }
    if (!lqr_state.valid)
    {
        return;
    }
    LQR_Control_Update(&lqr_state, (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_NORMAL));
    if (!lqr_state.gain_valid)
    {
        return;
    }
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &frame->left, &frame->right, &frame->compensation, &lqr_debug, (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_NORMAL), MACHINE_LQR_DT, torque);
    if (torque->valid)
    {
        *debug = leg_balance.compensation;
    }
}

/* 推进地面自起，按返回路由选择准备力矩、LQR平衡或停止。
 * 超时交任务层撤力等待；恢复交接需要AIR结果有效，不使用无效旧候选。 */
static void Compute_LQR_Fallen_Process(control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    uint8_t permit;
    uint8_t route;
    uint8_t was_done;

    permit = frame->recovery_permit;
    was_done = (uint8_t)(fallen_process_control.lifecycle.phase == FALLEN_PROCESS_DONE);
    if (was_done && permit && LQR_Engage_Update(frame))
    {
        Compute_LQR(frame, torque, debug);
    }
    if (fallen_process_control.lifecycle.retry < control_postprocess.recovery_retry)
    {
        fallen_process_control.lifecycle.retry = control_postprocess.recovery_retry;
    }
    route = Fallen_Process_Update(&fallen_process_control, &frame->imu, &frame->left,
                                   &frame->right, &frame->compensation, permit,
                                   MACHINE_LQR_DT, torque);
    if (fallen_process_control.lifecycle.phase == FALLEN_PROCESS_FAILED && fallen_process_control.lifecycle.fault == FALLEN_PROCESS_TIMEOUT)
    {
        Control_Postprocess_Request(1u);
    }
    if (route == FALLEN_PROCESS_ROUTE_BALANCE)
    {
        Control_Condition_Recovery_Ready(&control_condition);
        if (!control_condition.valid)
        {
            Torque_Output_Clear(torque);
            return;
        }
        /* 同拍从自起交给平衡，更新为普通姿态输入，避免沿用自起补偿。 */
        Control_Model_Prepare(frame);
        Air_Process_Update(frame);
        if (!was_done)
        {
            if (LQR_Engage_Update(frame))
            {
                Compute_LQR(frame, torque, debug);
            }
            else
            {
                Fallen_Process_Fail(&fallen_process_control, FALLEN_PROCESS_GATED);
            }
        }
        if (!torque->valid)
        {
            Fallen_Process_Fail(&fallen_process_control, FALLEN_PROCESS_BAD_INPUT);
            Torque_Output_Clear(torque);
            LQR_Idle();
            control_condition.condition = CONTROL_FALLEN;
            control_condition.normal_ready = 0u;
        }
    }
    else
    {
        LQR_Idle();
        if (route == FALLEN_PROCESS_ROUTE_PREPARE && torque->valid)
        {
            *debug = fallen_process_control.actuator.compensation;
        }
    }
}

/* 按RL控制分频读取/映射反馈、保存观测样本，再用最新有效action求电机力矩。
 * 非计算拍可令dispatch=0保持上一输出；退出/失败路径仍按代码要求清零处理。 */
static void Compute_RL(control_frame_t *frame, torque_output_t *torque, uint8_t *dispatch)
{
    uint8_t tick;

    tick = (uint8_t)(rl_wait_ticks == 0u);
    lqr_running = 0u;
    rl_wait_ticks = tick ? MACHINE_RL_CTRL_DIV - 1u : rl_wait_ticks - 1u;
    if (!frame->rc.mode.engage) { rl_motor_map.initialized = 0u; }
    if (tick)
    {
#if MACHINE_LATENCY_CAPTURE_ENABLE
        latency_record_t trace={0};
        trace.kind=LATENCY_SAMPLE;
        trace.begin_ns=frame->trace_snapshot_ns;
#endif
        (void)RL_Motor_Map(&machine->rl, 2.0f * machine->hip.dm_pos_max, frame->motor_pos, frame->motor_vel, frame->wheel_vel, frame->motor_valid, &rl_motor_map, &frame->rl_motor);
        RL_Observation_Sample(&rl_control.observation, frame->imu.gyro_rad_s, frame->imu.quat, &frame->rl_motor, frame->imu.online);
#if MACHINE_LATENCY_CAPTURE_ENABLE
        taskENTER_CRITICAL();
        trace.end_ns=Mono_Ns_Get();
        trace.correlation=++trace_sample.sequence;
        trace_sample.cached_ns=trace.end_ns;
        trace.reset_count=action_state.reset_count;
        for(uint8_t i=0u;i<8u;i++) { trace_sample_rx[i]=frame->trace_rx_ns[i]; trace.value[i]=Latency_Age_Us(trace.end_ns,frame->trace_rx_ns[i]); }
        taskEXIT_CRITICAL();
        Latency_Capture_Push(&trace);
#endif
    }
    rl_engaged = (uint8_t)(frame->drive_permit && frame->rc.mode.engage && rl_control.recovery.phase != RL_RECOVERY_IDLE && rl_control.recovery.phase != RL_RECOVERY_FAILED);
    if (rl_engaged && rl_control.recovery.phase != RL_RECOVERY_HOLD && action_state.rl_ready && frame->rl_motor.valid)
    {
        if (tick)
        {

#if MACHINE_LATENCY_CAPTURE_ENABLE
            static uint32_t last_policy,last_reset;
            static uint8_t seen;
            latency_record_t trace={0};
            trace.kind=LATENCY_PD;
            trace.value[0]=trace_policy_sequence;
            trace.reset_count=action_state.reset_count;
            trace.correlation=trace_dispatch_sequence+1u;
            trace.begin_ns=Mono_Ns_Get();
#endif
            (void)RL_Torque_Compute(&frame->rl_motor, &machine->rl, action_state.a, &rl_control.torque_state, torque);
#if MACHINE_LATENCY_CAPTURE_ENABLE
            trace.end_ns=Mono_Ns_Get(); trace.flags=torque->valid;
            trace.value[1]=Latency_Age_Us(trace.begin_ns,trace_policy_published_ns);
            for(uint8_t i=0u;i<6u;i++) { trace.value[i+2u]=Latency_Age_Us(trace.begin_ns,frame->trace_rx_ns[i+1u]); }
            if(torque->valid && (!seen || last_policy!=trace_policy_sequence || last_reset!=action_state.reset_count)) { trace.flags|=2u;seen=1u;last_policy=trace_policy_sequence;last_reset=action_state.reset_count; }
            Latency_Capture_Push(&trace);
#endif
        }
        else
        {
            *dispatch = 0u;
        }
    }
}

/* 先更新控制反馈并清零候选，再按strategy计算LQR/自起/AIR、RL或补偿测试输出。
 * torque.valid表示计算成功；dispatch可因RL分频为0，此时本拍不调用发送层。 */
void Control_Compute_Run(control_frame_t *frame, ctrl_strategy_t strategy,
                         uint8_t entered_fallen, torque_output_t *torque,
                         uint8_t *dispatch, compensation_debug_t *debug)
{
    uint8_t observer_permit;

    if (Stair_Process_Owns_Output(&stair_process))
    {
        if (!stair_owned_previous) { Control_Stair_Balance_Reset(); }
        stair_owned_previous = 1u;
        Torque_Output_Clear(torque);
        memset(debug, 0, sizeof(*debug));
        ctrl_strategy = strategy;
        rl_engaged = 0u;
        rl_wait_ticks = 0u;
        rl_motor_map.initialized = 0u;
        *dispatch = 1u;
        Compute_Stair(frame, torque, debug);
        return;
    }
    if (stair_owned_previous)
    {
        Control_Stair_Balance_Reset();
        stair_owned_previous = 0u;
    }
    leg_balance.turn_ff = 0.0f;
    if (entered_fallen)
    {
        Fallen_Process_Reset(&fallen_process_control);
        fallen_process_control.lifecycle.retry = control_postprocess.recovery_retry;
    }
    /* 保持原顺序：进入FALLEN先清动作，再构建本拍LQR状态。 */
    Control_State_Update(frame, 0u);
    /* 校验上一拍F确实通过最终许可并成功提交；HAL_OK不等于电机实际力已实现。 */
    observer_permit = (uint8_t)(leg_balance.roll_height.valid && leg_balance.cmd.valid && output_debug_dm_sent && output_debug_dji_sent && !leg_balance.compensation.saturated && air_detection.valid && air_detection.initialized && !air_detection.flight && air_detection.leg[0].force > air_param.force_on && air_detection.leg[1].force > air_param.force_on);
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (leg_balance.cmd.dm[i] != output_dm_torque_nm[i])
        {
            observer_permit = 0u;
        }
    }
    Roll_Height_Begin_Tick(&leg_balance.roll_height, (uint8_t)(strategy == CTRL_STRATEGY_LQR && frame->drive_permit && frame->rc.mode.engage && lqr_state.valid && control_condition.valid && control_condition.condition == CONTROL_NORMAL && !Control_Postprocess_Inhibited()), observer_permit);
    Torque_Output_Clear(torque);
    memset(debug, 0, sizeof(*debug));
    ctrl_strategy = strategy;
    if (!fallen_process_control.lifecycle.enabled || strategy == CTRL_STRATEGY_COMPENSATION || (frame->rc.online && !(frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage)))
    {
        Fallen_Process_Reset(&fallen_process_control);
    }
    else if (strategy != CTRL_STRATEGY_LQR && fallen_process_control.lifecycle.phase != FALLEN_PROCESS_IDLE)
    {
        Fallen_Process_Fail(&fallen_process_control, FALLEN_PROCESS_GATED);
    }
    rl_engaged = 0u;
    *dispatch = 1u;
    if (strategy != CTRL_STRATEGY_RL)
    {
        rl_wait_ticks = 0u;
        rl_motor_map.initialized = 0u;
    }

    switch (strategy)
    {
        case CTRL_STRATEGY_COMPENSATION:
            LQR_Idle();
            Compute_Compensation(frame, torque, debug);
            break;

        case CTRL_STRATEGY_LQR:
            if (Control_Postprocess_Inhibited())
            {
                LQR_Idle();
                Slip_Reset(&slip_control);
                lqr_state.pos = 0.0f;
                lqr_state.pos_armed = 0u;
                lqr_state.yaw_target = lqr_state.x[LQR_X_PHI];
            }
            else if (!frame->drive_permit || !frame->rc.mode.engage)
            {
                if (control_condition.condition == CONTROL_FALLEN && fallen_process_control.lifecycle.enabled && fallen_process_control.lifecycle.phase != FALLEN_PROCESS_IDLE && frame->rc.mode.engage)
                {
                    Compute_LQR_Fallen_Process(frame, torque, debug);
                }
                else
                {
                    LQR_Idle();
                }
            }
            else if (!control_condition.valid)
            {
                /* 初判/重确认期间保持默认无效零候选，不能清公共观察器计时。 */
            }
            else if (control_condition.condition == CONTROL_FALLEN)
            {
                if (fallen_process_control.lifecycle.enabled)
                {
                    Compute_LQR_Fallen_Process(frame, torque, debug);
                }
            }
            else if (control_condition.condition == CONTROL_FLIGHT && !air_param.control_enabled)
            {
                /* 关闭AIR处理不能隐式改选NORMAL；保持默认无效零候选。 */
            }
            else if (LQR_Engage_Update(frame))
            {
                Compute_LQR(frame, torque, debug);
            }
            else
            {
                LQR_Idle();
            }
            Control_Postprocess_Flight_Update((uint8_t)(frame->drive_permit && frame->rc.mode.engage && control_condition.valid && control_condition.condition == CONTROL_FLIGHT && air_detection.applied && torque->valid));
            if (Control_Postprocess_Inhibited())
            {
                Torque_Output_Clear(torque);
                memset(debug, 0, sizeof(*debug));
                LQR_Idle();
            }
            control_condition.normal_ready = (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_NORMAL && torque->valid);
            break;

        case CTRL_STRATEGY_RL:
            Compute_RL(frame, torque, dispatch);
            break;

        case CTRL_STRATEGY_DISABLE:
        default:
            LQR_Idle();
            break;
    }
    if (strategy != CTRL_STRATEGY_LQR || !torque->valid || control_condition.condition != CONTROL_NORMAL || Control_Postprocess_Inhibited())
    {
        Motion_Guard_Reset(&motion_guard);
        Roll_Height_Reset(&leg_balance.roll_height);
    }
}

uint8_t Control_Compute_LQR_Engaged(void)
{
    return lqr_running;
}

uint8_t Control_Compute_RL_Engaged(void)
{
    return rl_engaged;
}
