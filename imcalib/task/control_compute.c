/*
 * 文件职责：按已确认策略运行原LQR、自起、AIR、RL或补偿计算。
 * 策略/检测、模型准备和最终输出分别由相邻执行层负责。
 * 原控制公式和限幅保持；后处理超时统一撤力等待、重新初判。
 */
#include "control_compute.h"
#include "robot_state.h"
#include "machine_config.h"
#include "ground_recovery.h"
#include "slip.h"
#include "air_detection.h"
#include "control_condition.h"
#include "control_air_response.h"
#include "control_detection.h"
#include <string.h>

static slip_state_t slip_control;
static uint8_t lqr_running;
static uint8_t rl_engaged;
static uint8_t rl_wait_ticks;

/* 公共检测已在策略层更新；此处构建LQR状态、接入原AIR响应和速度融合。 */
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
    Control_Air_Response_Update(frame);
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

static uint8_t LQR_Engage_Update(control_frame_t *frame)
{
    uint8_t ready;
    lqr_debug_t saved_debug;

    ready = (uint8_t)(lqr_state.valid && (control_condition.condition == CONTROL_FALLEN ? frame->normal : (frame->drive && control_condition.valid)));
    if (!ready)
    {
        lqr_running = 0u;
    }
    else if (!lqr_running)
    {
        if (ground_recovery_control.lifecycle.recovered)
        {
            saved_debug = lqr_debug;
            LQR_Init(&lqr_state);
            lqr_debug = saved_debug;
            Control_State_Update(frame, 1u);
            ground_recovery_control.lifecycle.recovered = 0u;
        }
        lqr_running = LQR_Enable_Latch(&lqr_state, &frame->rc);
        if (lqr_running)
        {
            Leg_Balance_Reset(&leg_balance);
        }
    }
    return lqr_running;
}

/* LQR退出只清私有历史；公共检测的历史由检测模块维护。 */
static void LQR_Idle(void)
{
    lqr_running = 0u;
    Leg_Balance_Reset(&leg_balance);
    lqr_state.air_control = 0u;
    lqr_state.air_release = 0u;
    air_detection.applied = 0u;
}

static void Compute_Compensation(const control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    const float base_dm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (!frame->normal)
    {
        return;
    }
    torque->valid = Leg_Compensation_Apply(&frame->compensation.spring, &frame->compensation.gravity_world, base_dm, machine->dm_trq_clamp, torque->dm, debug);
}

static void Compute_LQR(const control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    rc_command_t command;

    command = frame->rc;
    if (lqr_state.air_control)
    {
        command.vel = 0.0f;
        command.yaw = 0.0f;
    }
    if (!LQR_Target_Update(&lqr_state, &command))
    {
        return;
    }
    if (!lqr_state.valid)
    {
        return;
    }
    LQR_Control_Update(&lqr_state);
    if (!lqr_state.gain_valid)
    {
        return;
    }
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &frame->left, &frame->right, &frame->compensation, &lqr_debug, MACHINE_LQR_DT, torque);
    if (torque->valid)
    {
        *debug = leg_balance.compensation;
    }
}

static void Compute_LQR_Ground_Recovery(control_frame_t *frame, torque_output_t *torque, compensation_debug_t *debug)
{
    uint8_t permit;
    uint8_t route;
    uint8_t was_done;

    permit = frame->recovery;
    was_done = (uint8_t)(ground_recovery_control.lifecycle.phase == GROUND_RECOVERY_DONE);
    if (was_done && permit && LQR_Engage_Update(frame))
    {
        Compute_LQR(frame, torque, debug);
    }
    if (ground_recovery_control.lifecycle.retry < control_postprocess.recovery_retry)
    {
        ground_recovery_control.lifecycle.retry = control_postprocess.recovery_retry;
    }
    route = Ground_Recovery_Update(&ground_recovery_control, &frame->imu, &frame->left,
                                   &frame->right, &frame->compensation, permit,
                                   MACHINE_LQR_DT, torque);
    if (ground_recovery_control.lifecycle.phase == GROUND_RECOVERY_FAILED && ground_recovery_control.lifecycle.fault == GROUND_RECOVERY_TIMEOUT)
    {
        Control_Postprocess_Request(1u);
    }
    if (route == GROUND_RECOVERY_ROUTE_BALANCE)
    {
        Control_Condition_Recovery_Ready(&control_condition, &air_detection);
        if (!control_condition.valid)
        {
            Torque_Output_Clear(torque);
            return;
        }
        Control_Air_Response_Update(frame);
        if (!was_done)
        {
            if (LQR_Engage_Update(frame))
            {
                Compute_LQR(frame, torque, debug);
            }
            else
            {
                Ground_Recovery_Fail(&ground_recovery_control, GROUND_RECOVERY_GATED);
            }
        }
        if (!torque->valid)
        {
            Ground_Recovery_Fail(&ground_recovery_control, GROUND_RECOVERY_BAD_INPUT);
            Torque_Output_Clear(torque);
            LQR_Idle();
            control_condition.condition = CONTROL_FALLEN;
            control_condition.normal_ready = 0u;
        }
    }
    else
    {
        LQR_Idle();
        if (route == GROUND_RECOVERY_ROUTE_PREPARE && torque->valid)
        {
            *debug = ground_recovery_control.actuator.compensation;
        }
    }
}

static void Compute_RL(control_frame_t *frame, torque_output_t *torque, uint8_t *dispatch)
{
    uint8_t tick;

    tick = (uint8_t)(rl_wait_ticks == 0u);
    lqr_running = 0u;
    rl_wait_ticks = tick ? MACHINE_RL_CTRL_DIV - 1u : rl_wait_ticks - 1u;
    if (tick)
    {
        (void)RL_Motor_Map(&machine->rl, frame->motor_pos, frame->motor_vel, frame->wheel_vel, frame->motor_valid, &frame->rl_motor);
        RL_Observation_Sample(&rl_control.observation, frame->imu.gyro_rad_s, frame->imu.quat, &frame->rl_motor, frame->imu.online);
    }
    rl_engaged = (uint8_t)(frame->drive && frame->rc.mode.engage && rl_control.recovery.phase != RL_RECOVERY_IDLE && rl_control.recovery.phase != RL_RECOVERY_FAILED);
    if (rl_engaged && rl_control.recovery.phase != RL_RECOVERY_HOLD && action_state.rl_ready && frame->rl_motor.valid)
    {
        if (tick)
        {
            (void)RL_Torque_Compute(&frame->rl_motor, &machine->rl, action_state.a, &rl_control.torque_state, torque);
        }
        else
        {
            *dispatch = 0u;
        }
    }
}

void Control_Compute_Run(control_frame_t *frame, ctrl_strategy_t strategy,
                         uint8_t entered_fallen, torque_output_t *torque,
                         uint8_t *dispatch, compensation_debug_t *debug)
{
    if (entered_fallen)
    {
        Ground_Recovery_Reset(&ground_recovery_control);
        ground_recovery_control.lifecycle.retry = control_postprocess.recovery_retry;
    }
    /* 保持原顺序：进入FALLEN先清动作，再构建本拍LQR状态。 */
    Control_State_Update(frame, 0u);
    Torque_Output_Clear(torque);
    memset(debug, 0, sizeof(*debug));
    ctrl_strategy = strategy;
    if (!ground_recovery_control.lifecycle.enabled || strategy == CTRL_STRATEGY_COMPENSATION || (frame->rc.online && !(frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage)))
    {
        Ground_Recovery_Reset(&ground_recovery_control);
    }
    else if (strategy != CTRL_STRATEGY_LQR && ground_recovery_control.lifecycle.phase != GROUND_RECOVERY_IDLE)
    {
        Ground_Recovery_Fail(&ground_recovery_control, GROUND_RECOVERY_GATED);
    }
    rl_engaged = 0u;
    *dispatch = 1u;
    if (strategy != CTRL_STRATEGY_RL)
    {
        rl_wait_ticks = 0u;
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
                lqr_state.yaw_tgt = lqr_state.x[LQR_X_PHI];
            }
            else if (!frame->drive || !frame->rc.mode.engage)
            {
                if (control_condition.condition == CONTROL_FALLEN && ground_recovery_control.lifecycle.enabled && ground_recovery_control.lifecycle.phase != GROUND_RECOVERY_IDLE && frame->rc.mode.engage)
                {
                    Compute_LQR_Ground_Recovery(frame, torque, debug);
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
                if (ground_recovery_control.lifecycle.enabled)
                {
                    Compute_LQR_Ground_Recovery(frame, torque, debug);
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
            Control_Postprocess_Flight_Update((uint8_t)(frame->drive && frame->rc.mode.engage && control_condition.valid && control_condition.condition == CONTROL_FLIGHT && air_detection.applied && torque->valid));
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
}

uint8_t Control_Compute_LQR_Engaged(void)
{
    return lqr_running;
}

uint8_t Control_Compute_RL_Engaged(void)
{
    return rl_engaged;
}
