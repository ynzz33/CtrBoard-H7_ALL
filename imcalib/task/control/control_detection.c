/*
 * 台阶接管时暂停公共判定；其他执行拍选择NORMAL/FALLEN/FLIGHT后检测台阶入口。
 * 超时后先撤力等待再重判；动作超时不是姿态已经倒地的证据。
 */
#include "control_detection.h"
#include "air_detection.h"
#include "control_condition.h"
#include "fallen_process.h"
#include "robot_state.h"
#include "stair_detection.h"
#include "stair_process.h"
#include "math_utils.h"
#include <string.h>

control_postprocess_t control_postprocess;

/* 锁存撤力等待请求并保存重试预算；recovery_timeout仅地面恢复超时时为1。
 * 不直接把姿态标为倒地，等待结束后重新判断实际状态。 */
void Control_Postprocess_Request(uint8_t recovery_timeout)
{
    if (control_postprocess.phase != CONTROL_POSTPROCESS_RUN)
    {
        return;
    }
    /* 任何撤力重判都保留此前已经消耗的地面恢复预算。 */
    if (fallen_process_control.lifecycle.retry > control_postprocess.recovery_retry)
    {
        control_postprocess.recovery_retry = fallen_process_control.lifecycle.retry;
    }
    if (recovery_timeout)
    {
        if (control_postprocess.recovery_retry >= fallen_process_param.recovery_retry_max)
        {
            control_postprocess.exhausted = 1u;
        }
        else
        {
            control_postprocess.recovery_retry++;
        }
        fallen_process_control.lifecycle.retry = control_postprocess.recovery_retry;
    }
    control_postprocess.phase = CONTROL_POSTPROCESS_WITHDRAW_WAIT;
    control_postprocess.wait_start_ms = HAL_GetTick();
    control_postprocess.flight_timing = 0u;
    control_condition.valid = 0u;
    control_condition.normal_ready = 0u;
}

/* 仅对实际应用且有效的FLIGHT控制计时，连续5s触发撤力等待；中断条件立即停止计时。 */
void Control_Postprocess_Flight_Update(uint8_t active)
{
    if (!active || control_postprocess.phase != CONTROL_POSTPROCESS_RUN)
    {
        control_postprocess.flight_timing = 0u;
        return;
    }
    if (!control_postprocess.flight_timing)
    {
        control_postprocess.flight_timing = 1u;
        control_postprocess.flight_start_ms = HAL_GetTick();
    }
    else if ((uint32_t)(HAL_GetTick() - control_postprocess.flight_start_ms) >= 5000u)
    {
        Control_Postprocess_Request(0u);
    }
}

/* 返回是否处于撤力等待/重判阶段，调用方据此清候选力矩。 */
uint8_t Control_Postprocess_Inhibited(void)
{
    return (uint8_t)(control_postprocess.phase != CONTROL_POSTPROCESS_RUN);
}

/* 只清公共检测历史；不读取或修改任何动作的等待、重试预算。 */
static void Control_Detection_Recheck_Begin(control_frame_t *frame)
{
    Air_Detection_Reset(&air_detection);
    Fallen_Detection_Reset(&fallen_detection);
    Control_Condition_Reset(&control_condition);
    frame->landed = 0u;
}

/* 台阶期间旧NORMAL只表示来源，撤销实时有效性及普通平衡就绪。 */
static void Control_Detection_Stair_Pause(control_frame_t *frame)
{
    control_condition.valid = 0u;
    control_condition.normal_ready = 0u;
    air_detection.valid = 0u;
    air_detection.applied = 0u;
    fallen_detection.valid = 0u;
    frame->landed = 0u;
    stair_detection.trigger = 0u;
    stair_detection.flags = (uint16_t)(STAIR_DETECT_PAUSED | (machine->stair.detection.confirmed ? STAIR_DETECT_CONFIRMED : 0u));
}

/* 公共选路后检测；沿用本机世界腿角定义，角度/角速度同时扣除机身俯仰运动。 */
static void Control_Detection_Stair_Update(control_frame_t *frame, uint8_t request)
{
    stair_detection_input_t input;
    stair_process_input_t process_input;
    if (machine->stair.mode == STAIR_MODE_OFF || MACHINE_DEFAULT != MACHINE_ID_BIG_WHEELLEG)
    {
        Stair_Detection_Reset(&stair_detection);
        return;
    }
    memset(&input, 0, sizeof(input));
    input.angle[0] = Angle_Wrap_Pi(frame->left.output.virtual_leg_angle - frame->imu.pitch_world);
    input.angle[1] = Angle_Wrap_Pi(frame->right.output.virtual_leg_angle - frame->imu.pitch_world);
    input.rate[0] = frame->left.output.d_virtual_leg_angle - frame->imu.gyro_rad_s[1];
    input.rate[1] = frame->right.output.d_virtual_leg_angle - frame->imu.gyro_rad_s[1];
    input.reference[0] = machine->lqr.reference.leg_trim[0];
    input.reference[1] = machine->lqr.reference.leg_trim[1];
    input.permit = (uint8_t)(request && frame->drive_permit && frame->joint_ready && control_condition.valid && control_condition.condition == CONTROL_NORMAL && control_condition.normal_ready && output_task_lqr_engaged() && !Control_Postprocess_Inhibited());
    Stair_Detection_Update(&stair_detection, &machine->stair.detection, &input, MACHINE_LQR_DT);
    if (machine->stair.mode != STAIR_MODE_AUTO || !machine->stair.detection.confirmed || !stair_detection.trigger) { return; }
    memset(&process_input, 0, sizeof(process_input));
    process_input.leg[0] = &frame->left;
    process_input.leg[1] = &frame->right;
    process_input.permit = input.permit;
    process_input.now_ms = HAL_GetTick();
    if (Stair_Process_Begin(&stair_process, &machine->stair.process, &process_input))
    {
        Stair_Detection_Disarm(&stair_detection);
        Control_Detection_Stair_Pause(frame);
    }
}

/* 台阶期间只管理退出；其他执行拍按原优先级选路，再判断NORMAL内台阶触发。
 * 返回新进入FALLEN事件；台阶/公共撤力等待均不能隐式恢复普通出力。 */
uint8_t Control_Detection_Update(control_frame_t *frame)
{
    air_input_t input;
    uint8_t was_flight;
    control_condition_type_t previous;
    uint8_t request;
    uint8_t zero_effort;
    uint8_t normal_running;
    uint8_t stair_rechecking = 0u;

    request = (uint8_t)(frame->rc.online && frame->rc.mode.rc_enable && frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage);
    if (!request)
    {
        memset(&control_postprocess, 0, sizeof(control_postprocess));
    }
    frame->landed = 0u;
    if (Stair_Process_Owns_Output(&stair_process))
    {
        if (!request || machine->stair.mode != STAIR_MODE_AUTO)
        {
            /* 退出投入即作废动作；再次投入从新反馈初判，不续跑旧阶段。 */
            Stair_Process_Abort(&stair_process, STAIR_EXIT_CANCELLED, HAL_GetTick());
            Control_Detection_Recheck_Begin(frame);
            Stair_Process_Reset(&stair_process);
            Stair_Detection_Disarm(&stair_detection);
        }
        else
        {
            if (!frame->drive_permit || !frame->joint_ready)
            {
                Stair_Process_Abort(&stair_process, STAIR_EXIT_INPUT, HAL_GetTick());
            }
            if (Stair_Process_Advance_Exit(&stair_process, &machine->stair.process, HAL_GetTick()))
            {
                Control_Detection_Recheck_Begin(frame);
            }
            if (stair_process.phase != STAIR_PROCESS_RECHECK)
            {
                Control_Detection_Stair_Pause(frame);
                return 0u;
            }
            stair_rechecking = 1u;
        }
    }
    if (control_postprocess.phase == CONTROL_POSTPROCESS_WITHDRAW_WAIT)
    {
        control_condition.valid = 0u;
        if (!control_postprocess.exhausted && (float)(uint32_t)(HAL_GetTick() - control_postprocess.wait_start_ms) * 0.001f + 1.0e-6f >= fallen_process_param.retry_wait)
        {
            /* 本拍保持撤力；下一拍才开始新的初判，不把历史复位当作落地。 */
            Air_Detection_Reset(&air_detection);
            Control_Condition_Reset(&control_condition);
            Fallen_Process_Reset(&fallen_process_control);
            fallen_process_control.lifecycle.retry = control_postprocess.recovery_retry;
            control_postprocess.phase = CONTROL_POSTPROCESS_RECHECK;
        }
        return 0u;
    }
    memset(&input, 0, sizeof(input));
    input.imu = &frame->imu;
    input.leg[0] = &frame->left;
    input.leg[1] = &frame->right;
    input.joint_ready = frame->joint_ready;
    for (uint8_t side = 0u; side < 2u; side++)
    {
        for (uint8_t i = 0u; i < 2u; i++)
        {
            input.torque[side][i] = frame->joint_torque[side * 2u + i];
        }
    }
    normal_running = (uint8_t)(request && frame->drive_permit && control_condition.valid && control_condition.condition == CONTROL_NORMAL && control_condition.normal_ready);
    /* 先观察FALLEN；AIR是否初始化不能阻止该观测或后续FALLEN选路。 */
    previous = control_condition.condition;
    zero_effort = (uint8_t)(!request || !frame->drive_permit || !control_condition.started);
    Control_Condition_Observe(&control_condition, &frame->imu, &frame->left, &frame->right, frame->height, frame->height_valid, zero_effort, normal_running);
    was_flight = air_detection.flight;
    Air_Detection_Update(&air_detection, &input);
    frame->landed = (uint8_t)(was_flight && !air_detection.flight && air_detection.valid);

    if (stair_rechecking && stair_process.result == STAIR_RESULT_DONE && request && frame->drive_permit && fallen_detection.valid && !fallen_detection.inverted)
    {
        /* 普通摆正已完成：复用自起交接，不把0.15m短腿重新当初始贴地自起。 */
        Control_Condition_Recovery_Ready(&control_condition);
    }
    else
    {
        Control_Condition_Select(&control_condition, &air_detection, request, frame->drive_permit, (uint8_t)(fallen_process_control.lifecycle.phase == FALLEN_PROCESS_DONE));
    }
    frame->fallen = (uint8_t)(control_condition.started && control_condition.condition == CONTROL_FALLEN);
    robot_state.fallen = frame->fallen;
    frame->nonfallen_drive_permit = (uint8_t)(frame->drive_permit && !frame->fallen);
    if (control_condition.valid && control_condition.condition == CONTROL_FALLEN && previous != CONTROL_FALLEN)
    {
        frame->landed = 0u;
    }
    if (control_postprocess.phase == CONTROL_POSTPROCESS_RECHECK)
    {
        /* 重判过程不发布落地事件，只有有效的新判断才能重新开放处理。 */
        frame->landed = 0u;
        if (control_condition.valid)
        {
            control_postprocess.phase = CONTROL_POSTPROCESS_RUN;
        }
    }
    if (stair_rechecking)
    {
        frame->landed = 0u;
        if (control_condition.valid)
        {
            Stair_Process_Reset(&stair_process);
            Stair_Detection_Disarm(&stair_detection);
        }
    }
    else
    {
        Control_Detection_Stair_Update(frame, request);
    }
    return (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_FALLEN && previous != CONTROL_FALLEN);
}
