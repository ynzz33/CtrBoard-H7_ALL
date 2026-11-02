/*
 * 将本拍IMU、双腿、反馈力矩和模型值交给检测器，再选择NORMAL/FALLEN/FLIGHT。
 * 超时后先撤力等待再重判；动作超时不是姿态已经倒地的证据。
 */
#include "control_detection.h"
#include "air_detection.h"
#include "control_condition.h"
#include "fallen_process.h"
#include "robot_state.h"
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

/* 组装AIR输入并独立更新倒地/AIR观测，再按原优先级选择公共控制状态。
 * 更新frame及robot_state的FALLEN归属，返回新进入FALLEN事件；撤力等待期间不重新出力。 */
uint8_t Control_Detection_Update(control_frame_t *frame)
{
    air_input_t input;
    uint8_t was_flight;
    control_condition_type_t previous;
    uint8_t request;
    uint8_t zero_effort;
    uint8_t normal_running;

    request = (uint8_t)(frame->rc.online && frame->rc.mode.rc_enable && frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage);
    if (!request)
    {
        memset(&control_postprocess, 0, sizeof(control_postprocess));
    }
    frame->landed = 0u;
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

    Control_Condition_Select(&control_condition, &air_detection, request, frame->drive_permit, (uint8_t)(fallen_process_control.lifecycle.phase == FALLEN_PROCESS_DONE));
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
    return (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_FALLEN && previous != CONTROL_FALLEN);
}
