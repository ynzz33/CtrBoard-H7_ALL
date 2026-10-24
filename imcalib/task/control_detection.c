/*
 * 文件职责：组装检测输入并按原顺序调用AIR观测、触地观测和三态选择。
 * 输入为执行拍快照及现有公共上下文；输出为判断、真实落地沿和进入FALLEN事件。
 * 数学算法归air_detection/control_condition；后处理超时在此统一等待和重判，不求解力矩。
 */
#include "control_detection.h"
#include "air_detection.h"
#include "control_condition.h"
#include "ground_recovery.h"
#include <string.h>

control_postprocess_t control_postprocess;

void Control_Postprocess_Request(uint8_t recovery_timeout)
{
    if (control_postprocess.phase != CONTROL_POSTPROCESS_RUN)
    {
        return;
    }
    /* 任何撤力重判都保留此前已经消耗的地面恢复预算。 */
    if (ground_recovery_control.lifecycle.retry > control_postprocess.recovery_retry)
    {
        control_postprocess.recovery_retry = ground_recovery_control.lifecycle.retry;
    }
    if (recovery_timeout)
    {
        if (control_postprocess.recovery_retry >= ground_recovery_param.recovery_retry_max)
        {
            control_postprocess.exhausted = 1u;
        }
        else
        {
            control_postprocess.recovery_retry++;
        }
        ground_recovery_control.lifecycle.retry = control_postprocess.recovery_retry;
    }
    control_postprocess.phase = CONTROL_POSTPROCESS_WITHDRAW_WAIT;
    control_postprocess.wait_start_ms = HAL_GetTick();
    control_postprocess.flight_timing = 0u;
    control_condition.valid = 0u;
    control_condition.normal_ready = 0u;
}

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

uint8_t Control_Postprocess_Inhibited(void)
{
    return (uint8_t)(control_postprocess.phase != CONTROL_POSTPROCESS_RUN);
}

uint8_t Control_Detection_Update(control_frame_t *frame)
{
    air_input_t input;
    uint8_t was_flight;
    control_condition_type_t previous;
    uint8_t request;
    uint8_t zero_effort;
    uint8_t normal_running;
    uint8_t air_inhibit;

    request = (uint8_t)(frame->rc.online && frame->rc.mode.rc_enable && frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage);
    if (!request)
    {
        memset(&control_postprocess, 0, sizeof(control_postprocess));
    }
    frame->landed = 0u;
    if (control_postprocess.phase == CONTROL_POSTPROCESS_WITHDRAW_WAIT)
    {
        control_condition.valid = 0u;
        if (!control_postprocess.exhausted && (float)(uint32_t)(HAL_GetTick() - control_postprocess.wait_start_ms) * 0.001f + 1.0e-6f >= ground_recovery_param.retry_wait)
        {
            /* 本拍保持撤力；下一拍才开始新的初判，不把历史复位当作落地。 */
            Air_Detection_Reset(&air_detection);
            air_inhibit = control_condition.air_inhibit;
            Control_Condition_Reset(&control_condition);
            control_condition.air_inhibit = air_inhibit;
            Ground_Recovery_Reset(&ground_recovery_control);
            ground_recovery_control.lifecycle.retry = control_postprocess.recovery_retry;
            control_postprocess.phase = CONTROL_POSTPROCESS_RECHECK;
        }
        return 0u;
    }
    memset(&input, 0, sizeof(input));
    input.imu = &frame->imu;
    input.leg[0] = &frame->left;
    input.leg[1] = &frame->right;
    input.compensation = &frame->compensation;
    input.joint_ready = frame->joint_ready;
    for (uint8_t side = 0u; side < 2u; side++)
    {
        for (uint8_t i = 0u; i < 2u; i++)
        {
            input.torque[side][i] = frame->joint_torque[side * 2u + i];
        }
    }
    normal_running = (uint8_t)(request && frame->drive && control_condition.valid && control_condition.condition == CONTROL_NORMAL && control_condition.normal_ready);
    if (Control_Condition_Air_Allowed(&control_condition, normal_running))
    {
        was_flight = air_detection.flight;
        Air_Detection_Update(&air_detection, &input);
        frame->landed = (uint8_t)(was_flight && !air_detection.flight && air_detection.valid);
    }
    else
    {
        Air_Detection_Reset(&air_detection);
    }

    previous = control_condition.condition;
    zero_effort = (uint8_t)(!request || !frame->drive || !control_condition.started);
    Control_Condition_Observe(&control_condition, &frame->imu, &frame->left, &frame->right, frame->height, frame->height_valid, zero_effort, normal_running);
    Control_Condition_Select(&control_condition, &air_detection, request, frame->drive, frame->fallen, (uint8_t)(ground_recovery_control.lifecycle.phase == GROUND_RECOVERY_DONE));
    if (control_condition.valid && control_condition.condition == CONTROL_FALLEN && previous != CONTROL_FALLEN)
    {
        Air_Detection_Reset(&air_detection);
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
