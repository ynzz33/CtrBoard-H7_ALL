/*
 * 只做公共低高度候选、原姿态异常观测和NORMAL/FALLEN/FLIGHT选路。
 * 初始触地优先；FALLEN由原动作报告完成，FLIGHT由原AIR退出判据结束。
 * 不读取LQR状态、不求解力矩，也不改原ground_recovery动作、AIR压缩或物理模型。
 */
#include "control_condition.h"
#include <math.h>
#include <string.h>

control_condition_param_t control_condition_param = {
    .body_contact_height = 0.15f,
    .confirm_time = 0.05f,
};
control_condition_t control_condition;

void Control_Condition_Reset(control_condition_t *state)
{
    memset(state, 0, sizeof(*state));
}

void Control_Condition_Observe(control_condition_t *state, const imu_state_t *imu,
                               const leg_state_t *left, const leg_state_t *right,
                               float height, uint8_t height_valid,
                               uint8_t zero_effort, uint8_t normal_running)
{
    state->hint_available = 0u;
    state->body_contact_candidate = 0u;
    state->body_contact_confirmed = 0u;
    state->prepare_required = 0u;
    Fallen_Detection_Update(&fallen_detection, imu, left, right);
    if (!fallen_detection.valid)
    {
        state->contact_elapsed = 0.0f;
        state->trigger_elapsed = 0.0f;
        return;
    }
    state->hint_available = (uint8_t)(zero_effort && height_valid);
    state->body_contact_candidate = (uint8_t)(state->hint_available && height <= control_condition_param.body_contact_height);
    state->contact_elapsed = state->body_contact_candidate ? fminf(state->contact_elapsed + MACHINE_LQR_DT, control_condition_param.confirm_time) : 0.0f;
    state->body_contact_confirmed = (uint8_t)(state->body_contact_candidate && state->contact_elapsed + 1.0e-6f >= control_condition_param.confirm_time);

    state->trigger_elapsed = (normal_running && fallen_detection.abnormal) ? fminf(state->trigger_elapsed + MACHINE_LQR_DT, fallen_detection_param.trigger_time) : 0.0f;
    state->prepare_required = (uint8_t)(normal_running && fallen_detection.abnormal && state->trigger_elapsed + 1.0e-6f >= fallen_detection_param.trigger_time);
}

static void Condition_Enter(control_condition_t *state, control_condition_type_t next)
{
    state->condition = next;
    state->started = 1u;
    state->valid = 1u;
    state->normal_ready = 0u;
    state->trigger_elapsed = 0.0f;
}

void Control_Condition_Recovery_Ready(control_condition_t *state, const air_detection_t *air)
{
    if (air_param.enabled && (!air->valid || !air->initialized))
    {
        state->valid = 0u;
        return;
    }
    Condition_Enter(state, (air_param.enabled && air->flight) ? CONTROL_FLIGHT : CONTROL_NORMAL);
}

void Control_Condition_Select(control_condition_t *state, const air_detection_t *air,
                              uint8_t request, uint8_t drive,
                              uint8_t fallen, uint8_t recovery_done)
{
    if (!request || !drive)
    {
        /* 短暂故障撤销本拍许可，保留恢复归属及原失败重试；退出投入才重新初判。 */
        if (!request)
        {
            state->started = 0u;
        }
        state->valid = 0u;
        state->normal_ready = 0u;
        state->trigger_elapsed = 0.0f;
        return;
    }
    state->valid = 0u;
    if (!fallen_detection.valid)
    {
        state->normal_ready = 0u;
        return;
    }
    if (state->started && state->condition == CONTROL_FALLEN)
    {
        if (recovery_done)
        {
            Control_Condition_Recovery_Ready(state, air);
        }
        else
        {
            state->valid = 1u;
        }
        return;
    }
    if (!state->started)
    {
        if (!state->hint_available || (state->body_contact_candidate && !state->body_contact_confirmed))
        {
            return;
        }
        if (state->body_contact_confirmed)
        {
            Condition_Enter(state, CONTROL_FALLEN);
            return;
        }
    }
    if (air_param.enabled && (!air->valid || !air->initialized))
    {
        state->normal_ready = 0u;
        return;
    }
    if (state->started && state->condition == CONTROL_FLIGHT)
    {
        /* AIR压缩落地未确认时，真正倒置不能持续锁在只摆腿的AIR处理。 */
        if (fallen_detection.upright < 0.0f)
        {
            Condition_Enter(state, CONTROL_FALLEN);
            return;
        }
        if (!air_param.enabled || !air->flight)
        {
            Condition_Enter(state, CONTROL_NORMAL);
        }
        else
        {
            state->valid = 1u;
        }
        return;
    }
    if (air_param.enabled && air->flight)
    {
        Condition_Enter(state, CONTROL_FLIGHT);
    }
    else if (!state->started)
    {
        Condition_Enter(state, (fallen || fallen_detection.inverted || fallen_detection.abnormal) ? CONTROL_FALLEN : CONTROL_NORMAL);
    }
    else if (state->normal_ready && (fallen_detection.inverted || state->prepare_required))
    {
        Condition_Enter(state, CONTROL_FALLEN);
    }
    else
    {
        state->valid = 1u;
    }
}
