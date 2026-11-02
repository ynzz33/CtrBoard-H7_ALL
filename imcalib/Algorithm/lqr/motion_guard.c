/*
 * 用实际车速、腿长/高度及增益比估计NORMAL偏航参考上限。
 * 收紧立即生效，解除按release_rate逐步放宽；只修改参考，不直接裁剪电机力矩。
 */
#include "motion_guard.h"
#include <math.h>
#include <string.h>
motion_guard_t motion_guard;
void Motion_Guard_Reset(motion_guard_t *state)
{
    memset(state, 0, sizeof(*state));
    state->reference_scale = 1.0f;
}
/* 从实际速度/高度和增益比求偏航参考上限；上限收紧立即跟随，放宽受release_rate限制。 */
uint8_t Motion_Guard_Update(motion_guard_t *state, const motion_guard_param_t *param,
                            const motion_guard_input_t *input)
{
    float previous = state->yaw_limit;
    uint8_t initialized = state->initialized;
    if (!param->enabled || !input->permit)
    {
        Motion_Guard_Reset(state);
        return 1u;
    }
    state->valid = 0u;
    state->length_guard = fmaxf(fmaxf(input->length[0], input->length[1]), input->length_target);
    state->height_reference = state->length_guard + input->height_offset;
    /* 按实际运动收紧转向，不因尚未实现的速度请求提前压低转向上限。 */
    state->speed_guard = fabsf(input->velocity);
    /* 所有实体输入由发布端验证；这里验证本模块产生的新预算和增益比。 */
    state->heading_ratio = input->heading_gain_diff / input->rate_gain_diff;
    state->lateral_budget = 9.81f * input->track * param->budget_fraction /
                            (2.0f * state->height_reference);
    state->yaw_limit_raw = fminf(input->yaw_max, state->lateral_budget / fmaxf(state->speed_guard, 0.05f));
    state->yaw_equivalent_raw = input->yaw_request + state->heading_ratio * input->heading_error;
    state->lateral_actual = fabsf(input->velocity * input->yaw_rate);
    if (!isfinite(state->heading_ratio) || !isfinite(state->height_reference) || state->height_reference <= 0.0f ||
        !isfinite(state->lateral_budget) || state->lateral_budget <= 0.0f ||
        !isfinite(state->yaw_equivalent_raw) || !isfinite(state->lateral_actual) ||
        !isfinite(state->yaw_limit_raw) || state->yaw_limit_raw < 0.0f)
    {
        Motion_Guard_Reset(state);
        state->reason = MOTION_GUARD_INVALID;
        return 0u;
    }
    /* 收紧立即生效，仅放宽上限限速；不是对所有遥控指令增加斜坡。 */
    state->yaw_limit = initialized ? fminf(state->yaw_limit_raw, previous + param->release_rate * input->dt) : state->yaw_limit_raw;
    state->yaw_equivalent = fminf(fmaxf(state->yaw_equivalent_raw, -state->yaw_limit), state->yaw_limit);
    state->limited = (uint8_t)(fabsf(state->yaw_equivalent_raw) > state->yaw_limit);
    state->reference_scale = state->limited ? state->yaw_limit / fabsf(state->yaw_equivalent_raw) : 1.0f;
    state->releasing = (uint8_t)(state->yaw_limit < state->yaw_limit_raw);
    state->reason = (uint8_t)(MOTION_GUARD_READY | (state->limited ? MOTION_GUARD_LIMITED : 0u) | (state->releasing ? MOTION_GUARD_RELEASING : 0u));
    state->initialized = 1u;
    state->valid = 1u;
    return 1u;
}
