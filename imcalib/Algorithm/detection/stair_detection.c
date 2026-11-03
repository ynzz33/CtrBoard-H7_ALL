/* 世界腿角与负向角加速度联合连续确认；不推进任何动作历史。 */
#include "stair_detection.h"
#include "math_utils.h"
#include <string.h>

stair_detection_t stair_detection;

void Stair_Detection_Reset(stair_detection_t *state)
{
    memset(state, 0, sizeof(*state));
    state->armed = 1u;
}

/* 一次持续满足只报告一次；联合条件解除后重新允许检测，不要求摇杆回中。 */
void Stair_Detection_Disarm(stair_detection_t *state)
{
    state->armed = 0u;
    state->seeded = 0u;
    state->trigger = 0u;
    state->elapsed = 0.0f;
}

void Stair_Detection_Update(stair_detection_t *state, const stair_detection_param_t *param, const stair_detection_input_t *input, float dt)
{
    uint8_t pose;
    uint8_t acceleration;
    uint8_t matched;
    state->trigger = 0u;
    state->flags = param->confirmed ? STAIR_DETECT_CONFIRMED : 0u;
    if (!input->permit)
    {
        state->seeded = 0u;
        state->elapsed = 0.0f;
        return;
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        float derivative = state->seeded ? (input->rate[i] - state->previous_rate[i]) / dt : 0.0f;
        if (!state->seeded) { LPF_Init(&state->acceleration[i], dt / (param->acceleration_tau + dt)); }
        LPF_Update(&state->acceleration[i], derivative);
        state->previous_rate[i] = input->rate[i];
        state->deviation[i] = Angle_Wrap_Pi(input->angle[i] - input->reference[i]);
        if (!isfinite(state->acceleration[i].out) || !isfinite(state->deviation[i]))
        {
            state->seeded = 0u;
            state->elapsed = 0.0f;
            return;
        }
    }
    state->seeded = 1u;
    pose = (uint8_t)(state->deviation[0] < param->angle_threshold && state->deviation[1] < param->angle_threshold);
    acceleration = (uint8_t)(state->acceleration[0].out <= param->acceleration_threshold || state->acceleration[1].out <= param->acceleration_threshold);
    matched = (uint8_t)(pose && acceleration);
    if (!matched) { state->armed = 1u; }
    state->elapsed = matched && state->armed ? state->elapsed + dt : 0.0f;
    if (matched && state->armed && state->elapsed + 1.0e-6f >= param->confirm_time)
    {
        state->trigger = 1u;
        state->armed = 0u;
        state->events++;
    }
    state->flags |= (uint16_t)(STAIR_DETECT_ELIGIBLE | (state->armed ? STAIR_DETECT_ARMED : 0u) | (pose ? STAIR_DETECT_POSE : 0u) | (acceleration ? STAIR_DETECT_ACCELERATION : 0u) | (matched ? STAIR_DETECT_CANDIDATE : 0u) | (state->trigger ? STAIR_DETECT_TRIGGER : 0u));
}
