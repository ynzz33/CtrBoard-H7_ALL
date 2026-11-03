/* 先记录双腿姿态/角加速度，再在本机确认的范围内发布单次撞阶事件。 */
#include "stair_detection.h"
#include "math_utils.h"
#include <string.h>

stair_detection_t stair_detection;

void Stair_Detection_Reset(stair_detection_t *state)
{
    memset(state, 0, sizeof(*state));
}

/* 只在确实结束候选时锁存原因；空闲拍不能覆盖已经发生的退出。 */
static void Stair_Detection_Clear(stair_detection_t *state, uint8_t reason)
{
    if (state->impact_pending) { state->clear_reason = reason; }
    state->impact_pending = 0u;
    state->impact_elapsed = 0.0f;
    state->elapsed = 0.0f;
}

void Stair_Detection_Disarm(stair_detection_t *state)
{
    Stair_Detection_Clear(state, STAIR_CLEAR_DISARM);
    state->armed = 0u;
    state->seeded = 0u;
    state->trigger = 0u;
    state->impact_high = 0u;
}

/* 双腿高腿及实测偏航限制开窗，不限制轮速；捕获后按请求/许可和原期限保持。 */
static void Stair_Detection_Confirm(stair_detection_t *state, const stair_detection_param_t *param, const stair_detection_input_t *input, float dt)
{
    uint8_t pose;
    uint8_t acceleration;
    uint8_t hold;
    uint8_t eligible;
    uint8_t reason;
    state->trigger = 0u;
    state->flags = param->confirmed ? STAIR_DETECT_CONFIRMED : 0u;
    pose = (uint8_t)(fabsf(state->deviation[0]) >= param->angle_min && fabsf(state->deviation[1]) >= param->angle_min);
    acceleration = (uint8_t)(fmaxf(fabsf(state->acceleration[0]), fabsf(state->acceleration[1])) >= param->acceleration_min);
    hold = (uint8_t)(input->permit && input->velocity_request >= param->request_min && fabsf(input->yaw_request) <= param->yaw_request_max);
    eligible = (uint8_t)(hold && input->length[0] >= param->length_min && input->length[1] >= param->length_min && fabsf(input->yaw_rate) <= param->yaw_rate_max);
    if (input->permit && input->velocity_request == 0.0f && !pose && !acceleration)
    {
        state->armed = 1u;
    }
    if (!state->armed || !hold)
    {
        reason = !input->permit ? STAIR_CLEAR_PERMIT : (!state->armed ? STAIR_CLEAR_DISARM : (input->velocity_request < param->request_min ? STAIR_CLEAR_REQUEST : STAIR_CLEAR_YAW));
        Stair_Detection_Clear(state, reason);
    }
    if (state->impact_pending)
    {
        state->impact_elapsed += dt;
        if (state->impact_elapsed > param->impact_window + 1.0e-6f)
        {
            Stair_Detection_Clear(state, STAIR_CLEAR_TIMEOUT);
        }
    }
    if (eligible && state->armed && acceleration && !state->impact_high && !state->impact_pending)
    {
        state->impact_pending = 1u;
        state->impact_elapsed = 0.0f;
        state->elapsed = 0.0f;
    }
    if (state->impact_pending)
    {
        state->elapsed = pose ? state->elapsed + dt : 0.0f;
        if (pose && state->elapsed + 1.0e-6f >= param->confirm_time)
        {
            state->trigger = 1u;
            state->armed = 0u;
            Stair_Detection_Clear(state, STAIR_CLEAR_CONFIRMED);
            state->events++;
        }
    }
    state->impact_high = acceleration;
    state->flags |= (uint16_t)((eligible ? STAIR_DETECT_ELIGIBLE : 0u) | (state->armed ? STAIR_DETECT_ARMED : 0u) | (pose ? STAIR_DETECT_POSE : 0u) | (acceleration ? STAIR_DETECT_ACCELERATION : 0u) | (state->impact_pending ? STAIR_DETECT_CANDIDATE : 0u) | (state->trigger ? STAIR_DETECT_TRIGGER : 0u));
    state->flags |= (uint16_t)((uint16_t)state->clear_reason << STAIR_CLEAR_SHIFT);
}

/* 先生成一次特征，再推进确认；失去NORMAL许可时丢弃窗口和差分历史。 */
void Stair_Detection_Update(stair_detection_t *state, const stair_detection_param_t *param, const stair_detection_input_t *input, float dt)
{
    float alpha = dt / (param->acceleration_tau + dt);

    state->trigger = 0u;
    state->flags = param->confirmed ? STAIR_DETECT_CONFIRMED : 0u;
    if (!input->permit)
    {
        Stair_Detection_Clear(state, STAIR_CLEAR_PERMIT);
        state->seeded = 0u;
        state->impact_high = 0u;
        state->flags |= (uint16_t)((uint16_t)state->clear_reason << STAIR_CLEAR_SHIFT);
        return;
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        float derivative = state->seeded ? (input->rate[i] - state->previous_rate[i]) / dt : 0.0f;
        state->acceleration[i] = state->seeded ? state->acceleration[i] + alpha * (derivative - state->acceleration[i]) : 0.0f;
        state->previous_rate[i] = input->rate[i];
        state->deviation[i] = Angle_Wrap_Pi(input->angle[i] - input->reference[i]);
        if (!isfinite(state->acceleration[i]) || !isfinite(state->deviation[i]))
        {
            Stair_Detection_Clear(state, STAIR_CLEAR_INVALID);
            state->seeded = 0u;
            state->impact_high = 0u;
            state->flags |= (uint16_t)((uint16_t)state->clear_reason << STAIR_CLEAR_SHIFT);
            return;
        }
    }
    state->seeded = 1u;
    Stair_Detection_Confirm(state, param, input, dt);
}
