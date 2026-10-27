/* NORMAL四状态反馈；速度低通只在1ms执行拍推进，不发送电机命令。 */
#include "roll_height_balance.h"
#include "machine_config.h"
#include "mono_ns.h"
#include <math.h>
#include <string.h>

uint8_t Roll_Height_Available(void)
{
    return (uint8_t)(roll_height_model.machine_id == MACHINE_DEFAULT);
}

void Roll_Height_Reset(roll_height_balance_t *state)
{
    memset(state, 0, sizeof(*state));
    Lowpass_Init(&state->velocity_filter[0], machine->lqr.lpf_alpha[2]);
    Lowpass_Init(&state->velocity_filter[1], machine->lqr.lpf_alpha[3]);
}

void Roll_Height_Begin_Tick(roll_height_balance_t *state, uint8_t permit, uint8_t observer_permit)
{
    if (!permit || !Roll_Height_Available())
    {
        Roll_Height_Reset(state);
        return;
    }
    state->permit = 1u;
    state->sampled = 0u;
    state->observer.permit = observer_permit;
    state->observer.updated = 0u;
    state->observer.limited = 0u;
}

/* 输入F保持补偿前坐标；仅使用上一拍确认提交且未限矩的请求，不把FN估计当模型输入。 */
static uint8_t Roll_Height_Observe(roll_height_balance_t *state, const float measurement[RH_NX])
{
    roll_height_observer_t *observer = &state->observer;
    float predicted[RH_NX];
    float innovation[RH_NX];
    float next[RH_NE];
    float common;
    float difference;
    float bounded_common;
    float bounded_difference;

    if (!observer->permit || !observer->seeded)
    {
        memcpy(observer->state, measurement, sizeof(observer->state));
        observer->seeded = observer->permit;
        return 1u; /* 失去支撑/发送失败/限矩时仅重对齐状态，保持偏置、不学习。 */
    }
    for (uint8_t i = 0u; i < RH_NX; i++)
    {
        predicted[i] = roll_height_model.f[i];
        for (uint8_t j = 0u; j < RH_NX; j++)
        {
            predicted[i] += roll_height_model.A[i][j] * observer->state[j];
        }
        for (uint8_t j = 0u; j < RH_NU; j++)
        {
            predicted[i] += roll_height_model.B[i][j] * (state->force[j] + observer->bias[j]);
        }
        innovation[i] = measurement[i] - predicted[i];
    }
    for (uint8_t i = 0u; i < RH_NE; i++)
    {
        next[i] = i < RH_NX ? predicted[i] : observer->bias[i - RH_NX];
        for (uint8_t j = 0u; j < RH_NX; j++)
        {
            next[i] += roll_height_model.L[i][j] * innovation[j];
        }
        if (!isfinite(next[i]))
        {
            return 0u;
        }
    }
    common = 0.5f * (next[RH_NX] + next[RH_NX + 1u]);
    difference = next[RH_NX] - next[RH_NX + 1u];
    bounded_common = clampf(common, -machine->lqr.roll_height_bias_common_max, machine->lqr.roll_height_bias_common_max);
    bounded_difference = clampf(difference, -machine->lqr.roll_height_bias_diff_max, machine->lqr.roll_height_bias_diff_max);
    memcpy(observer->state, next, sizeof(observer->state));
    observer->bias[0] = bounded_common + 0.5f * bounded_difference;
    observer->bias[1] = bounded_common - 0.5f * bounded_difference;
    observer->limited = (uint8_t)(bounded_common != common || bounded_difference != difference);
    observer->updated = 1u;
    return 1u;
}

uint8_t Roll_Height_Update(roll_height_balance_t *state, const lqr_state_t *lqr,
                          const leg_state_t *left, const leg_state_t *right,
                          const float length_target[RH_NU], float turn_ff)
{
    const leg_state_t *legs[2] = {left, right};
    float projection = 0.0f;
    float velocity = 0.0f;
    float theta;
    float omega;
    float length;
    float value;
    float measurement[RH_NX];
    uint32_t started;

    if (!state->permit)
    {
        return 0u;
    }
    if (state->sampled)
    {
        return state->valid;
    }
    state->sampled = 1u;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        theta = lqr->x[i == 0u ? LQR_X_THL : LQR_X_THR];
        omega = lqr->x[i == 0u ? LQR_X_DTHL : LQR_X_DTHR];
        length = legs[i]->output.virtual_leg_length;
        projection += length * cosf(theta);
        velocity += legs[i]->output.d_virtual_leg_length * cosf(theta) - length * sinf(theta) * omega;
    }
    state->x[0] = lqr->roll;
    state->x[2] = 0.5f * cosf(lqr->roll) * projection;
    state->height_rate_raw = 0.5f * (cosf(lqr->roll) * velocity - sinf(lqr->roll) * lqr->roll_rate * projection);
    state->reference[0] = 0.0f;
    state->reference[1] = 0.0f;
    /* 复用原腿长斜坡；与原PD一致，只阻尼实际速度，不微分给定。 */
    state->reference[2] = 0.5f * (length_target[0] * cosf(lqr->target[LQR_X_THL]) + length_target[1] * cosf(lqr->target[LQR_X_THR]));
    state->reference[3] = 0.0f;
    state->height_goal = 0.5f * (lqr->leg_len_tgt[0] * cosf(lqr->target[LQR_X_THL]) + lqr->leg_len_tgt[1] * cosf(lqr->target[LQR_X_THR]));
    if (!isfinite(state->x[2]) || !isfinite(state->height_rate_raw) || !isfinite(state->reference[2]))
    {
        Roll_Height_Reset(state);
        state->lqr.result = RH_LQR_BAD_RESULT;
        return 0u;
    }
    state->x[1] = Lowpass_Update(&state->velocity_filter[0], lqr->roll_rate);
    state->x[3] = Lowpass_Update(&state->velocity_filter[1], state->height_rate_raw);
    started = (uint32_t)(Mono_Ns_Get() / 1000u);
    state->valid = 0u;
    measurement[0] = state->x[0];
    measurement[1] = lqr->roll_rate;
    measurement[2] = state->x[2];
    measurement[3] = state->height_rate_raw;
    /* 观测用原始速度，避免将控制低通的延迟误认为物理扰动。 */
    if (!Roll_Height_Observe(state, measurement))
    {
        Roll_Height_Reset(state);
        state->lqr.result = RH_LQR_BAD_RESULT;
        return 0u;
    }
    for (uint8_t i = 0u; i < RH_NU; i++)
    {
        state->lqr.equilibrium[i] = machine->lqr.support_force[i] + (i == 0u ? -turn_ff : turn_ff) - state->observer.bias[i];
        value = state->lqr.equilibrium[i];
        for (uint8_t j = 0u; j < RH_NX; j++)
        {
            state->lqr.feedback[i][j] = -roll_height_model.K[i][j] * (state->x[j] - state->reference[j]);
            value += state->lqr.feedback[i][j];
        }
        if (!isfinite(value))
        {
            Roll_Height_Reset(state);
            state->lqr.result = RH_LQR_BAD_RESULT;
            return 0u;
        }
        state->force[i] = value; /* 允许负F主动压缩；最终电机限矩由原输出口执行。 */
    }
    state->elapsed_us = (uint32_t)(Mono_Ns_Get() / 1000u) - started;
    if (state->elapsed_us > state->max_elapsed_us)
    {
        state->max_elapsed_us = state->elapsed_us;
    }
    state->update_count++;
    state->lqr.result = RH_LQR_OK;
    state->valid = 1u;
    return 1u;
}
