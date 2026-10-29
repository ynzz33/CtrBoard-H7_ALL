/* NORMAL四状态反馈；速度低通只在1ms执行拍推进，不发送电机命令。 */
#include "roll_height_balance.h"
#include "machine_config.h"
#include "mono_ns.h"
#include <math.h>
#include <string.h>

/* 单腿轴向力下限N：允许负值补偿，但不允许无限往下拉把腿卸掉。 */
#define ROLL_HEIGHT_FORCE_MIN (-10.0f)

uint8_t Roll_Height_Available(void)
{
    return (uint8_t)(roll_height_model.machine_id == MACHINE_DEFAULT);
}

void Roll_Height_Reset(roll_height_balance_t *state)
{
    memset(state, 0, sizeof(*state));
    LPF_Init(&state->lpf_omg_roll, machine->lqr.lpf_alpha[2]);
    LPF_Init(&state->lpf_height_rate, machine->lqr.lpf_alpha[3]);
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

/* 几何投影、成对位置/速度参考和控制速度低通；不推进观测器。 */
static uint8_t Roll_Height_State_Update(roll_height_balance_t *state, const lqr_state_t *lqr,
                                      const leg_state_t *left, const leg_state_t *right,
                                      const float length_ramp_target[RH_NU], const float length_rate_target[RH_NU])
{
    const leg_state_t *legs[2] = {left, right};
    float projection = 0.0f;
    float velocity = 0.0f;
    float theta;
    float omega;
    float length;

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
    state->target[0] = 0.0f;
    state->target[1] = 0.0f;
    /* 固定腿角目标、roll目标为0：高度位置与速度来自同一长度参考。 */
    state->target[2] = 0.5f * (length_ramp_target[0] * cosf(lqr->target[LQR_X_THL]) + length_ramp_target[1] * cosf(lqr->target[LQR_X_THR]));
    state->target[3] = 0.5f * (length_rate_target[0] * cosf(lqr->target[LQR_X_THL]) + length_rate_target[1] * cosf(lqr->target[LQR_X_THR]));
    state->height_target = 0.5f * (lqr->leg_length_target[0] * cosf(lqr->target[LQR_X_THL]) + lqr->leg_length_target[1] * cosf(lqr->target[LQR_X_THR]));
    if (!isfinite(state->x[2]) || !isfinite(state->height_rate_raw) || !isfinite(state->target[2]) || !isfinite(state->target[3]))
    {
        return 0u;
    }
    state->x[1] = LPF_Update(&state->lpf_omg_roll, lqr->roll_rate);
    state->x[3] = LPF_Update(&state->lpf_height_rate, state->height_rate_raw);
    return 1u;
}

/* 原支撑+向心-偏置-K误差；只生成补偿前左右F。 */
static uint8_t Roll_Height_Force_Update(roll_height_balance_t *state, float turn_ff)
{
    float value;

    for (uint8_t i = 0u; i < RH_NU; i++)
    {
        state->lqr.base_force[i] = machine->lqr.support_force[i] + (i == 0u ? -turn_ff : turn_ff) - state->observer.bias[i];
        value = state->lqr.base_force[i];
        for (uint8_t j = 0u; j < RH_NX; j++)
        {
            state->lqr.feedback[i][j] = -roll_height_model.K[i][j] * (state->x[j] - state->target[j]);
            value += state->lqr.feedback[i][j];
        }
        if (!isfinite(value))
        {
            return 0u;
        }
        /* 允许负F主动压缩；只拦住深负值，不改其余计算。 */
        state->force[i] = fmaxf(value, ROLL_HEIGHT_FORCE_MIN);
    }
    return 1u;
}

/* 所有新计算异常统一清历史并拒绝本拍候选。 */
static uint8_t Roll_Height_Fail(roll_height_balance_t *state)
{
    Roll_Height_Reset(state);
    return 0u;
}

uint8_t Roll_Height_Update(roll_height_balance_t *state, const lqr_state_t *lqr,
                          const leg_state_t *left, const leg_state_t *right,
                          const float length_ramp_target[RH_NU], const float length_rate_target[RH_NU], float turn_ff)
{
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
    if (!Roll_Height_State_Update(state, lqr, left, right, length_ramp_target, length_rate_target))
    {
        return Roll_Height_Fail(state);
    }

    /* 保持原计时范围：状态/滤波更新之后，观测及反馈计算之前。 */
    started = (uint32_t)(Mono_Ns_Get() / 1000u);
    state->valid = 0u;
    measurement[0] = state->x[0];
    measurement[1] = lqr->roll_rate;
    measurement[2] = state->x[2];
    measurement[3] = state->height_rate_raw;
    /* 观测用原始速度，避免将控制低通的延迟误认为物理扰动。 */
    if (!Roll_Height_Observe(state, measurement))
    {
        return Roll_Height_Fail(state);
    }
    if (!Roll_Height_Force_Update(state, turn_ff))
    {
        return Roll_Height_Fail(state);
    }
    state->elapsed_us = (uint32_t)(Mono_Ns_Get() / 1000u) - started;
    if (state->elapsed_us > state->max_elapsed_us)
    {
        state->max_elapsed_us = state->elapsed_us;
    }
    state->update_count++;
    state->valid = 1u;
    return 1u;
}
