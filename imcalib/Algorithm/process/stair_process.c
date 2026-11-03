/* 自动触发后锁存摆腿/收腿，真实期限兜底；退出由任务层重新判断姿态。 */
#include "stair_process.h"
#include <math.h>
#include <string.h>

stair_process_t stair_process;

void Stair_Process_Init(stair_process_t *state)
{
    memset(state, 0, sizeof(*state));
}

/* 释放输出归属并清控制历史；保留最后一次结果/原因和序号供诊断。 */
void Stair_Process_Reset(stair_process_t *state)
{
    uint32_t sequence = state->sequence;
    uint8_t result = state->result;
    uint8_t reason = state->exit_reason;
    float total = state->total_elapsed;
    memset(state, 0, sizeof(*state));
    state->sequence = sequence;
    state->result = result;
    state->exit_reason = reason;
    state->total_elapsed = total;
}

uint8_t Stair_Process_Owns_Output(const stair_process_t *state)
{
    return (uint8_t)(state->phase != STAIR_PROCESS_IDLE);
}

/* 轨迹只限速，不改变实物角方向；连续角目标分支由本机实测配置确定。 */
static float Stair_Process_Target(float current, float target, float rate, float dt)
{
    return current + clampf(target - current, -rate * dt, rate * dt);
}

/* 使用自起同口径的误差差分PID；每段首次预置历史，不借用自起实例。 */
static void Stair_Process_PID_Init(stair_process_t *state, const stair_control_param_t *control)
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        PID_struct_init(&state->length_pid[i], POSITION_PID, control->length_pd[i].max_output, 0.0f, control->length_pd[i].kp, 0.0f, control->length_pd[i].kd, 0.0f, 0.0f);
        PID_struct_init(&state->angle_pos_pid[i], POSITION_PID, control->angle_pos.max_output, 0.0f, control->angle_pos.kp, 0.0f, control->angle_pos.kd, 0.0f, 0.0f);
        state->angle_pos_pid[i].angle_wrap = 1u;
        PID_struct_init(&state->angle_speed_pid[i], POSITION_PID, control->angle_speed.max_output, 0.0f, control->angle_speed.kp, 0.0f, control->angle_speed.kd, 0.0f, 0.0f);
    }
}

uint8_t Stair_Process_Begin(stair_process_t *state, const stair_process_param_t *param, const stair_process_input_t *input)
{
    uint32_t sequence = state->sequence;
    if (Stair_Process_Owns_Output(state) || !param->confirmed || !input->permit) { return 0u; }
    Stair_Process_Init(state);
    state->sequence = sequence + 1u;
    state->phase = STAIR_PROCESS_SWING;
    state->result = STAIR_RESULT_RUNNING;
    state->begin_ms = input->now_ms;
    state->phase_begin_ms = input->now_ms;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        state->position[i] = input->leg[i]->output.virtual_leg_angle;
        state->last_angle[i] = state->position[i];
        state->angle_target[i] = state->position[i];
        state->length_target[i] = input->leg[i]->output.virtual_leg_length;
        state->swing_length[i] = state->length_target[i];
        state->stall_begin_ms[i] = input->now_ms;
        state->stall_remaining[i] = fabsf(param->swing_angle[i] - state->position[i]);
    }
    return 1u;
}

void Stair_Process_Abort(stair_process_t *state, uint8_t reason, uint32_t now_ms)
{
    if (state->phase != STAIR_PROCESS_SWING && state->phase != STAIR_PROCESS_RETRACT) { return; }
    state->total_elapsed = (float)(uint32_t)(now_ms - state->begin_ms) * 0.001f;
    state->phase = STAIR_PROCESS_WAIT;
    state->phase_begin_ms = now_ms;
    state->elapsed = 0.0f;
    state->stable_elapsed = 0.0f;
    state->exit_reason = reason;
    state->result = reason == STAIR_EXIT_CANCELLED ? STAIR_RESULT_CANCELLED : STAIR_RESULT_FAILED;
    memset(state->force, 0, sizeof(state->force));
    memset(state->torque, 0, sizeof(state->torque));
    memset(&state->compensation, 0, sizeof(state->compensation));
}

uint8_t Stair_Process_Advance_Exit(stair_process_t *state, const stair_process_param_t *param, uint32_t now_ms)
{
    if (state->phase == STAIR_PROCESS_WAIT)
    {
        state->elapsed = (float)(uint32_t)(now_ms - state->phase_begin_ms) * 0.001f;
        if ((uint32_t)(now_ms - state->phase_begin_ms) >= param->wait_ms)
        {
            state->phase = STAIR_PROCESS_RECHECK;
            state->phase_begin_ms = now_ms;
            state->recheck_pending = 1u;
        }
    }
    if (state->phase == STAIR_PROCESS_RECHECK)
    {
        state->elapsed = (float)(uint32_t)(now_ms - state->phase_begin_ms) * 0.001f;
        if (state->recheck_pending)
        {
            state->recheck_pending = 0u;
            return 1u;
        }
    }
    return 0u;
}

/* 只在指令与实际仍有偏差时检测卡滞；两腿独立，不能由另一条腿的运动掩盖。 */
static uint8_t Stair_Process_Stalled(stair_process_t *state, const stair_process_param_t *param, uint8_t leg, float tracking_error, float remaining, float tolerance, float progress, uint32_t now_ms)
{
    if (tracking_error <= tolerance)
    {
        state->stall_begin_ms[leg] = now_ms;
        state->stall_remaining[leg] = remaining;
    }
    else if ((uint32_t)(now_ms - state->stall_begin_ms[leg]) >= param->stall_timeout_ms)
    {
        if (state->stall_remaining[leg] - remaining < progress) { return 1u; }
        state->stall_begin_ms[leg] = now_ms;
        state->stall_remaining[leg] = remaining;
    }
    return 0u;
}

void Stair_Process_Update(stair_process_t *state, const stair_process_param_t *param, const stair_process_input_t *input, float dt, torque_output_t *output)
{
    float base[DM_MOTOR_NUM];
    uint8_t ready = 1u;
    uint8_t swing = (uint8_t)(state->phase == STAIR_PROCESS_SWING);
    uint32_t phase_ms = (uint32_t)(input->now_ms - state->phase_begin_ms);
    const stair_control_param_t *control = input->control;
    Torque_Output_Clear(output);
    if (!swing && state->phase != STAIR_PROCESS_RETRACT) { return; }
    state->elapsed = (float)phase_ms * 0.001f;
    state->total_elapsed = (float)(uint32_t)(input->now_ms - state->begin_ms) * 0.001f;
    if (!input->permit)
    {
        Stair_Process_Abort(state, STAIR_EXIT_INPUT, input->now_ms);
        return;
    }
    if ((uint32_t)(input->now_ms - state->begin_ms) >= param->total_timeout_ms)
    {
        Stair_Process_Abort(state, STAIR_EXIT_TOTAL_TIMEOUT, input->now_ms);
        return;
    }
    if (phase_ms >= (swing ? param->swing_timeout_ms : param->retract_timeout_ms))
    {
        Stair_Process_Abort(state, swing ? STAIR_EXIT_SWING_TIMEOUT : STAIR_EXIT_RETRACT_TIMEOUT, input->now_ms);
        return;
    }
    if (!state->controllers_ready) { Stair_Process_PID_Init(state, control); }
    if (swing) { state->support = clampf(state->support + dt / control->support_time, 0.0f, 1.0f); }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        const leg_output_t *leg = &input->leg[i]->output;
        float goal = swing ? state->swing_length[i] : param->retract_length[i];
        float remaining;
        float tracking;
        float angle_speed_target;
        float support_weight;
        state->position[i] += Angle_Wrap_Pi(leg->virtual_leg_angle - state->last_angle[i]);
        state->last_angle[i] = leg->virtual_leg_angle;
        state->length_target[i] = Ramp_Target_Update(&state->length_ramp[i], leg->virtual_leg_length, goal, control->length_rate, dt);
        if (!isfinite(state->length_target[i]))
        {
            Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms);
            return;
        }
        if (!state->controllers_ready) { PID_History_Set(&state->length_pid[i], leg->virtual_leg_length, state->length_target[i], state->length_target[i] - leg->virtual_leg_length); }
        state->force[i] = pid_calc(&state->length_pid[i], leg->virtual_leg_length, state->length_target[i], dt);
        state->torque[i] = 0.0f;
        if (swing)
        {
            state->angle_target[i] = Stair_Process_Target(state->angle_target[i], param->swing_angle[i], control->angle_rate, dt);
            if (!state->controllers_ready) { PID_History_Set(&state->angle_pos_pid[i], state->position[i], state->angle_target[i], Angle_Wrap_Pi(state->angle_target[i] - state->position[i])); }
            angle_speed_target = pid_calc(&state->angle_pos_pid[i], state->position[i], state->angle_target[i], dt);
            if (!state->controllers_ready) { PID_History_Set(&state->angle_speed_pid[i], leg->d_virtual_leg_angle, angle_speed_target, angle_speed_target - leg->d_virtual_leg_angle); }
            state->torque[i] = pid_calc(&state->angle_speed_pid[i], leg->d_virtual_leg_angle, angle_speed_target, dt);
            support_weight = clampf(1.0f - fabsf(Angle_Wrap_Pi(state->position[i] - control->leg_trim[i])) / control->support_angle_range, 0.0f, 1.0f);
            state->force[i] += state->support * support_weight * control->support_force[i];
            remaining = fabsf(param->swing_angle[i] - state->position[i]);
            tracking = fabsf(state->angle_target[i] - state->position[i]);
            ready &= (uint8_t)(remaining <= param->angle_tolerance && fabsf(state->angle_target[i] - param->swing_angle[i]) <= 1.0e-6f);
        }
        else
        {
            remaining = fabsf(goal - leg->virtual_leg_length);
            tracking = fabsf(state->length_target[i] - leg->virtual_leg_length);
        }
        ready &= (uint8_t)(fabsf(goal - leg->virtual_leg_length) <= param->length_tolerance && fabsf(state->length_target[i] - goal) <= 1.0e-6f);
        if (!isfinite(state->position[i]) || !isfinite(state->force[i]) || !isfinite(state->torque[i]))
        {
            Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms);
            return;
        }
        if (Stair_Process_Stalled(state, param, i, tracking, remaining, swing ? param->angle_tolerance : param->length_tolerance, swing ? param->stall_angle_progress : param->stall_length_progress, input->now_ms))
        {
            Stair_Process_Abort(state, STAIR_EXIT_STALL, input->now_ms);
            return;
        }
    }
    state->controllers_ready = 1u;
    state->stable_elapsed = ready ? state->stable_elapsed + dt : 0.0f;
    if (ready && state->stable_elapsed + 1.0e-6f >= param->confirm_time)
    {
        state->stable_elapsed = 0.0f;
        state->elapsed = 0.0f;
        state->phase_begin_ms = input->now_ms;
        if (swing)
        {
            state->phase = STAIR_PROCESS_RETRACT;
            state->controllers_ready = 0u;
            for (uint8_t i = 0u; i < 2u; i++)
            {
                state->length_target[i] = input->leg[i]->output.virtual_leg_length;
                state->stall_begin_ms[i] = input->now_ms;
                state->stall_remaining[i] = fabsf(param->retract_length[i] - state->length_target[i]);
                Ramp_Reset(&state->length_ramp[i], state->length_target[i]);
                state->force[i] = 0.0f;
                state->torque[i] = 0.0f;
            }
        }
        else
        {
            state->phase = STAIR_PROCESS_RECHECK;
            state->recheck_pending = 1u;
            state->result = STAIR_RESULT_DONE;
            state->exit_reason = STAIR_EXIT_COMPLETE;
            memset(state->force, 0, sizeof(state->force));
            memset(state->torque, 0, sizeof(state->torque));
            memset(&state->compensation, 0, sizeof(state->compensation));
            return;
        }
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!Leg_Force_Map_Forward(input->leg[i], state->force[i], state->torque[i], &base[2u * i]))
        {
            Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms);
            return;
        }
    }
    if (!Leg_Compensation_Apply(&input->compensation->spring, &input->compensation->gravity, base, input->hip_limit, output->dm, &state->compensation))
    {
        Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms);
        Torque_Output_Clear(output);
        return;
    }
    output->valid = 1u;
}
