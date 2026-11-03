/* 自动触发后锁存摆腿/收腿，真实期限兜底；退出由任务层重新判断姿态。 */
#include "stair_process.h"
#include "machine_config.h"
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


/* 使用自起同口径的误差差分PID；每段首次预置历史，不借用自起实例。 */
static void Stair_Process_PID_Init(stair_process_t *state)
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        PID_struct_init(&state->length_pid[i], POSITION_PID, machine->lqr.pd.leg_len[i].max_output, 0.0f, machine->lqr.pd.leg_len[i].kp, 0.0f, machine->lqr.pd.leg_len[i].kd, 0.0f, 0.0f);
        PID_struct_init(&state->angle_pos_pid[i], POSITION_PID, fallen_process_param.angle_speed_max, 0.0f, fallen_process_param.angle_pos_kp, 0.0f, fallen_process_param.angle_pos_kd, 0.0f, 0.0f);
        state->angle_pos_pid[i].angle_wrap = 1u;
        PID_struct_init(&state->angle_speed_pid[i], POSITION_PID, machine->hip.dm_trq_clamp, 0.0f, fallen_process_param.angle_speed_kp, 0.0f, fallen_process_param.angle_speed_kd, 0.0f, 0.0f);
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
        Ramp_Reset(&state->angle_ramp[i], state->position[i]);
        state->length_target[i] = input->leg[i]->output.virtual_leg_length;
        state->swing_length[i] = state->length_target[i];
        state->stall_begin_ms[i] = input->now_ms;
        state->stall_remaining[i] = 0.0f; /* 首个计算拍由本机后点初始化 */
    }
    return 1u;
}

void Stair_Process_Abort(stair_process_t *state, uint8_t reason, uint32_t now_ms)
{
    if (state->phase != STAIR_PROCESS_SWING && state->phase != STAIR_PROCESS_RETRACT && state->phase != STAIR_PROCESS_ALIGN) { return; }
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

void Stair_Process_Complete(stair_process_t *state, uint32_t now_ms)
{
    state->total_elapsed = (float)(uint32_t)(now_ms - state->begin_ms) * 0.001f;
    state->phase = STAIR_PROCESS_IDLE;
    state->phase_begin_ms = now_ms;
    state->recheck_pending = 0u;
    state->result = STAIR_RESULT_DONE;
    state->exit_reason = STAIR_EXIT_COMPLETE;
    memset(state->force, 0, sizeof(state->force));
    memset(state->torque, 0, sizeof(state->torque));
    memset(&state->compensation, 0, sizeof(state->compensation));
}

/* 台阶只生成后摆候选；RETRACT阶段的收腿/摆正由任务委托普通自起实现。 */
void Stair_Process_Update(stair_process_t *state, const stair_process_param_t *param, const stair_process_input_t *input, float dt, torque_output_t *output)
{
    float base[DM_MOTOR_NUM];
    uint8_t ready = 1u;
    Torque_Output_Clear(output);
    if (state->phase != STAIR_PROCESS_SWING) { return; }
    state->elapsed = (float)(uint32_t)(input->now_ms - state->phase_begin_ms) * 0.001f;
    state->total_elapsed = (float)(uint32_t)(input->now_ms - state->begin_ms) * 0.001f;
    if (!input->permit) { Stair_Process_Abort(state, STAIR_EXIT_INPUT, input->now_ms); return; }
    if ((uint32_t)(input->now_ms - state->begin_ms) >= param->total_timeout_ms) { Stair_Process_Abort(state, STAIR_EXIT_TOTAL_TIMEOUT, input->now_ms); return; }
    if ((uint32_t)(input->now_ms - state->phase_begin_ms) >= param->swing_timeout_ms) { Stair_Process_Abort(state, STAIR_EXIT_SWING_TIMEOUT, input->now_ms); return; }
    if (!state->controllers_ready)
    {
        Stair_Process_PID_Init(state);
        for (uint8_t i = 0u; i < 2u; i++) { state->stall_remaining[i] = fabsf(fallen_process_param.rear_angle - state->position[i]); }
    }
    state->support = clampf(state->support + dt / fallen_process_param.support_time, 0.0f, 1.0f);
    for (uint8_t i = 0u; i < 2u; i++)
    {
        const leg_output_t *leg = &input->leg[i]->output;
        float speed_target;
        float remaining;
        float tracking;
        float weight;
        state->position[i] += Angle_Wrap_Pi(leg->virtual_leg_angle - state->last_angle[i]);
        state->last_angle[i] = leg->virtual_leg_angle;
        state->length_target[i] = Ramp_Target_Update(&state->length_ramp[i], leg->virtual_leg_length, state->swing_length[i], fallen_process_param.length_rate, dt);
        if (!isfinite(state->length_target[i])) { Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms); return; }
        if (!state->controllers_ready) { PID_History_Set(&state->length_pid[i], leg->virtual_leg_length, state->length_target[i], state->length_target[i] - leg->virtual_leg_length); }
        state->force[i] = pid_calc(&state->length_pid[i], leg->virtual_leg_length, state->length_target[i], dt);
        state->angle_ramp[i].rate = param->angle_rate;
        Ramp_Update(&state->angle_ramp[i], fallen_process_param.rear_angle, dt);
        if (!state->controllers_ready) { PID_History_Set(&state->angle_pos_pid[i], state->position[i], state->angle_ramp[i].out, Angle_Wrap_Pi(state->angle_ramp[i].out - state->position[i])); }
        speed_target = pid_calc(&state->angle_pos_pid[i], state->position[i], state->angle_ramp[i].out, dt);
        if (!state->controllers_ready) { PID_History_Set(&state->angle_speed_pid[i], leg->d_virtual_leg_angle, speed_target, speed_target - leg->d_virtual_leg_angle); }
        state->torque[i] = pid_calc(&state->angle_speed_pid[i], leg->d_virtual_leg_angle, speed_target, dt);
        weight = clampf(1.0f - fabsf(Angle_Wrap_Pi(state->position[i] - machine->lqr.reference.leg_trim[i])) / machine->recovery.fallen_support_angle_range, 0.0f, 1.0f);
        state->force[i] += state->support * weight * machine->lqr.roll_height.support_force[i];
        remaining = fabsf(fallen_process_param.rear_angle - state->position[i]);
        tracking = fabsf(state->angle_ramp[i].out - state->position[i]);
        ready &= (uint8_t)(remaining <= param->angle_tolerance && fabsf(state->angle_ramp[i].out - fallen_process_param.rear_angle) <= 1.0e-6f);
        if (!isfinite(state->position[i]) || !isfinite(state->force[i]) || !isfinite(state->torque[i])) { Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms); return; }
        if (Stair_Process_Stalled(state, param, i, tracking, remaining, param->angle_tolerance, param->stall_angle_progress, input->now_ms)) { Stair_Process_Abort(state, STAIR_EXIT_STALL, input->now_ms); return; }
    }
    state->controllers_ready = 1u;
    state->stable_elapsed = ready ? state->stable_elapsed + dt : 0.0f;
    if (ready && state->stable_elapsed + 1.0e-6f >= param->confirm_time)
    {
        state->phase = STAIR_PROCESS_RETRACT;
        state->phase_begin_ms = input->now_ms;
        state->elapsed = state->stable_elapsed = 0.0f;
        memset(state->force, 0, sizeof(state->force));
        memset(state->torque, 0, sizeof(state->torque));
        return;
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!Leg_Force_Map_Forward(input->leg[i], state->force[i], state->torque[i], &base[2u * i])) { Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms); return; }
    }
    if (!Leg_Compensation_Apply(&input->compensation->spring, &input->compensation->gravity, base, machine->hip.dm_trq_clamp, output->dm, &state->compensation)) { Stair_Process_Abort(state, STAIR_EXIT_MAPPING, input->now_ms); Torque_Output_Clear(output); return; }
    output->valid = 1u;
}
