/* 蓄力委托普通LQR；蹬伸/空中/落地由本模块生成F、Tp及轮矩，支撑力随实测腿长取静态值，腿长给定从实测起步。 */
#include "jump_process.h"
#include <math.h>
#include <string.h>

jump_process_t jump_process;

void Jump_Process_Init(jump_process_t *state)
{
    memset(state, 0, sizeof(*state));
}

/* 释放输出归属并清控制历史；保留最后一次结果、原因、着地依据、标志和序号供诊断。 */
void Jump_Process_Reset(jump_process_t *state)
{
    uint32_t sequence = state->sequence;
    uint8_t result = state->result;
    uint8_t reason = state->exit_reason;
    uint8_t touchdown = state->touchdown_source;
    uint8_t flags = state->flags;
    float total = state->total_elapsed;
    memset(state, 0, sizeof(*state));
    state->sequence = sequence;
    state->result = result;
    state->exit_reason = reason;
    state->touchdown_source = touchdown;
    state->flags = flags;
    state->total_elapsed = total;
}

uint8_t Jump_Process_Owns_Output(const jump_process_t *state)
{
    return (uint8_t)(state->phase != JUMP_PROCESS_IDLE && state->phase != JUMP_PROCESS_CROUCH);
}

/* 转段统一清阶段计时与连续确认。 */
static void Jump_Process_Enter(jump_process_t *state, uint8_t phase, uint32_t now_ms)
{
    state->phase = phase;
    state->phase_begin_ms = now_ms;
    state->elapsed = 0.0f;
    state->confirm_elapsed = 0.0f;
}

/* 收腿/缓冲给定从本拍实测腿长起步，避免转段时PD给定阶跃。 */
static void Jump_Process_Seed_Length(jump_process_t *state, const jump_process_input_t *input, float rate)
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        state->length_target[i] = Ramp_Reset(&state->length_ramp[i], input->leg[i]->output.virtual_leg_length);
        state->length_ramp[i].rate = rate;
    }
}

static void Jump_Process_Clear_Output(jump_process_t *state)
{
    memset(state->force, 0, sizeof(state->force));
    memset(state->torque, 0, sizeof(state->torque));
    memset(state->wheel, 0, sizeof(state->wheel));
    memset(&state->compensation, 0, sizeof(state->compensation));
}

uint8_t Jump_Process_Begin(jump_process_t *state, const jump_process_param_t *param, const jump_process_input_t *input)
{
    uint32_t sequence = state->sequence;
    if (state->phase != JUMP_PROCESS_IDLE || !param->confirmed || !input->permit) { return 0u; }
    Jump_Process_Init(state);
    state->sequence = sequence + 1u;
    state->result = JUMP_RESULT_RUNNING;
    state->begin_ms = input->now_ms;
    Jump_Process_Enter(state, JUMP_PROCESS_CROUCH, input->now_ms);
    state->length_target[0] = param->crouch_len;
    state->length_target[1] = param->crouch_len;
    return 1u;
}

void Jump_Process_Abort(jump_process_t *state, uint8_t reason, uint32_t now_ms)
{
    if (state->phase == JUMP_PROCESS_IDLE || state->phase == JUMP_PROCESS_WAIT || state->phase == JUMP_PROCESS_RECHECK) { return; }
    state->total_elapsed = (float)(uint32_t)(now_ms - state->begin_ms) * 0.001f;
    state->exit_reason = reason;
    state->result = reason == JUMP_EXIT_CANCELLED ? JUMP_RESULT_CANCELLED : JUMP_RESULT_FAILED;
    Jump_Process_Clear_Output(state);
    /* 蓄力未接管输出，直接交回仍在运行的普通LQR；接管后失败才撤力等待重判。 */
    Jump_Process_Enter(state, state->phase == JUMP_PROCESS_CROUCH ? JUMP_PROCESS_IDLE : JUMP_PROCESS_WAIT, now_ms);
}

void Jump_Process_Complete(jump_process_t *state, uint32_t now_ms)
{
    if (state->phase != JUMP_PROCESS_HANDOFF) { return; }
    state->total_elapsed = (float)(uint32_t)(now_ms - state->begin_ms) * 0.001f;
    state->result = JUMP_RESULT_DONE;
    state->exit_reason = JUMP_EXIT_COMPLETE;
    Jump_Process_Clear_Output(state);
    Jump_Process_Enter(state, JUMP_PROCESS_IDLE, now_ms);
}

uint8_t Jump_Process_Advance_Exit(jump_process_t *state, const jump_process_param_t *param, uint32_t now_ms)
{
    if (state->phase == JUMP_PROCESS_WAIT)
    {
        state->elapsed = (float)(uint32_t)(now_ms - state->phase_begin_ms) * 0.001f;
        if ((uint32_t)(now_ms - state->phase_begin_ms) >= param->wait_ms)
        {
            Jump_Process_Enter(state, JUMP_PROCESS_RECHECK, now_ms);
            state->recheck_pending = 1u;
        }
    }
    if (state->phase == JUMP_PROCESS_RECHECK)
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

/* 按当前阶段判据转段；转段当拍由调用者按新阶段出力，空中/着地不插入零力矩拍。返回0表示已中止。 */
static uint8_t Jump_Process_Advance(jump_process_t *state, const jump_process_param_t *param, const jump_process_input_t *input, float dt)
{
    const float *support_force = machine->lqr.roll_height.support_force;
    const leg_output_t *left = &input->leg[0]->output;
    const leg_output_t *right = &input->leg[1]->output;
    uint32_t phase_ms = (uint32_t)(input->now_ms - state->phase_begin_ms);
    uint8_t unloaded = (uint8_t)(input->support_valid && input->support[0] + input->support[1] < param->liftoff_ratio * (support_force[0] + support_force[1]));
    uint8_t source = 0u;
    uint8_t lift;

    if (state->phase == JUMP_PROCESS_THRUST)
    {
        /* 起飞锁存：支持力之和卸载且双腿仍快速伸长；行程保护先到则直接停推，不顶机械限位。 */
        lift = (uint8_t)(unloaded && left->d_virtual_leg_length > param->liftoff_speed && right->d_virtual_leg_length > param->liftoff_speed);
        state->confirm_elapsed = lift ? state->confirm_elapsed + dt : 0.0f;
        if (lift && state->confirm_elapsed + 1.0e-6f >= param->liftoff_time) { state->flags |= JUMP_FLAG_UNLOADED; }
        if (left->virtual_leg_length >= param->thrust_len || right->virtual_leg_length >= param->thrust_len) { state->flags |= JUMP_FLAG_STROKE_END; }
        if (state->flags & (JUMP_FLAG_UNLOADED | JUMP_FLAG_STROKE_END))
        {
            Jump_Process_Enter(state, JUMP_PROCESS_FLIGHT, input->now_ms);
            Jump_Process_Seed_Length(state, input, param->retract_rate);
        }
        else if (phase_ms >= param->thrust_timeout_ms)
        {
            Jump_Process_Abort(state, JUMP_EXIT_THRUST_TIMEOUT, input->now_ms);
            return 0u;
        }
    }
    else if (state->phase == JUMP_PROCESS_FLIGHT)
    {
        /* 支持力判据须先见到卸载，避免蹬伸末段残余支持力误判；压缩/速度判据须收腿给定到位后才生效。 */
        if (unloaded) { state->flags |= JUMP_FLAG_UNLOADED; }
        for (uint8_t i = 0u; i < 2u; i++)
        {
            const leg_output_t *leg = &input->leg[i]->output;
            if ((state->flags & JUMP_FLAG_UNLOADED) && input->support_valid && input->support[i] > param->touchdown_ratio * support_force[i]) { source |= JUMP_TOUCH_FORCE; }
            if ((state->flags & JUMP_FLAG_RETRACTED) && param->retract_len - leg->virtual_leg_length >= param->touchdown_compression) { source |= JUMP_TOUCH_COMPRESSION; }
            if ((state->flags & JUMP_FLAG_RETRACTED) && -leg->d_virtual_leg_length >= param->touchdown_speed) { source |= JUMP_TOUCH_SPEED; }
        }
        state->confirm_elapsed = source ? state->confirm_elapsed + dt : 0.0f;
        /* 空中超时多为漏判着地，按着地进入缓冲，不在空中撤力。 */
        if (!source || state->confirm_elapsed + 1.0e-6f < param->touchdown_time) { source = phase_ms >= param->flight_timeout_ms ? JUMP_TOUCH_TIMEOUT : 0u; }
        if (source)
        {
            Jump_Process_Enter(state, JUMP_PROCESS_LAND, input->now_ms);
            Jump_Process_Seed_Length(state, input, param->land_rate);
            state->touchdown_source = source;
        }
    }
    else if (state->phase == JUMP_PROCESS_LAND)
    {
        state->land_scale = clampf(state->land_scale + dt / param->land_time, 0.0f, 1.0f);
        if (state->land_scale >= 1.0f) { Jump_Process_Enter(state, JUMP_PROCESS_HANDOFF, input->now_ms); }
    }
    else if (phase_ms >= param->handoff_timeout_ms)
    {
        Jump_Process_Abort(state, JUMP_EXIT_HANDOFF_TIMEOUT, input->now_ms);
        return 0u;
    }
    return 1u;
}

/* 生成本拍F/Tp/轮矩：蹬伸为静态支撑+爬升推力，空中为收腿PD，落地/交接为支撑渐入+缓冲PD。
 * 蹬伸/落地/交接的Tp和轮矩沿用普通候选；空中或普通候选缺失时用世界腿角PD且轮矩0。返回0表示需中止。 */
static uint8_t Jump_Process_Law(jump_process_t *state, const jump_process_param_t *param, const jump_process_input_t *input, const float angle[2], float dt)
{
    const machine_pid_cfg_t *roll_pd = &machine->lqr.pd.roll;
    float roll_force = 0.0f;
    uint8_t own_balance = (uint8_t)(state->phase == JUMP_PROCESS_FLIGHT || !input->normal_valid);
    uint8_t retracted = 1u;

    if (state->phase == JUMP_PROCESS_THRUST) { state->thrust_scale = clampf(state->thrust_scale + param->thrust_ramp * dt, 0.0f, 1.0f); }
    if (state->phase != JUMP_PROCESS_FLIGHT)
    {
        /* 与leg_balance非NORMAL横滚PD同参数同口径：Kd乘基础周期并作用于实测角速度，左加右减。 */
        roll_force = clip(-roll_pd->kp * input->roll - roll_pd->kd * MACHINE_LQR_DT * input->roll_rate, roll_pd->max_output);
        if (!input->normal_valid) { state->flags |= JUMP_FLAG_FALLBACK; }
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        const leg_output_t *leg = &input->leg[i]->output;
        float previous = state->length_ramp[i].out;
        float reference_rate;
        float side = i == 0u ? roll_force : -roll_force;

        if (state->phase == JUMP_PROCESS_THRUST)
        {
            state->length_target[i] = param->thrust_len;
            /* 静态支撑随实测腿长取值：height trim沿行程变化，锁存蓄力时的力会让推力沿行程衰减。 */
            state->force[i] = input->static_force[i] + state->thrust_scale * param->thrust_force + side;
        }
        else
        {
            /* D作用于斜坡给定速度与实测伸缩速度之差，收腿过程中不反向阻碍跟踪。 */
            Ramp_Update(&state->length_ramp[i], state->phase == JUMP_PROCESS_FLIGHT ? param->retract_len : param->land_len, dt);
            state->length_target[i] = state->length_ramp[i].out;
            reference_rate = (state->length_target[i] - previous) / dt;
            /* 给定到位时腿仍以收腿速度运动，须实测速度也降到压缩速度门槛下才开放几何着地判据。 */
            retracted &= (uint8_t)(state->length_target[i] == param->retract_len && fabsf(leg->d_virtual_leg_length) < param->touchdown_speed);
            if (state->phase == JUMP_PROCESS_FLIGHT)
            {
                state->force[i] = param->flight_kp * (state->length_target[i] - leg->virtual_leg_length) + param->flight_kd * (reference_rate - leg->d_virtual_leg_length);
            }
            else
            {
                state->force[i] = state->land_scale * input->static_force[i] + param->land_kp * (state->length_target[i] - leg->virtual_leg_length) + param->land_kd * (reference_rate - leg->d_virtual_leg_length) + side;
            }
        }
        if (own_balance)
        {
            /* 正Tp增大几何腿角，与自起/台阶摆角环同一约定；空中轮矩只会让轮子空转，保持0。 */
            state->torque[i] = clip(param->angle_kp * Angle_Wrap_Pi(machine->lqr.reference.leg_trim[i] - angle[i]) - param->angle_kd * (leg->d_virtual_leg_angle - input->pitch_rate), param->angle_torque_max);
            state->wheel[i] = 0.0f;
        }
        else
        {
            state->torque[i] = input->normal_torque[i];
            state->wheel[i] = clampf(input->normal_wheel[i], -machine->wheel.dji_trq_clamp, machine->wheel.dji_trq_clamp);
            if (fabsf(input->normal_wheel[i]) >= machine->wheel.dji_trq_clamp) { state->flags |= JUMP_FLAG_WHEEL_LIMIT; }
        }
        if (!isfinite(state->wheel[i])) { return 0u; }
    }
    if (state->phase == JUMP_PROCESS_FLIGHT && retracted) { state->flags |= JUMP_FLAG_RETRACTED; }
    return 1u;
}

/* 每执行拍：先处理许可/期限，蓄力只推进判据；接管阶段硬中止检查→转段→力律→映射补偿。 */
void Jump_Process_Update(jump_process_t *state, const jump_process_param_t *param, const jump_process_input_t *input, float dt, torque_output_t *output)
{
    float base[DM_MOTOR_NUM];
    float angle[2];
    uint8_t ready;

    if (Jump_Process_Owns_Output(state)) { Torque_Output_Clear(output); }
    /* CROUCH～HANDOFF编号连续；IDLE不归属，WAIT/RECHECK只保持零候选。 */
    if (state->phase < JUMP_PROCESS_CROUCH || state->phase > JUMP_PROCESS_HANDOFF) { return; }
    state->elapsed = (float)(uint32_t)(input->now_ms - state->phase_begin_ms) * 0.001f;
    state->total_elapsed = (float)(uint32_t)(input->now_ms - state->begin_ms) * 0.001f;
    if (!input->permit) { Jump_Process_Abort(state, JUMP_EXIT_INPUT, input->now_ms); return; }
    if ((uint32_t)(input->now_ms - state->begin_ms) >= param->total_timeout_ms) { Jump_Process_Abort(state, JUMP_EXIT_TOTAL_TIMEOUT, input->now_ms); return; }
    if (state->phase == JUMP_PROCESS_CROUCH)
    {
        if ((uint32_t)(input->now_ms - state->phase_begin_ms) >= param->crouch_timeout_ms) { Jump_Process_Abort(state, JUMP_EXIT_CROUCH_TIMEOUT, input->now_ms); return; }
        ready = (uint8_t)(input->normal_valid && input->leg[0]->output.virtual_leg_length <= param->crouch_len + param->crouch_tol && input->leg[1]->output.virtual_leg_length <= param->crouch_len + param->crouch_tol);
        state->confirm_elapsed = ready ? state->confirm_elapsed + dt : 0.0f;
        if (ready && state->confirm_elapsed + 1.0e-6f >= param->crouch_hold)
        {
            /* 下一拍起本模块接管输出；推力系数从0爬升，首拍轴向力约等于普通静态支撑。 */
            Jump_Process_Enter(state, JUMP_PROCESS_THRUST, input->now_ms);
        }
        return;
    }
    /* 接管期间硬中止：反馈失效或姿态超界直接撤力；取反比较使NaN同样中止，pitch有限后才包角。 */
    if (!input->leg[0]->output.valid || !input->leg[1]->output.valid) { Jump_Process_Abort(state, JUMP_EXIT_INPUT, input->now_ms); return; }
    if (!(fabsf(input->pitch) <= param->pitch_max) || !(fabsf(input->roll) <= param->roll_max)) { Jump_Process_Abort(state, JUMP_EXIT_POSE, input->now_ms); return; }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        angle[i] = input->leg[i]->output.virtual_leg_angle - input->pitch;
        if (!(fabsf(Angle_Wrap_Pi(angle[i] - machine->lqr.reference.leg_trim[i])) <= param->angle_max)) { Jump_Process_Abort(state, JUMP_EXIT_POSE, input->now_ms); return; }
    }
    if (!Jump_Process_Advance(state, param, input, dt)) { return; }
    if (!Jump_Process_Law(state, param, input, angle, dt)) { Jump_Process_Abort(state, JUMP_EXIT_MAPPING, input->now_ms); return; }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!Leg_Force_Map_Forward(input->leg[i], state->force[i], state->torque[i], &base[2u * i])) { Jump_Process_Abort(state, JUMP_EXIT_MAPPING, input->now_ms); return; }
    }
    if (!Leg_Compensation_Apply(&input->compensation->spring, &input->compensation->gravity, base, machine->hip.dm_trq_clamp, output->dm, &state->compensation)) { Jump_Process_Abort(state, JUMP_EXIT_MAPPING, input->now_ms); Torque_Output_Clear(output); return; }
    output->dji[DJI_MOTOR_WHEEL_LFT] = state->wheel[0];
    output->dji[DJI_MOTOR_WHEEL_RGT] = state->wheel[1];
    output->valid = 1u;
}
