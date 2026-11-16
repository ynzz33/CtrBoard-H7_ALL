/* USB任务收发实际推理请求、动作回包和遥测，策略与执行任务只发布数据。 */
#include "jetson_usb.h"
#include "jetson_protocol.h"
#include "stair_process.h"
#include "control_condition.h"
#include "air_detection.h"
#include "robot_state.h"
#include "mono_ns.h"
#include "usbd_cdc_if.h"
#include "dma_cache.h"
#include "math_utils.h"
#include "task.h"

#if MACHINE_LATENCY_CAPTURE_ENABLE
#error Jetson control requires the JRL2 USB transport
#endif

typedef struct {
    uint64_t input_ready_us;
    uint64_t session;
    uint32_t sequence;
    uint32_t reset_generation;
    float obs[RL_OBS_SIZE];
    float history[RL_HISTORY_SIZE];
} jetson_request_t;

/* 策略任务提交单槽快照，USB任务取走后释放。 */
static jetson_request_t request;
static volatile uint8_t request_ready;
static uint32_t request_sequence;
static volatile uint32_t peer_ready_ms;
static volatile uint8_t peer_seen;
static uint64_t peer_session;
static TaskHandle_t volatile usb_task;
static volatile uint8_t external_blocked;
static volatile uint8_t cold_restart_pending;
static uint64_t cold_restart_session;
static uint32_t cold_restart_sent_ms;
static uint64_t accepted_input_us, accepted_session;
static uint32_t accepted_reset, accepted_seq, accepted_return_us;
static uint32_t first_pd_age_us;
enum { ExternalReplyUs = 10000u, ExternalActionUs = 20000u, ExternalWarmupMs = 30u };
enum { ColdRestartRetryMs = 50u };

/* Jetson每50ms更新就绪，只有RL投入且就绪在有效期内才发送请求。 */
static uint8_t Link_Ready(void)
{
    return !cold_restart_pending && peer_seen &&
           (uint32_t)(HAL_GetTick() - peer_ready_ms) < 200u &&
           output_task_rl_engaged();
}

enum { QueueSize = 32, BatchSize = 2 };
enum {
    RlSelected = 1u, RlEngaged = 2u, ActionReady = 4u, PdValid = 8u,
    Dispatched = 16u, ObsValid = 32u, Warmup = 64u, DrivePermit = 128u,
    DmSubmitted = 256u, WheelSubmitted = 512u
};

typedef struct {
    uint64_t time_us;
    uint32_t sequence, reset, flags, fault, drops, obs_sequence, obs_ms;
    float obs[RL_OBS_SIZE];
    float target[6], current[6], velocity[6], pd_nm[6], request_nm[6];
} telemetry_sample_t;

static telemetry_sample_t queue[QueueSize];
static volatile uint32_t head, tail;
static uint32_t sequence, drops;
typedef struct {
    uint64_t time_us;
    uint32_t trial, sequence;
    uint32_t status[6];
    uint8_t type;
    float values[JRL_STAIR_FLOATS];
} diagnostic_sample_t;
static diagnostic_sample_t swing_queue[QueueSize];
static volatile uint32_t swing_head, swing_tail;
static uint32_t swing_sequence, swing_drops;
static uint32_t stair_sequence, stair_drops;

void Jetson_Usb_Record_Swing(uint32_t trial, const uint32_t status[6], const float values[JRL_SWING_FLOATS])
{
    ++swing_sequence;
    if (swing_head - swing_tail == QueueSize)
    {
        ++swing_drops;
        return;
    }
    diagnostic_sample_t *sample = &swing_queue[swing_head % QueueSize];
    sample->type = JRL_SWING_SAMPLE;
    sample->trial = trial;
    sample->sequence = swing_sequence;
    sample->time_us = Mono_Ns_Get() / 1000u;
    memcpy(sample->status, status, sizeof(sample->status));
    sample->status[1] = swing_drops;
    memcpy(sample->values, values, JRL_SWING_FLOATS * sizeof(float));
    __DMB();
    ++swing_head;
}

/* 台阶字段随现有USB遥测采样连续记录，台阶状态仅用于标注。 */
static void Record_Stair(const control_frame_t *input, uint8_t dispatched)
{
    ++stair_sequence;
    if (swing_head - swing_tail == QueueSize)
    {
        ++stair_drops;
        return;
    }
    diagnostic_sample_t *sample = &swing_queue[swing_head % QueueSize];
    sample->type = JRL_STAIR_SAMPLE;
    sample->time_us = Mono_Ns_Get() / 1000u;
    sample->trial = stair_process.sequence;
    sample->sequence = stair_sequence;
    sample->status[0] = input->imu.online | (input->motor_valid << 1) | (input->left.output.valid << 2) |
        (input->right.output.valid << 3) | (input->height_valid << 4) | (input->drive_permit << 5) |
        (dispatched << 6) | ((dispatched && output_debug_dm_sent) << 7) | ((dispatched && output_debug_dji_sent) << 8) |
        (control_condition.valid << 9) | (air_detection.valid << 10) | (air_detection.flight << 11);
    sample->status[1] = stair_drops;
    sample->status[2] = ctrl_fault;
    sample->status[3] = stair_process.phase;
    sample->status[4] = stair_process.result | ((uint32_t)stair_process.exit_reason << 8);
    sample->status[5] = control_condition.condition;
    float *v = sample->values;
    memcpy(v, input->imu.quat, 4 * sizeof(float)); v += 4;
    memcpy(v, input->imu.euler_rad, 3 * sizeof(float)); v += 3;
    memcpy(v, input->imu.gyro_rad_s, 3 * sizeof(float)); v += 3;
    memcpy(v, input->imu.acc_g, 3 * sizeof(float)); v += 3;
    memcpy(v, input->motor_pos, 4 * sizeof(float)); v += 4;
    memcpy(v, input->motor_vel, 4 * sizeof(float)); v += 4;
    memcpy(v, input->joint_torque, 4 * sizeof(float)); v += 4;
    memcpy(v, input->wheel_vel, 2 * sizeof(float)); v += 2;
    for (unsigned i = 0; i < 4; ++i) { *v++ = output_dm_torque_nm[i]; }
    *v++ = output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT];
    *v++ = output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT];
    const leg_state_t *legs[] = {&input->left, &input->right};
    for (unsigned i = 0; i < 2; ++i)
    {
        *v++ = legs[i]->output.virtual_leg_length;
        *v++ = legs[i]->output.virtual_leg_angle;
        *v++ = legs[i]->output.d_virtual_leg_length;
        *v++ = legs[i]->output.d_virtual_leg_angle;
    }
    *v++ = input->height;
    *v++ = input->rc.velocity_target;
    *v++ = input->rc.yaw_rate_target;
    *v = input->rc.leg_length_target;
    __DMB();
    ++swing_head;
}
static float action_obs[RL_OBS_SIZE];
static uint32_t obs_sequence, obs_ms;
static uint8_t obs_warmup;
static uint8_t tx[BatchSize * (JRL_HEADER + JRL_TELEMETRY_BYTES + 4)
    + 2 * (JRL_HEADER + JRL_INPUT_BYTES + 4)
    + JRL_HEADER + JRL_EXTERNAL_PD_BYTES + 4
    + JRL_HEADER + JRL_EXTERNAL_RESTART_BYTES + 4
    + BatchSize * (JRL_HEADER + JRL_STAIR_SAMPLE_BYTES + 4)]
    __attribute__((aligned(DMA_CACHE_LINE_SIZE)));

/* Called in the same critical section that publishes the corresponding action. */
void Jetson_Usb_Record_Action_Observation(const float obs[RL_OBS_SIZE], uint8_t warmup)
{
    memcpy(action_obs, obs, sizeof(action_obs));
    ++obs_sequence;
    obs_ms = HAL_GetTick();
    obs_warmup = warmup;
}

void Jetson_Usb_Record_Sample(const control_frame_t *input, uint8_t pd_valid, uint8_t dispatched)
{
    Record_Stair(input, dispatched);
    ++sequence;
    if (head - tail == QueueSize) { ++drops; return; }
    telemetry_sample_t *sample = &queue[head % QueueSize];
    memset(sample, 0, sizeof(*sample));
    sample->time_us = Mono_Ns_Get() / 1000u;
    sample->sequence = sequence;
    sample->reset = action_state.reset_count;
    sample->fault = ctrl_fault;
    sample->drops = drops;
    if (ctrl_strategy == CTRL_STRATEGY_RL) { sample->flags |= RlSelected; }
    if (output_task_rl_engaged()) { sample->flags |= RlEngaged; }
    if (input->drive_permit) { sample->flags |= DrivePermit; }
    if (action_state.rl_ready && action_state.updated)
    {
        sample->flags |= ActionReady | ObsValid;
        if (obs_warmup) { sample->flags |= Warmup; }
        sample->obs_sequence = obs_sequence;
        sample->obs_ms = obs_ms;
        memcpy(sample->obs, action_obs, sizeof(sample->obs));
    }
    if (pd_valid)
    {
        const rl_torque_state_t *pd = &rl_control.torque_state;
        sample->flags |= PdValid;
        memcpy(sample->target, pd->target, sizeof(sample->target));
        memcpy(sample->current, pd->pos, sizeof(pd->pos));
        memcpy(sample->current + 4, pd->vel + 4, 2 * sizeof(float));
        memcpy(sample->velocity, pd->vel, sizeof(sample->velocity));
        memcpy(sample->pd_nm, pd->motor_torque, sizeof(sample->pd_nm));
    }
    if (dispatched)
    {
        sample->flags |= Dispatched;
        if (output_debug_dm_sent) { sample->flags |= DmSubmitted; }
        if (output_debug_dji_sent) { sample->flags |= WheelSubmitted; }
        for (unsigned i = 0; i < 4; ++i) { sample->request_nm[i] = output_dm_torque_nm[i]; }
        sample->request_nm[4] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT];
        sample->request_nm[5] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT];
    }
    __DMB();
    ++head;
}

static void Encode_Floats(uint8_t **out, const float *values, unsigned count)
{
    for (unsigned i = 0; i < count; ++i)
    {
        jrl_pf32(*out, values[i]);
        *out += 4;
    }
}

enum { RxSize = 512, PendingCount = 16 };
static uint8_t rx[RxSize];
static volatile uint32_t rx_head, rx_tail;
static jrl_parser_t parser;
static jetson_request_t outgoing, pending[PendingCount];
static uint8_t outgoing_ready;
static uint8_t pd_report[JRL_HEADER + JRL_EXTERNAL_PD_BYTES + 4];
static unsigned pd_report_length;
static uint32_t rejected_replies;

void Jetson_Usb_Notify_From_ISR(void)
{
    BaseType_t wake = pdFALSE;
    if (usb_task != NULL) { vTaskNotifyGiveFromISR(usb_task, &wake); }
    portYIELD_FROM_ISR(wake);
}

void Jetson_Usb_Request_Cold_Restart(void)
{
    taskENTER_CRITICAL();
    cold_restart_session = peer_session;
    cold_restart_sent_ms = HAL_GetTick() - ColdRestartRetryMs;
    cold_restart_pending = 1u;
    taskEXIT_CRITICAL();
}

uint8_t Jetson_Usb_Cold_Restart_Pending(void)
{
    return cold_restart_pending;
}

uint8_t Jetson_Usb_External_Link_Ready(void)
{
    return (uint8_t)(Link_Ready() && !external_blocked);
}

/* 超时或会话变化后锁止到遥控退出本轮RL，防止旧动作继续出力。 */
uint8_t Jetson_Usb_External_Action_Allowed(void)
{
    if (rc_command.online && (!rc_command.mode.engage || rc_command.mode.strategy != CTRL_STRATEGY_RL))
    {
        external_blocked = 0u;
        return 0u;
    }
    if (!action_state.rl_ready) { return 0u; }
    if (!Link_Ready() ||
        (accepted_input_us && accepted_reset == action_state.reset_count ?
         (accepted_session != peer_session || Mono_Ns_Get() / 1000u - accepted_input_us >= ExternalActionUs) :
         (uint32_t)(HAL_GetTick() - action_state.last_ok_tick) >= ExternalWarmupMs))
    {
        external_blocked = 1u;
    }
    return (uint8_t)!external_blocked;
}

void Jetson_Usb_External_Submit(const float *obs, const float *history, uint32_t reset)
{
    if (external_blocked) { return; }
    const uint64_t input_ready_us = Mono_Ns_Get() / 1000u;
    if (!Link_Ready()) { return; }
    ++request_sequence;
    if (request_ready) { return; }
    __DMB();
    request.input_ready_us = input_ready_us;
    request.session = peer_session;
    request.sequence = request_sequence;
    request.reset_generation = reset;
    memcpy(request.obs, obs, sizeof(request.obs));
    memcpy(request.history, history, sizeof(request.history));
    __DMB();
    request_ready = 1u;
    if (usb_task != NULL) { xTaskNotifyGive(usb_task); }
}

/* 发布有限且仍有效的动作，观测和动作在同一个临界区更新。 */
static uint8_t External_Accept(const jetson_request_t *sent, const uint8_t *payload)
{
    float action[RL_ACTION_SIZE];
    for (uint8_t i = 0; i < RL_ACTION_SIZE; ++i) { action[i] = jrl_f32(payload + 4u + 4u * i); }
    if (!Float_Array_Finite(action, RL_ACTION_SIZE)) { return 0u; }
    for (uint8_t i = 0; i < RL_ACTION_SIZE; ++i) { action[i] = clip(action[i], RL_ACTION_CLIP); }

    taskENTER_CRITICAL();
    const uint64_t age = Mono_Ns_Get() / 1000u - sent->input_ready_us;
    const uint8_t valid = (uint8_t)(age < ExternalReplyUs &&
        sent->session == peer_session && sent->reset_generation == action_state.reset_count &&
        (!accepted_input_us || accepted_reset != sent->reset_generation || sent->input_ready_us > accepted_input_us) &&
        rc_command.online && rc_command.mode.rc_enable && rc_command.mode.engage &&
        rc_command.mode.strategy == CTRL_STRATEGY_RL && rl_control.recovery.phase == RL_RECOVERY_NORMAL &&
        ctrl_fault == FAULT_NONE && Jetson_Usb_External_Action_Allowed());
    if (valid)
    {
        memcpy(action_state.a, action, sizeof(action));
        action_state.last_ok_tick = HAL_GetTick();
        action_state.updated = action_state.rl_ready = 1u;
        accepted_input_us = sent->input_ready_us;
        accepted_session = sent->session;
        accepted_reset = sent->reset_generation;
        accepted_seq = sent->sequence;
        accepted_return_us = (uint32_t)age;
        first_pd_age_us = 0u;
        Jetson_Usb_Record_Action_Observation(sent->obs, 0u);
    }
    taskEXIT_CRITICAL();
    return valid;
}

void Jetson_Usb_External_PD_Used(void)
{
    if (!accepted_input_us || accepted_reset != action_state.reset_count) { return; }
    if (first_pd_age_us == 0u)
    {
        first_pd_age_us = (uint32_t)(Mono_Ns_Get() / 1000u - accepted_input_us);
        RL_Observation_Set_Last_Action(&rl_control.observation, action_state.a);
    }
}

static void External_Encode_PD(void)
{
    static uint32_t logged_seq;
    if (pd_report_length || !first_pd_age_us || accepted_seq == logged_seq) { return; }
    taskENTER_CRITICAL();
    jrl_header(pd_report, JRL_EXTERNAL_PD, JRL_EXTERNAL_PD_BYTES, accepted_session,
               accepted_reset, accepted_seq, accepted_input_us);
    uint8_t *out = pd_report + JRL_HEADER;
    jrl_p32(out, JRL_EXTERNAL_CONTRACT);
    jrl_p32(out + 4, accepted_return_us);
    jrl_p32(out + 8, first_pd_age_us);
    jrl_p32(out + 12, rejected_replies);
    logged_seq = accepted_seq;
    taskEXIT_CRITICAL();
    pd_report_length = jrl_finish(pd_report);
}

void Jetson_Usb_Receive_From_ISR(const uint8_t *data, uint32_t length)
{
    if (length > RxSize - (rx_head - rx_tail)) { return; }
    for (uint32_t i = 0; i < length; ++i)
    {
        rx[(rx_head + i) % RxSize] = data[i];
    }
    __DMB();
    rx_head += length;
    Jetson_Usb_Notify_From_ISR();
}

static void Receive_Frames(void)
{
    const uint32_t end = rx_head;
    __DMB();
    while (rx_tail != end)
    {
        const uint8_t byte = rx[rx_tail % RxSize];
        __DMB();
        ++rx_tail;
        if (!jrl_feed(&parser, byte)) { continue; }
        const uint8_t *reply = parser.data;
        const uint8_t *payload = reply + JRL_HEADER;
        if (reply[5] == JRL_READY && jrl_u16(reply + 6) == JRL_READY_BYTES &&
            jrl_u32(payload) == JRL_EXTERNAL_CONTRACT)
        {
            const uint64_t session = jrl_u64(reply + 8);
            if (!session) { continue; }
            taskENTER_CRITICAL();
            if (session == cold_restart_session)
            {
                taskEXIT_CRITICAL();
                continue;
            }
            peer_session = session;
            peer_ready_ms = HAL_GetTick();
            peer_seen = 1u;
            if (cold_restart_pending)
            {
                external_blocked = 0u;
                cold_restart_pending = 0u;
            }
            taskEXIT_CRITICAL();
            continue;
        }
        const uint32_t seq = jrl_u32(reply + 20);
        jetson_request_t *sent = &pending[seq % PendingCount];
        if (reply[5] != JRL_REPLY || jrl_u16(reply + 6) != JRL_REPLY_BYTES ||
            jrl_u32(payload) != JRL_EXTERNAL_CONTRACT || jrl_u64(reply + 8) != sent->session ||
            sent->sequence != seq || !sent->input_ready_us ||
            sent->reset_generation != jrl_u32(reply + 16) ||
            sent->input_ready_us != jrl_u64(reply + 24))
        {
            ++rejected_replies;
            continue;
        }
        if (!External_Accept(sent, payload)) { ++rejected_replies; }
        sent->input_ready_us = 0;
    }
}

static unsigned Encode_Input(uint8_t *frame, const jetson_request_t *sample)
{
    unsigned length = 0;
    for (unsigned half = 0; half < 2; ++half)
    {
        jrl_header(frame + length, JRL_INPUT_FIRST + half, JRL_INPUT_BYTES,
                   sample->session, sample->reset_generation, sample->sequence, sample->input_ready_us);
        uint8_t *out = frame + length + JRL_HEADER;
        jrl_p32(out, JRL_EXTERNAL_CONTRACT);
        out += 4;
        for (unsigned i = 0; i < JRL_HALF_FLOATS; ++i)
        {
            const unsigned index = half * JRL_HALF_FLOATS + i;
            jrl_pf32(out, index < RL_OBS_SIZE ? sample->obs[index] : sample->history[index - RL_OBS_SIZE]);
            out += 4;
        }
        length += jrl_finish(frame + length);
    }
    return length;
}

/* 每次优先发送完整观测请求和PD回执，再追加最多两帧遥测。 */
static void Usb_Pump(void)
{
    Receive_Frames();
    External_Encode_PD();
    if (!CDC_Transmit_Ready_HS()) { return; }
    const uint32_t available = head - tail;
    __DMB();
    const uint32_t count = available < BatchSize ? available : BatchSize;
    unsigned length = 0;
    const uint32_t now = HAL_GetTick();
    uint64_t restart_session = 0u;
    taskENTER_CRITICAL();
    if (cold_restart_pending && ctrl_strategy == CTRL_STRATEGY_LQR && now - cold_restart_sent_ms >= ColdRestartRetryMs)
    {
        restart_session = cold_restart_session;
    }
    taskEXIT_CRITICAL();
    if (restart_session)
    {
        jrl_header(tx, JRL_EXTERNAL_RESTART, JRL_EXTERNAL_RESTART_BYTES, restart_session, action_state.reset_count, 0u, 0u);
        jrl_p32(tx + JRL_HEADER, JRL_EXTERNAL_CONTRACT);
        length = jrl_finish(tx);
    }
    if (!outgoing_ready && request_ready)
    {
        __DMB();
        outgoing = request;
        __DMB();
        request_ready = 0u;
        outgoing_ready = 1u;
    }
    if (outgoing_ready &&
        (!Link_Ready() || outgoing.reset_generation != action_state.reset_count ||
         external_blocked || outgoing.session != peer_session ||
         Mono_Ns_Get() / 1000u - outgoing.input_ready_us >= ExternalReplyUs))
    {
        outgoing_ready = 0u;
    }
    if (outgoing_ready) { length += Encode_Input(tx + length, &outgoing); }
    memcpy(tx + length, pd_report, pd_report_length);
    length += pd_report_length;
    for (uint32_t i = 0; i < count; ++i)
    {
        const telemetry_sample_t *s = &queue[(tail + i) % QueueSize];
        uint8_t *frame = tx + length;
        jrl_header(frame, JRL_TELEMETRY, JRL_TELEMETRY_BYTES, 0u, s->reset, s->sequence, s->time_us);
        uint8_t *p = frame + JRL_HEADER;
        jrl_p32(p, s->flags);
        jrl_p32(p + 4, s->fault);
        jrl_p32(p + 8, s->drops);
        jrl_p32(p + 12, s->obs_sequence);
        jrl_p32(p + 16, s->obs_ms);
        p += 20;
        Encode_Floats(&p, s->obs, RL_OBS_SIZE);
        Encode_Floats(&p, s->target, 6);
        Encode_Floats(&p, s->current, 6);
        Encode_Floats(&p, s->velocity, 6);
        Encode_Floats(&p, s->pd_nm, 6);
        Encode_Floats(&p, s->request_nm, 6);
        length += jrl_finish(frame);
    }
    const uint32_t swing_available = swing_head - swing_tail;
    const uint32_t swing_count = swing_available < BatchSize ? swing_available : BatchSize;
    __DMB();
    for (uint32_t i = 0; i < swing_count; ++i)
    {
        const diagnostic_sample_t *sample = &swing_queue[(swing_tail + i) % QueueSize];
        uint8_t *frame = tx + length;
        const uint8_t stair = sample->type == JRL_STAIR_SAMPLE;
        jrl_header(frame, sample->type, stair ? JRL_STAIR_SAMPLE_BYTES : JRL_SWING_SAMPLE_BYTES, 0u,
                   sample->trial, sample->sequence, sample->time_us);
        uint8_t *payload = frame + JRL_HEADER;
        for (unsigned j = 0; j < 6; ++j)
        {
            jrl_p32(payload, sample->status[j]);
            payload += 4;
        }
        Encode_Floats(&payload, sample->values, stair ? JRL_STAIR_FLOATS : JRL_SWING_FLOATS);
        length += jrl_finish(frame);
    }
    if (length && CDC_Transmit_HS(tx, (uint16_t)length) == USBD_OK)
    {
        if (restart_session) { cold_restart_sent_ms = now; }
        if (outgoing_ready)
        {
            pending[outgoing.sequence % PendingCount] = outgoing;
            outgoing_ready = 0u;
        }
        pd_report_length = 0u;
        __DMB();
        tail += count;
        swing_tail += swing_count;
    }
}

void Jetson_Usb_Task(void *argument)
{
    (void)argument;
    usb_task = xTaskGetCurrentTaskHandle();
    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, 1u);
        Usb_Pump();
    }
}
