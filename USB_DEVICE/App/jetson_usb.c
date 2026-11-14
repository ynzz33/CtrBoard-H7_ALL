#include "jetson_usb.h"
#include "jetson_protocol.h"
#include "robot_state.h"
#include "mono_ns.h"
#include "usbd_cdc_if.h"
#include "dma_cache.h"
#include "jetson_shadow_protocol.h"

#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
/* One policy producer and one USB task consumer. The producer skips a full
 * slot. The consumer releases it only after copying a complete sample. */
static jetson_shadow_sample_t shadow_sample;
static volatile uint8_t shadow_ready;
static uint32_t shadow_sequence, shadow_drops;

uint8_t Jetson_Usb_Shadow_Begin(const float *obs, const float *history, uint64_t input_ready_us)
{
    ++shadow_sequence;
    if (shadow_ready) { ++shadow_drops; return 0u; }
    __DMB();
    shadow_sample.input_ready_us = input_ready_us;
    shadow_sample.sequence = shadow_sequence;
    shadow_sample.reset_generation = action_state.reset_count;
    shadow_sample.dropped = shadow_drops;
    memcpy(shadow_sample.obs, obs, sizeof(shadow_sample.obs));
    memcpy(shadow_sample.history, history, sizeof(shadow_sample.history));
    return 1u;
}

void Jetson_Usb_Shadow_Publish(const float *raw_action)
{
    memcpy(shadow_sample.raw_action, raw_action, sizeof(shadow_sample.raw_action));
    __DMB();
    shadow_ready = 1u;
}

uint8_t Jetson_Usb_Shadow_Take(jetson_shadow_sample_t *sample)
{
    if (!shadow_ready) { return 0u; }
    __DMB();
    *sample = shadow_sample;
    __DMB();
    shadow_ready = 0u;
    return 1u;
}
#endif

enum { QueueSize = 32, BatchSize = 8, FrameType = 5, PayloadSize = 240 };
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
static float action_obs[RL_OBS_SIZE];
static uint32_t obs_sequence, obs_ms;
static uint8_t obs_warmup;
static uint8_t tx[BatchSize * (JRL_HEADER + PayloadSize + 4)
#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
    + 2 * (JRL_HEADER + JRL_SHADOW_INPUT_BYTES + 4)
    + JRL_HEADER + JRL_SHADOW_RESULT_BYTES + 4
#endif
    ]
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

#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
enum { ShadowRxSize = 512, ShadowPendingCount = 16 };
static uint8_t shadow_rx[ShadowRxSize];
static volatile uint32_t shadow_rx_head, shadow_rx_tail, shadow_rx_drops;
static jrl_parser_t shadow_parser;
static jetson_shadow_sample_t shadow_outgoing, shadow_pending[ShadowPendingCount];
static uint8_t shadow_outgoing_ready;
static uint8_t shadow_result[JRL_HEADER + JRL_SHADOW_RESULT_BYTES + 4];
static unsigned shadow_result_length;
static uint32_t shadow_rejected;

void Jetson_Usb_Shadow_Receive(const uint8_t *data, uint32_t length)
{
    if (length > ShadowRxSize - (shadow_rx_head - shadow_rx_tail))
    {
        ++shadow_rx_drops;
        return;
    }
    for (uint32_t i = 0; i < length; ++i)
    {
        shadow_rx[(shadow_rx_head + i) % ShadowRxSize] = data[i];
    }
    __DMB();
    shadow_rx_head += length;
}

static void Shadow_Read_Reply(void)
{
    const uint32_t end = shadow_rx_head;
    __DMB();
    while (!shadow_result_length && shadow_rx_tail != end)
    {
        const uint8_t byte = shadow_rx[shadow_rx_tail % ShadowRxSize];
        __DMB();
        ++shadow_rx_tail;
        if (!jrl_feed(&shadow_parser, byte)) { continue; }
        const uint8_t *reply = shadow_parser.data;
        const uint8_t *payload = reply + JRL_HEADER;
        const uint32_t seq = jrl_u32(reply + 20);
        jetson_shadow_sample_t *sent = &shadow_pending[seq % ShadowPendingCount];
        if (reply[5] != JRL_SHADOW_REPLY || jrl_u16(reply + 6) != JRL_SHADOW_REPLY_BYTES ||
            jrl_u32(payload) != JRL_SHADOW_CONTRACT || jrl_u64(reply + 8) != 0 ||
            sent->sequence != seq || !sent->input_ready_us ||
            sent->reset_generation != jrl_u32(reply + 16) ||
            sent->input_ready_us != jrl_u64(reply + 24))
        {
            ++shadow_rejected;
            continue;
        }
        jrl_header(shadow_result, JRL_SHADOW_RESULT, JRL_SHADOW_RESULT_BYTES, 0,
                   sent->reset_generation, seq, sent->input_ready_us);
        uint8_t *out = shadow_result + JRL_HEADER;
        memcpy(out, payload, JRL_SHADOW_REPLY_BYTES);
        out += JRL_SHADOW_REPLY_BYTES;
        Encode_Floats(&out, sent->raw_action, JRL_SHADOW_ACTION_FLOATS);
        jrl_p32(out, (uint32_t)(Mono_Ns_Get() / 1000u - sent->input_ready_us));
        jrl_p32(out + 4, sent->dropped);
        jrl_p32(out + 8, shadow_rx_drops);
        jrl_p32(out + 12, shadow_parser.errors);
        jrl_p32(out + 16, shadow_rejected);
        shadow_result_length = jrl_finish(shadow_result);
        sent->input_ready_us = 0;
    }
}

static unsigned Shadow_Encode_Input(uint8_t *frame, const jetson_shadow_sample_t *sample)
{
    unsigned length = 0;
    for (unsigned half = 0; half < 2; ++half)
    {
        jrl_header(frame + length, JRL_SHADOW_INPUT_FIRST + half, JRL_SHADOW_INPUT_BYTES,
                   0, sample->reset_generation, sample->sequence, sample->input_ready_us);
        uint8_t *out = frame + length + JRL_HEADER;
        jrl_p32(out, JRL_SHADOW_CONTRACT);
        out += 4;
        for (unsigned i = 0; i < JRL_SHADOW_HALF_FLOATS; ++i)
        {
            const unsigned index = half * JRL_SHADOW_HALF_FLOATS + i;
            jrl_pf32(out, index < RL_OBS_SIZE ? sample->obs[index] : sample->history[index - RL_OBS_SIZE]);
            out += 4;
        }
        length += jrl_finish(frame + length);
    }
    return length;
}
#endif

void Jetson_Usb_Record_Pump(void)
{
#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
    Shadow_Read_Reply();
#endif
    if (!CDC_Transmit_Ready_HS()) { return; }
    const uint32_t available = head - tail;
    __DMB();
    const uint32_t count = available < BatchSize ? available : BatchSize;
    unsigned length = 0;
#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
    if (!shadow_outgoing_ready)
    {
        shadow_outgoing_ready = Jetson_Usb_Shadow_Take(&shadow_outgoing);
    }
    if (shadow_outgoing_ready) { length = Shadow_Encode_Input(tx, &shadow_outgoing); }
    memcpy(tx + length, shadow_result, shadow_result_length);
    length += shadow_result_length;
#endif
    for (uint32_t i = 0; i < count; ++i)
    {
#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
        if (shadow_outgoing_ready)
        {
            shadow_pending[shadow_outgoing.sequence % ShadowPendingCount] = shadow_outgoing;
            shadow_outgoing_ready = 0;
        }
        shadow_result_length = 0;
#endif
        const telemetry_sample_t *s = &queue[(tail + i) % QueueSize];
        uint8_t *frame = tx + length;
        jrl_header(frame, FrameType, PayloadSize, 0u, s->reset, s->sequence, s->time_us);
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
    if (length && CDC_Transmit_HS(tx, (uint16_t)length) == USBD_OK)
    {
        __DMB();
        tail += count;
    }
}
