#include "jetson_usb.h"
#include "jetson_protocol.h"
#include "robot_state.h"
#include "mono_ns.h"
#include "usbd_cdc_if.h"
#include "dma_cache.h"

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
static uint8_t tx[BatchSize * (JRL_HEADER + PayloadSize + 4)]
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

void Jetson_Usb_Record_Pump(void)
{
    if (!CDC_Transmit_Ready_HS()) { return; }
    const uint32_t available = head - tail;
    __DMB();
    const uint32_t count = available < BatchSize ? available : BatchSize;
    unsigned length = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
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
