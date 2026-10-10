#include "vofa_debug.h"
#include "robot_state.h"
#include "dm.h"
#include "air_detection.h"
#include "Vofa_send.h"
#include "mono_ns.h"

#include <string.h>

typedef struct {
    compensation_debug_t compensation;
    float command[DM_MOTOR_NUM];
    float feedback[DM_MOTOR_NUM];
    float length[2];
    air_detection_t air;
    float state[10];
    float target[10];
    float acceleration;
    float force_off;
    float force_on;
    uint32_t fault;
    uint16_t state_bits;
    uint8_t online_mask;
    uint8_t air_page;
    uint8_t air_control;
} vofa_sample_t;

static struct {
    vofa_sample_t sample;
    float frame[VOFA_MAX_CH];
    uint64_t last_sent_ns;
    uint32_t sent;
    uint32_t busy;
} vofa_service;

/* 短区间采集 */
static void Vofa_Debug_Read(vofa_sample_t *snapshot)
{
    uint8_t online_mask;
    uint16_t state_bits;
    uint8_t motor_index;

    taskENTER_CRITICAL();
    /* ch0 在线掩码 */
    online_mask  = imu_state.online ? 0x01u : 0x00u;
    online_mask |= rc_command.online ? 0x02u : 0x00u;
    online_mask |= motor_state.dm.online[0] ? 0x04u : 0x00u;
    online_mask |= motor_state.dm.online[1] ? 0x08u : 0x00u;
    online_mask |= motor_state.dm.online[2] ? 0x10u : 0x00u;
    online_mask |= motor_state.dm.online[3] ? 0x20u : 0x00u;
    online_mask |= motor_state.dji.online[0] ? 0x40u : 0x00u;
    online_mask |= motor_state.dji.online[1] ? 0x80u : 0x00u;
    snapshot->online_mask = online_mask;

    /* ch1 状态位: 使能/跌倒/左腿有效/右腿有效/四髋使能/已投入 */
    state_bits  = robot_state.motor_enabled ? 0x01u : 0x00u;
    state_bits |= robot_state.fallen ? 0x02u : 0x00u;
    state_bits |= leg_l.output.valid ? 0x04u : 0x00u;
    state_bits |= leg_r.output.valid ? 0x08u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_LFT) ? 0x10u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_LFT) ? 0x20u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_RGT) ? 0x40u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_RGT) ? 0x80u : 0x00u;
    state_bits |= output_task_lqr_engaged() ? 0x100u : 0x00u;
    state_bits |= output_task_rl_engaged() ? 0x200u : 0x00u;
    snapshot->state_bits = state_bits;
    snapshot->fault = ctrl_fault;


    memcpy(snapshot->state, lqr_state.x, sizeof(snapshot->state));
    memcpy(snapshot->target, lqr_state.target, sizeof(snapshot->target));
    snapshot->acceleration = lqr_state.a_fwd;
    snapshot->compensation = compensation_debug;
    for (motor_index = 0u; motor_index < DM_MOTOR_NUM; motor_index++)
    {
        snapshot->command[motor_index] = rl_output_dm_cmd_nm[motor_index];
        snapshot->feedback[motor_index] = motor_state.dm.trq_nm[motor_index];
    }
    snapshot->length[0] = leg_l.output.virtual_leg_length;
    snapshot->length[1] = leg_r.output.virtual_leg_length;
    snapshot->air = air_debug;
    snapshot->air_page = air_param.vofa_page;
    snapshot->air_control = air_param.control_enabled;
    snapshot->force_off = air_param.force_off;
    snapshot->force_on = air_param.force_on;
    taskEXIT_CRITICAL();
}

/* 原39通道 */
static uint8_t Vofa_Debug_Send(void)
{
    vofa_sample_t *snapshot = &vofa_service.sample;
    float *dbg = vofa_service.frame;
    uint8_t motor_index;
    uint8_t compensation_bits;

    Vofa_Debug_Read(snapshot);
    memset(dbg, 0, sizeof(vofa_service.frame));
    dbg[0] = (float)snapshot->online_mask;
    dbg[1] = (float)snapshot->state_bits;
    dbg[2] = (float)snapshot->fault;
    memcpy(&dbg[3], snapshot->state, sizeof(snapshot->state));
    memcpy(&dbg[13], snapshot->target, sizeof(snapshot->target));
    dbg[23] = snapshot->acceleration;
    for (motor_index = 0u; motor_index < 2u; motor_index++)
    {
        dbg[24u + motor_index] = snapshot->compensation.gravity.leg[motor_index].torque;
        dbg[26u + motor_index] = snapshot->compensation.gravity.leg[motor_index].world_angle;
    }
    for (motor_index = 0u; motor_index < DM_MOTOR_NUM; motor_index++)
    {
        dbg[28u + motor_index] = snapshot->command[motor_index];
        dbg[32u + motor_index] = snapshot->feedback[motor_index];
    }
    dbg[36] = snapshot->length[0];
    dbg[37] = snapshot->length[1];
    compensation_bits = snapshot->compensation.gravity.valid ? 0x01u : 0u;
    compensation_bits |= snapshot->compensation.bench ? 0x02u : 0u;
    compensation_bits |= snapshot->compensation.gravity.leg[0].clamped ? 0x04u : 0u;
    compensation_bits |= snapshot->compensation.gravity.leg[1].clamped ? 0x08u : 0u;
    compensation_bits |= snapshot->compensation.saturated ? 0x10u : 0u;
    dbg[38] = (float)compensation_bits;
    if (snapshot->air_page)
    {
        dbg[24] = snapshot->air.leg[0].force;
        dbg[25] = snapshot->air.leg[1].force;
        dbg[26] = snapshot->air.mean_force;
        dbg[27] = (float)snapshot->air.flight;
        dbg[28] = snapshot->air.elapsed;
        dbg[29] = snapshot->force_off;
        dbg[30] = snapshot->force_on;
        dbg[31] = (float)(snapshot->air.valid | (snapshot->air.fault << 1u)
            | (snapshot->air.applied << 2u) | (snapshot->air.flight << 3u)
            | (snapshot->air.armed << 4u));
        dbg[32] = snapshot->air.leg[0].motor_force;
        dbg[33] = snapshot->air.leg[1].motor_force;
        dbg[34] = snapshot->air.leg[0].pressure;
        dbg[35] = snapshot->air.leg[1].pressure;
        dbg[36] = snapshot->air.leg[0].wheel_acceleration;
        dbg[37] = snapshot->air.leg[1].wheel_acceleration;
        dbg[38] = (float)snapshot->air_control;
    }
    return Vofa_Send(dbg, VOFA_MAX_CH);
}


void Vofa_Debug_Tick(void)
{
    uint64_t now_ns;

    taskENTER_CRITICAL();
    (void)Vofa_Transport_Update((uint8_t)(!robot_state.motor_enabled));
    taskEXIT_CRITICAL();
    now_ns = Mono_Ns_Get();
    if (vofa_service.sent != 0u
        && now_ns - vofa_service.last_sent_ns < (uint64_t)MACHINE_VOFA_PERIOD_MS * 1000000u)
    {
        return;
    }
    if (!Vofa_Transport_Ready())
    {
        vofa_service.busy++;
        return;
    }
    if (Vofa_Debug_Send())
    {
        vofa_service.last_sent_ns = Mono_Ns_Get();
        vofa_service.sent++;
    }
}
