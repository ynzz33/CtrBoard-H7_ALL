#include "joint_sysid.h"
#include "robot_state.h"
#include "latency_state.h"
#include "mono_ns.h"
#include "jetson_usb.h"
#include "pid.h"
#include <math.h>

static uint8_t Map_Swing_Torque(float swing_nm,
                            const leg_state_t *leg, float torque_nm[2])
{
    const leg_output_t *state = &leg->output;
    float mapped_nm[2];

    torque_nm[0] = 0.0f;
    torque_nm[1] = 0.0f;
    if (!state->valid || !isfinite(swing_nm))
    {
        return 0u;
    }

    if (!Leg_Force_Map_Forward(leg, 0.0f, swing_nm, mapped_nm) ||
        !isfinite(mapped_nm[0]) || !isfinite(mapped_nm[1]))
    {
        return 0u;
    }

    torque_nm[0] = mapped_nm[0];
    torque_nm[1] = mapped_nm[1];
    return 1u;
}

enum {
    SysidPreMs = 500u,
    SysidPostMs = 500u
};
static const float sysid_speed_levels_rad_s[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
static const float sysid_speed_kp = 15.0f;
static const float sysid_speed_ki = 1.0f;
static const float sysid_speed_kd = 1.0f;
static pid_t sysid_speed_pd;
static uint8_t sysid_level;
static uint32_t sysid_start_ms;
static uint32_t sysid_trial;
static uint8_t sysid_selected, sysid_failed, sysid_armed;
static uint8_t sysid_started, sysid_complete, sysid_record_ending;
static uint32_t sysid_record_end_ms;
static float sysid_prev_angle, sysid_travel;
static float sysid_ref_nm;
static float sysid_speed_ref;

void Joint_Sysid_Update(const control_frame_t *frame, torque_output_t *torque)
{
    const leg_state_t *leg = &frame->right;
    const uint32_t now = HAL_GetTick();
    if (!sysid_selected)
    {
        if (!sysid_armed ||
            sysid_level >= sizeof(sysid_speed_levels_rad_s) / sizeof(sysid_speed_levels_rad_s[0]))
        {
            return;
        }
        sysid_selected = 1u;
        sysid_failed = 0u;
        sysid_armed = 0u;
        sysid_start_ms = now;
        sysid_started = 0u;
        sysid_complete = 0u;
        sysid_record_ending = 0u;
        sysid_travel = 0.0f;
        sysid_speed_pd.iout = 0.0f;
        PID_struct_init(&sysid_speed_pd, POSITION_PID, 0.0f, 0.0f,
                        sysid_speed_kp, sysid_speed_ki, sysid_speed_kd);
        ++sysid_trial;
    }
    sysid_ref_nm = 0.0f;
    sysid_speed_ref = 0.0f;
    const uint32_t elapsed = now - sysid_start_ms;
    if (sysid_failed || sysid_complete || elapsed < SysidPreMs) { return; }
    if (!frame->drive_permit || !frame->joint_ready || !frame->motor_valid ||
        !frame->rc.online || !torque_output_enabled || ctrl_fault != FAULT_NONE)
    {
        sysid_failed = 1u;
        return;
    }
    float unit_map[2];
    const float angle = leg->output.virtual_leg_angle;
    const float speed = leg->output.d_virtual_leg_angle;
    if (!isfinite(angle) || !isfinite(speed) || !Map_Swing_Torque(1.0f, leg, unit_map))
    {
        sysid_failed = 1u;
        return;
    }
    if (!sysid_started)
    {
        sysid_started = 1u;
        sysid_prev_angle = angle;
        PID_History_Set(&sysid_speed_pd, -speed);
    }
    sysid_travel += Angle_Wrap_Pi(angle - sysid_prev_angle);
    sysid_prev_angle = angle;
    const float progress = fabsf(sysid_travel);
    if (progress >= 3.0f * LEG_2PI)
    {
        sysid_complete = 1u;
        return;
    }
    const float map_peak = fmaxf(fabsf(unit_map[0]), fabsf(unit_map[1]));
    if (!(map_peak > 0.0f))
    {
        sysid_failed = 1u;
        return;
    }
    const float limit = machine->hip.dm_trq_max / map_peak;
    const float ramp = fminf((float)(elapsed - SysidPreMs) / 500.0f, 1.0f);
    sysid_speed_ref = ramp * sysid_speed_levels_rad_s[sysid_level];
    sysid_speed_pd.MaxOutput = limit;
    sysid_speed_pd.IntegralLimit = limit;
    sysid_ref_nm = pid_calc(&sysid_speed_pd, speed, sysid_speed_ref, MACHINE_TICK_DT);
    torque->dm[DM_MOTOR_LEG_F_RGT] = unit_map[0] * sysid_ref_nm;
    torque->dm[DM_MOTOR_LEG_B_RGT] = unit_map[1] * sysid_ref_nm;
    torque->valid = 1u;
}

void Joint_Sysid_Record(const control_frame_t *frame,
                                  const torque_output_t *requested,
                                  const torque_output_t *applied)
{
    if (!sysid_selected) { return; }
    const uint32_t now_ms = HAL_GetTick();
    if (!output_debug_dm_sent || (requested->valid && !applied->valid))
    {
        sysid_failed = 1u;
    }
    if ((sysid_failed || sysid_complete) && !sysid_record_ending)
    {
        sysid_record_ending = 1u;
        sysid_record_end_ms = now_ms;
    }
    if (sysid_record_ending && now_ms - sysid_record_end_ms >= SysidPostMs) { return; }
    const leg_output_t *leg = &frame->right.output;
    const uint64_t now_ns = Mono_Ns_Get();
    const uint32_t status[6] = {
        (leg->valid ? 1u : 0u) | (frame->motor_valid ? 2u : 0u) |
        (output_debug_dm_sent ? 4u : 0u) | (sysid_failed ? 8u : 0u) |
        (applied->valid ? 16u : 0u) | (sysid_ref_nm != 0.0f ? 32u : 0u) |
        (sysid_complete ? 64u : 0u),
        0u, 2u | ((uint32_t)(sysid_level + 1u) << 8) |
        (1u << 16), ctrl_fault,
        Latency_Age_Us(now_ns, frame->joint_rx_ns[DM_MOTOR_LEG_F_RGT]),
        Latency_Age_Us(now_ns, frame->joint_rx_ns[DM_MOTOR_LEG_B_RGT])
    };
    const float values[JRL_SWING_FLOATS] = {
        sysid_ref_nm, 0.0f, frame->motor_pos[DM_MOTOR_LEG_F_RGT], frame->motor_pos[DM_MOTOR_LEG_B_RGT],
        frame->motor_vel[DM_MOTOR_LEG_F_RGT], frame->motor_vel[DM_MOTOR_LEG_B_RGT], frame->joint_torque[DM_MOTOR_LEG_F_RGT], frame->joint_torque[DM_MOTOR_LEG_B_RGT],
        requested->dm[DM_MOTOR_LEG_F_RGT], requested->dm[DM_MOTOR_LEG_B_RGT], requested->dm[DM_MOTOR_LEG_F_RGT], requested->dm[DM_MOTOR_LEG_B_RGT],
        applied->dm[DM_MOTOR_LEG_F_RGT], applied->dm[DM_MOTOR_LEG_B_RGT], leg->virtual_leg_length, leg->d_virtual_leg_length,
        leg->virtual_leg_angle, leg->d_virtual_leg_angle, sysid_speed_ref
    };
    Jetson_Usb_Record_Swing(sysid_trial, status, values);
}

void Joint_Sysid_Update_Mode(const control_frame_t *frame, ctrl_strategy_t strategy)
{
    if (strategy != CTRL_STRATEGY_JOINT_SYSID)
    {
        if (sysid_selected && sysid_record_ending &&
            HAL_GetTick() - sysid_record_end_ms >= SysidPostMs &&
            sysid_complete && !sysid_failed)
        {
            ++sysid_level;
        }
        sysid_selected = 0u;
        sysid_armed = frame->rc.online;
    }
}
