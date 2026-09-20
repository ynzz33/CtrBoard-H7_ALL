#include "robot_control.h"
#include "Attitude_Algorithm.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"
#include "Vofa_send.h"
#include "ws2812.h"
#include "../Sysid/sysid_config.h"
#if SYSID_ENABLE
#include "../Sysid/sysid_log.h"
#endif

#include <math.h>

static leg_debug_history_t leg_debug_l;
static leg_debug_history_t leg_debug_r;

static float Leg_Debug_Angle_Diff(float current, float previous)
{
    float diff;

    diff = current - previous;
    while (diff > 3.14159265358979f)
    {
        diff -= 6.28318530717959f;
    }
    while (diff < -3.14159265358979f)
    {
        diff += 6.28318530717959f;
    }
    return diff;
}

/* 速度自检 */
static void Leg_Debug_Validate(const leg_state_t *leg, leg_debug_history_t *history)
{
    float dt_s;
    uint32_t now_ms;

    if (leg == NULL || history == NULL || !leg->output.valid)
    {
        if (history != NULL)
        {
            history->ready = 0u;
        }
        return;
    }
    now_ms = HAL_GetTick();
    if (!history->ready)
    {
        history->virtual_leg_length = leg->output.virtual_leg_length;
        history->virtual_leg_angle = leg->output.virtual_leg_angle;
        history->virtual_shank_angle = leg->output.virtual_shank_angle;
        history->tick_ms = now_ms;
        history->ready = 1u;
        return;
    }
    if (now_ms == history->tick_ms)
    {
        return;
    }

    dt_s = (float)(now_ms - history->tick_ms) * 0.001f;
    history->measured[0] = (leg->output.virtual_leg_length - history->virtual_leg_length) / dt_s;
    history->measured[1] = Leg_Debug_Angle_Diff(leg->output.virtual_leg_angle, history->virtual_leg_angle) / dt_s;
    history->measured[2] = Leg_Debug_Angle_Diff(leg->output.virtual_shank_angle, history->virtual_shank_angle) / dt_s;
    history->predicted[0] = leg->output.d_virtual_leg_length;
    history->predicted[1] = leg->output.d_virtual_leg_angle;
    history->predicted[2] = leg->output.d_virtual_shank_angle;
    history->residual[0] = history->predicted[0] - history->measured[0];
    history->residual[1] = history->predicted[1] - history->measured[1];
    history->residual[2] = history->predicted[2] - history->measured[2];
    history->virtual_leg_length = leg->output.virtual_leg_length;
    history->virtual_leg_angle = leg->output.virtual_leg_angle;
    history->virtual_shank_angle = leg->output.virtual_shank_angle;
    history->tick_ms = now_ms;
}

/* 更新电机状态 */
static void Motor_State_Update(void)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];

        motor_state.dm.pos_rad[i] = feedback->pos_rad;
        motor_state.dm.pos_zero_rad[i] = feedback->pos_zero_rad;
        motor_state.dm.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dm.trq_nm[i] = feedback->trq_nm;
        motor_state.dm.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

        motor_state.dji.angle_rad[i] = feedback->angle_rad;
        motor_state.dji.angle_total_rad[i] = feedback->angle_total_rad;
        motor_state.dji.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dji.current_raw[i] = feedback->current_raw;
        motor_state.dji.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dji.online[i] = (uint8_t)Dji_Is_Online(i);
    }
    motor_state.timestamp_ms = HAL_GetTick();
    motor_state.updated = 1u;
}

/* 更新腿部状态 */
static void Leg_State_Update(void)
{
    if (leg_map_l.configured)
    {
        leg_l.input.hip_f = motor_state.dm.pos_zero_rad[leg_map_l.dm_front] + LEG_PI;
        leg_l.input.hip_b = motor_state.dm.pos_zero_rad[leg_map_l.dm_rear];
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_zero_rad[leg_map_r.dm_front] + LEG_PI;
        leg_r.input.hip_b = motor_state.dm.pos_zero_rad[leg_map_r.dm_rear];
        leg_r.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_r.dm_front];
        leg_r.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_r.dm_rear];
    }
    (void)Leg_Solve(&leg_l);
    (void)Leg_Solve(&leg_r);
    Leg_Debug_Validate(&leg_l, &leg_debug_l);
    Leg_Debug_Validate(&leg_r, &leg_debug_r);
}

/* 更新遥控使能 (指令由 policyTask 统一处理) */
static void Remote_Control_Update(void)
{
    dr16_t remote;

    DR16_Process();
    remote = DR16_Snapshot();
    robot_state.rc_enable = (uint8_t)(remote.online && remote.s1 != DR16_SW_DOWN);
    input_command.mode = remote.online ? remote.s1 : 0u;
}

/* 更新故障状态 */
static void Robot_Fault_Update(void)
{
    uint32_t fault;
    uint8_t motors_ok;

    fault = FAULT_NONE;
    motors_ok = 1u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i])
        {
            motors_ok = 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i])
        {
            motors_ok = 0u;
        }
    }
    if (!imu_state.online)
    {
        fault |= FAULT_IMU;
    }
    if (!DR16_Online())
    {
        fault |= FAULT_RC;
    }
    if (!Can_Bus_Online(true))
    {
        fault |= FAULT_CAN;
    }
    if (!motors_ok)
    {
        fault |= FAULT_MOTOR;
    }
    if (robot_state.rc_enable
        && (!action_state.updated || HAL_GetTick() - action_state.last_ok_tick >= 100u))
    {
        fault |= FAULT_ACTION;
    }
    ctrl_fault = fault;
}

/* 更新翻倒状态 */
static void Robot_Fallen_Update(void)
{
    float pitch_abs;

    pitch_abs = fabsf(imu_state.euler_rad[ATTITUDE_PITCH]);
    if (pitch_abs > 1.4f)
    {
        robot_state.fallen = 1u;
    }
    else if (pitch_abs < 1.0f)
    {
        robot_state.fallen = 0u;
    }
}

/* 更新使能状态 */
static void Robot_Enable_Update(void)
{
    uint8_t enable_request;

    enable_request = (uint8_t)(robot_state.rc_enable
        && ctrl_fault == FAULT_NONE && !robot_state.fallen);
    if (enable_request && !robot_state.motor_enabled)
    {
        robot_state.motor_enabled = 1u;
        (void)Dm_All_Enable();
    }
    else if (!enable_request && robot_state.motor_enabled)
    {
        robot_state.motor_enabled = 0u;
        (void)Dji_All_Stop();
        (void)Dm_All_Disable();
    }
}

/*
 * VOFA 观测帧 (JustFloat, 32 通道, 200Hz)
 * ch0  在线掩码: bit0 IMU / bit1 遥控 / bit2~5 髋(前左后左前右后右) / bit6~7 轮(左/右)
 * ch1  状态位: bit0 使能 / bit1 跌倒 / bit2 左腿有效 / bit3 右腿有效
 * ch2  策略号: 0 手动 / 1 LQR
 * ch3~6   四髋位置 (零点后 rad): 前左 / 后左 / 前右 / 后右
 * ch7~10  左腿: 大腿角 / 虚拟小腿角 / 虚拟腿摆角 / 腿长 (rad, m)
 * ch11~14 右腿: 大腿角 / 虚拟小腿角 / 虚拟腿摆角 / 腿长 (rad, m)
 * ch15~16 腿长目标 (左 / 右 m)
 * ch17~19 俯仰角 (rad) / 俯仰角速度 (rad/s) / 前进速度 (m/s)
 * ch20~23 LQR 输出 (N·m): 左轮 / 右轮 / 左髋 / 右髋
 * ch24~25 轮转速 (rad/s): 左 / 右
 * ch26~27 轮实测电流 (A): 左 / 右
 * ch28~31 髋力矩反馈 (N·m): 前左 / 后左 / 前右 / 后右
 */
static void Robot_Control_Send_Vofa(void)
{
    static uint8_t vofa_div;
    static float dbg[VOFA_MAX_CH];
    uint8_t online_mask;
    uint8_t state_bits;
    uint8_t kind;
    uint16_t seq;
    uint64_t tx_ns;
    /* ch0 在线掩码 */
    online_mask  = imu_state.online ? 0x01u : 0x00u;
    online_mask |= DR16_Online() ? 0x02u : 0x00u;
    online_mask |= motor_state.dm.online[0] ? 0x04u : 0x00u;
    online_mask |= motor_state.dm.online[1] ? 0x08u : 0x00u;
    online_mask |= motor_state.dm.online[2] ? 0x10u : 0x00u;
    online_mask |= motor_state.dm.online[3] ? 0x20u : 0x00u;
    online_mask |= motor_state.dji.online[0] ? 0x40u : 0x00u;
    online_mask |= motor_state.dji.online[1] ? 0x80u : 0x00u;
    dbg[0] = (float)online_mask;

    /* ch1 状态位: 使能/跌倒/左腿有效/右腿有效 */
    state_bits  = robot_state.motor_enabled ? 0x01u : 0x00u;
    state_bits |= robot_state.fallen ? 0x02u : 0x00u;
    state_bits |= leg_l.output.valid ? 0x04u : 0x00u;
    state_bits |= leg_r.output.valid ? 0x08u : 0x00u;
    dbg[1] = (float)state_bits;

    /* ch2 策略号: 0 手动 / 1 LQR */
    dbg[2] = (float)ctrl_strategy;

    /* ch3~6 四髋位置 (零点后 rad) */
    dbg[3] = motor_state.dm.pos_zero_rad[DM_MOTOR_LEG_F_LFT];
    dbg[4] = motor_state.dm.pos_zero_rad[DM_MOTOR_LEG_B_LFT];
    dbg[5] = motor_state.dm.pos_zero_rad[DM_MOTOR_LEG_F_RGT];
    dbg[6] = motor_state.dm.pos_zero_rad[DM_MOTOR_LEG_B_RGT];

    /* ch7~10 左腿解算: 大腿角/小腿角/摆角/腿长 */
    dbg[7]  = leg_l.output.thigh_angle;
    dbg[8]  = leg_l.output.virtual_shank_angle;
    dbg[9]  = leg_l.output.virtual_leg_angle;
    dbg[10] = leg_l.output.virtual_leg_length;

    /* ch11~14 右腿解算 */
    dbg[11] = leg_r.output.thigh_angle;
    dbg[12] = leg_r.output.virtual_shank_angle;
    dbg[13] = leg_r.output.virtual_leg_angle;
    dbg[14] = leg_r.output.virtual_leg_length;

    /* ch15~16 腿长目标 (左/右 m) */
    dbg[15] = lqr_state.leg_len_tgt[0];
    dbg[16] = lqr_state.leg_len_tgt[1];

    /* ch17~19 俯仰角/俯仰角速度/前进速度 */
    dbg[17] = lqr_state.x[LQR_X_THB];
    dbg[18] = lqr_state.x[LQR_X_DTHB];
    dbg[19] = lqr_state.x[LQR_X_DS];

    /* ch20~23 LQR 输出: 左轮/右轮/左髋/右髋 (N·m) */
    dbg[20] = lqr_state.u[LQR_U_WL];
    dbg[21] = lqr_state.u[LQR_U_WR];
    dbg[22] = lqr_state.u[LQR_U_BL];
    dbg[23] = lqr_state.u[LQR_U_BR];

    /* ch24~25 轮转速 (rad/s) */
    dbg[24] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    dbg[25] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];

    /* ch26~27 轮实测电流 (A) */
    dbg[26] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].current_raw / 819.2f;
    dbg[27] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].current_raw / 819.2f;

    /* ch28~31 髋力矩反馈 (N·m): 左前/左后/右前/右后 */
    dbg[28] = dm_motor_feedback[DM_MOTOR_LEG_F_LFT].trq_nm;
    dbg[29] = dm_motor_feedback[DM_MOTOR_LEG_B_LFT].trq_nm;
    dbg[30] = dm_motor_feedback[DM_MOTOR_LEG_F_RGT].trq_nm;
    dbg[31] = dm_motor_feedback[DM_MOTOR_LEG_B_RGT].trq_nm;
    /* 清空 CAN 发送完成环 (不进通道, 防止积满) */
    while (Can_Bus_Tx_Pop((uint8_t)machine->dji_bus, &kind, &seq, &tx_ns)) 
    {
    }
    while (Can_Bus_Tx_Pop((uint8_t)machine->dm_bus[0], &kind, &seq, &tx_ns))
    {
    }

    Vofa_Send(dbg, 32u);
}

/* 通信单周期 */
void comm_task_body(void)
{
    Dm_Parse();
    Dji_Parse();
    Motor_State_Update();
    Leg_State_Update();
    WS2812_RainbowBlink();
    Remote_Control_Update();
    Robot_Fallen_Update();
    Robot_Fault_Update();
    Robot_Enable_Update();
    Robot_Control_Send_Vofa();
}
