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
 * 通道表 (32 通道 FireWater, 200 Hz)
 *
 *  ch  含义                       单位 / 来源
 *  --  -------------------------  -----------------------------------
 *   0  在线掩码                   bit0=IMU bit1=RC bit2~5=DM bit6~7=DJI
 *   1  解算有效                   1=左 2=右 3=双
 *   2  策略号                     0=手动 1=LQR 2=测试
 *   3  腿指令力矩 前左            Nm, last_torque.dm[0]
 *   4  腿指令力矩 后左            Nm, last_torque.dm[1]
 *   5  腿指令力矩 前右            Nm, last_torque.dm[2]
 *   6  腿指令力矩 后右            Nm, last_torque.dm[3]
 *   7  腿零点后角 前左            rad, dm.pos_zero_rad[0]
 *   8  腿零点后角 后左            rad, dm.pos_zero_rad[1]
 *   9  腿零点后角 前右            rad, dm.pos_zero_rad[2]
 *  10  腿零点后角 后右            rad, dm.pos_zero_rad[3]
 *  11  左腿长                     m, leg_l.output.virtual_leg_length
 *  12  右腿长                     m, leg_r.output.virtual_leg_length
 *  13  左腿摆角                   rad, leg_l.output.virtual_leg_angle
 *  14  右腿摆角                   rad, leg_r.output.virtual_leg_angle
 *  15  轮指令电流 左              raw, Dji_Torque_To_Current
 *  16  轮指令电流 右              raw
 *  17  轮转速 左                  rad/s, dji.vel_rad_s[0]
 *  18  轮转速 右                  rad/s
 *  19  轮编码器 左                raw, dji.angle_raw[0]
 *  20  轮编码器 右                raw
 *  21  轮实际电流 左              raw, dji.current_raw[0]
 *  22  轮实际电流 右              raw
 *  23  左轮温度                   temp_raw
 *  24  轮总线 TX 间隔             us
 *  25  轮 RX 间隔                 us
 *  26  腿总线 TX 间隔             us
 *  27  轮总线 TX 条数             本周期
 *  28  腿总线 TX 条数             本周期
 *  29  轮总线 TX 丢帧             累计
 *  30  rx_ns 高位                 rx_ns >> 20
 *  31  rx_ns 低位                 rx_ns & 0xFFFFF
 */
static void Robot_Control_Send_Vofa(void)
{
    static uint8_t vofa_div;
    static float dbg[VOFA_MAX_CH];
    const pid_t *pid;
    uint8_t online_mask;
    uint8_t kind;
    uint16_t seq;
    uint64_t tx_ns;

    vofa_div++;
    if (vofa_div < 5u)
    {
        return;
    }
    vofa_div = 0u;

#if SYSID_ENABLE
    if (ctrl_strategy == CTRL_STRATEGY_SYSID)
    {
        return;
    }
#endif

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

    /* ch1~3 左前髋: 大腿角目标 / 当前 / 输出力矩 */
    pid    = &rl_control.torque_state.controller[0];    /* 0=左大腿 */
    dbg[1] = pid->set[NOW];
    dbg[2] = pid->get[NOW];
    dbg[3] = rl_control.torque_state.last_torque.dm[DM_MOTOR_LEG_F_LFT];

    /* ch4~6 左后髋: 小腿角目标 / 当前 / 输出力矩 */
    pid    = &rl_control.torque_state.controller[1];    /* 1=左小腿 */
    dbg[4] = pid->set[NOW];
    dbg[5] = pid->get[NOW];
    dbg[6] = rl_control.torque_state.last_torque.dm[DM_MOTOR_LEG_B_LFT];

    /* ch7~9 右前髋: 大腿角目标 / 当前 / 输出力矩 */
    pid    = &rl_control.torque_state.controller[3];    /* 3=右大腿 */
    dbg[7] = pid->set[NOW];
    dbg[8] = pid->get[NOW];
    dbg[9] = rl_control.torque_state.last_torque.dm[DM_MOTOR_LEG_F_RGT];

    /* ch10~12 右后髋: 小腿角目标 / 当前 / 输出力矩 */
    pid     = &rl_control.torque_state.controller[4];   /* 4=右小腿 */
    dbg[10] = pid->set[NOW];
    dbg[11] = pid->get[NOW];
    dbg[12] = rl_control.torque_state.last_torque.dm[DM_MOTOR_LEG_B_RGT];

    /* ch13~15 左轮: 速度目标 / 当前 / 输出电流 */
    pid     = &rl_control.torque_state.controller[2];   /* 2=左轮 */
    dbg[13] = pid->set[NOW];
    dbg[14] = pid->get[NOW];
    dbg[15] = (float)Dji_Torque_To_Current(DJI_MOTOR_WHEEL_LFT,
        rl_control.torque_state.last_torque.dji[DJI_MOTOR_WHEEL_LFT]
        * (float)machine->dji_sign[DJI_MOTOR_WHEEL_LFT].out);

    /* ch16~18 右轮: 速度目标 / 当前 / 输出电流 */
    pid     = &rl_control.torque_state.controller[5];   /* 5=右轮 */
    dbg[16] = pid->set[NOW];
    dbg[17] = pid->get[NOW];
    dbg[18] = (float)Dji_Torque_To_Current(DJI_MOTOR_WHEEL_RGT,
        rl_control.torque_state.last_torque.dji[DJI_MOTOR_WHEEL_RGT]
        * (float)machine->dji_sign[DJI_MOTOR_WHEEL_RGT].out);

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
#if SYSID_ENABLE
    if (ctrl_strategy == CTRL_STRATEGY_SYSID)
    {
        (void)Sysid_Log_Send_Pump();
    }
    else
#endif
    {
        Robot_Control_Send_Vofa();
    }
}
