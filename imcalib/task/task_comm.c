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

static void Robot_Control_Send_Vofa(void)
{
    static uint8_t vofa_div;
    static float dbg[VOFA_MAX_CH];
    const pid_t *pid;
    uint8_t online_mask;
    uint8_t kind;
    uint16_t seq;
    uint64_t tx_ns;

#if SYSID_ENABLE
    /* 轮测试模式: 最简 10 通道, 每个周期都发 (1kHz) */
    extern const uint8_t sysid_is_wheel_mode;
    if (sysid_is_wheel_mode && (ctrl_strategy == CTRL_STRATEGY_SYSID))
    {
        extern volatile int16_t sysid_wheel_cmd_raw[DJI_MOTOR_NUM];
        static uint64_t tx_last_ns;
        uint8_t m;
        uint8_t k;
        uint16_t s;
        uint64_t t;

        m  = imu_state.online ? 0x01u : 0x00u;
        m |= DR16_Online() ? 0x02u : 0x00u;
        m |= motor_state.dm.online[0] ? 0x04u : 0x00u;
        m |= motor_state.dm.online[1] ? 0x08u : 0x00u;
        m |= motor_state.dm.online[2] ? 0x10u : 0x00u;
        m |= motor_state.dm.online[3] ? 0x20u : 0x00u;
        m |= motor_state.dji.online[0] ? 0x40u : 0x00u;
        m |= motor_state.dji.online[1] ? 0x80u : 0x00u;

        while (Can_Bus_Tx_Pop((uint8_t)machine->dji_bus, &k, &s, &t))
        {
            tx_last_ns = t;
        }

        dbg[0] = (float)m;
        dbg[1] = (float)sysid_wheel_cmd_raw[DJI_MOTOR_WHEEL_LFT];
        dbg[2] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].current_raw;
        dbg[3] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].current_raw;
        dbg[4] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].angle_raw;
        dbg[5] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].angle_raw;
        dbg[6] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].vel_raw
                 * (0.1047198f / machine->dji_gear_ratio);
        dbg[7] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].vel_raw
                 * (0.1047198f / machine->dji_gear_ratio);
        dbg[8] = (float)(uint32_t)(tx_last_ns / 1000u);
        dbg[9] = (float)(uint32_t)(dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].rx_ns
                                   / 1000u);

        Vofa_Send(dbg, 10u);
        return;
    }
#endif

    vofa_div++;
    if (vofa_div < 5u)
    {
        return;
    }
    vofa_div = 0u;

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

    /* 轮测试: ch13~19 左轮 / ch20~26 右轮
       命令raw / 命令A / 实测raw / 实测A / 编码器 / 转速rpm / 转速rad-s */
    extern volatile int16_t sysid_wheel_cmd_raw[DJI_MOTOR_NUM];
#if SYSID_ENABLE
    dbg[13] = (float)sysid_wheel_cmd_raw[DJI_MOTOR_WHEEL_LFT];
#else
    dbg[13] = 0.0f;
#endif
    dbg[14] = dbg[13] / 819.2f;
    dbg[15] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].current_raw;
    dbg[16] = dbg[15] / 819.2f;
    dbg[17] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].angle_raw;
    dbg[18] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].vel_raw;
    dbg[19] = dbg[18] * (0.1047198f / machine->dji_gear_ratio);

#if SYSID_ENABLE
    dbg[20] = (float)sysid_wheel_cmd_raw[DJI_MOTOR_WHEEL_RGT];
#else
    dbg[20] = 0.0f;
#endif
    dbg[21] = dbg[20] / 819.2f;
    dbg[22] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].current_raw;
    dbg[23] = dbg[22] / 819.2f;
    dbg[24] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].angle_raw;
    dbg[25] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].vel_raw;
    dbg[26] = dbg[25] * (0.1047198f / machine->dji_gear_ratio);

    /* ch27~31 腿部解算值 (从原 ch19~24 挪过来) */
    dbg[27] = leg_l.output.virtual_shank_angle;
    dbg[28] = leg_l.output.thigh_angle;
    dbg[29] = leg_l.output.virtual_leg_length;
    dbg[30] = leg_r.output.thigh_angle;
    dbg[31] = leg_r.output.virtual_leg_length;
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
