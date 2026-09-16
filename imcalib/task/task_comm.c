#include "robot_control.h"
#include "Attitude_Algorithm.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "Vofa_send.h"
#include "ws2812.h"

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
        leg_l.input.hip_f = motor_state.dm.pos_rad[leg_map_l.dm_front]
            + LEG_PI + leg_l.config.offset_f;
        leg_l.input.hip_b = motor_state.dm.pos_rad[leg_map_l.dm_rear] + leg_l.config.offset_b;
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_rad[leg_map_r.dm_front]
            + LEG_PI + leg_r.config.offset_f;
        leg_r.input.hip_b = motor_state.dm.pos_rad[leg_map_r.dm_rear] + leg_r.config.offset_b;
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

/* 发送调试数据 */
static void Robot_Control_Send_Vofa(void)
{
    static uint8_t vofa_div;
    static float dbg[48];
    uint8_t online_mask;

    vofa_div++;
    if (vofa_div < 5u)
    {
        return;
    }
    vofa_div = 0u;

    /* 在线掩码 (始终 dbg[0]) */
    online_mask  = imu_state.online ? 0x01u : 0x00u;
    online_mask |= DR16_Online()    ? 0x02u : 0x00u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        online_mask |= motor_state.dm.online[i] ? (uint8_t)(0x04u << i) : 0x00u;
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        online_mask |= motor_state.dji.online[i] ? (uint8_t)(0x40u << i) : 0x00u;
    }
    dbg[0]  = (float)online_mask;

    /* 左腿 */
    dbg[1]  = motor_state.dm.pos_rad[0];           /* F_LFT 电机角 */
    dbg[2]  = motor_state.dm.pos_rad[1];           /* B_LFT 电机角 */
    dbg[3]  = leg_l.input.hip_f;                   /* 前髋角 */
    dbg[4]  = leg_l.input.hip_b;                   /* 后髋角 */
    dbg[5]  = leg_l.output.thigh_angle;              /* 大腿角 */
    dbg[6]  = leg_l.output.virtual_leg_length;       /* 虚拟腿长 */
    dbg[7]  = leg_l.output.virtual_leg_angle;        /* 虚拟腿摆角 */
    dbg[8]  = leg_l.output.virtual_shank_angle;      /* 虚拟小腿角 */
    dbg[9]  = leg_l.output.d_virtual_shank_angle;    /* 虚拟小腿角速度 */
    dbg[10] = leg_l.output.vshank_jac[0];            /* jac_b */
    dbg[11] = leg_l.output.vshank_jac[1];            /* jac_a */
    /* 右腿 */
    dbg[12] = motor_state.dm.pos_rad[2];           /* F_RGT 电机角 */
    dbg[13] = motor_state.dm.pos_rad[3];           /* B_RGT 电机角 */
    dbg[14] = leg_r.input.hip_f;
    dbg[15] = leg_r.input.hip_b;
    dbg[16] = leg_r.output.thigh_angle;
    dbg[17] = leg_r.output.virtual_leg_length;
    dbg[18] = leg_r.output.virtual_leg_angle;
    dbg[19] = leg_r.output.virtual_shank_angle;
    dbg[20] = leg_r.output.d_virtual_shank_angle;
    dbg[21] = leg_r.output.vshank_jac[0];
    dbg[22] = leg_r.output.vshank_jac[1];
    /* 状态 */
    dbg[23] = imu_state.euler_rad[1];              /* pitch */
    dbg[24] = (float)leg_l.output.valid;
    dbg[25] = (float)leg_r.output.valid;
    /* 力矩调试链路 */
    dbg[26] = input_command.vx_cmd;                  /* 遥控 vx */
    dbg[27] = input_command.height_cmd;              /* 遥控 height */
    dbg[28] = action_state.a[0];                     /* action 左大腿 */
    dbg[29] = action_state.a[1];                     /* action 左小腿 */
    dbg[30] = rl_control.torque_state.virtual_torque[0]; /* tau_v 左大腿 */
    dbg[31] = rl_control.torque_state.virtual_torque[1]; /* tau_v 左小腿 */
    dbg[32] = leg_l.input.hip_f;                          /* 当前大腿角 */
    dbg[33] = leg_l.output.virtual_shank_angle;           /* 当前虚拟小腿角 */
    dbg[34] = (float)action_state.base_action_locked;    /* 锁存标志 */
    dbg[35] = (float)robot_state.motor_enabled;          /* 使能标志 */
    /* PID debug: 左腿 */
    dbg[36] = rl_control.torque_state.pid_target[0];     /* L_thigh target */
    dbg[37] = rl_control.torque_state.pid_err[0];        /* L_thigh error */
    dbg[38] = rl_control.torque_state.pid_output[0];     /* L_thigh tau_v */
    dbg[39] = rl_control.torque_state.pid_target[1];     /* L_shank target */
    dbg[40] = rl_control.torque_state.pid_err[1];        /* L_shank error */
    dbg[41] = rl_control.torque_state.pid_output[1];     /* L_shank tau_v */
    Vofa_Send(dbg, 42u);
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
