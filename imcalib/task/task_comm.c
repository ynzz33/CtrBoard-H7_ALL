#include "robot_control.h"
#include "Attitude_Algorithm.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "Vofa_send.h"
#include "ws2812.h"

#include <math.h>

#define LEG_GEOMETRY_PI      3.14159265358979f
#define REMOTE_COMMAND_SCALE 0.1f

typedef struct {
    float l0;
    float phi0;
    float virtual_shank;
    float measured[3];
    float predicted[3];
    float residual[3];
    uint32_t tick_ms;
    uint8_t ready;
} leg_debug_history_t;

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
        history->l0 = leg->output.l0;
        history->phi0 = leg->output.phi0;
        history->virtual_shank = leg->output.virtual_shank;
        history->tick_ms = now_ms;
        history->ready = 1u;
        return;
    }
    if (now_ms == history->tick_ms)
    {
        return;
    }

    dt_s = (float)(now_ms - history->tick_ms) * 0.001f;
    history->measured[0] = (leg->output.l0 - history->l0) / dt_s;
    history->measured[1] = Leg_Debug_Angle_Diff(leg->output.phi0, history->phi0) / dt_s;
    history->measured[2] = Leg_Debug_Angle_Diff(leg->output.virtual_shank, history->virtual_shank) / dt_s;
    history->predicted[0] = leg->output.dl0;
    history->predicted[1] = leg->output.dphi0;
    history->predicted[2] = leg->output.d_virtual_shank;
    history->residual[0] = history->predicted[0] - history->measured[0];
    history->residual[1] = history->predicted[1] - history->measured[1];
    history->residual[2] = history->predicted[2] - history->measured[2];
    history->l0 = leg->output.l0;
    history->phi0 = leg->output.phi0;
    history->virtual_shank = leg->output.virtual_shank;
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
            + LEG_GEOMETRY_PI + leg_l.config.offset_f;
        leg_l.input.hip_b = motor_state.dm.pos_rad[leg_map_l.dm_rear] + leg_l.config.offset_b;
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_rad[leg_map_r.dm_front]
            + LEG_GEOMETRY_PI + leg_r.config.offset_f;
        leg_r.input.hip_b = motor_state.dm.pos_rad[leg_map_r.dm_rear] + leg_r.config.offset_b;
        leg_r.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_r.dm_front];
        leg_r.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_r.dm_rear];
    }
    (void)Leg_Solve(&leg_l);
    (void)Leg_Solve(&leg_r);
    Leg_Debug_Validate(&leg_l, &leg_debug_l);
    Leg_Debug_Validate(&leg_r, &leg_debug_r);
}

/* 遥控死区 */
static int16_t DR16_Command_Axis(int16_t input)
{
    int16_t value;

    value = DR16_Deadline(input, 20u);
    if (value > DR16_CH_LIMIT)
    {
        value = DR16_CH_LIMIT;
    }
    else if (value < -DR16_CH_LIMIT)
    {
        value = -DR16_CH_LIMIT;
    }
    return value;
}

/* 更新遥控指令 */
static void Remote_Control_Update(void)
{
    dr16_t remote;

    DR16_Process();
    remote = DR16_Snapshot();
    robot_state.rc_enable = (uint8_t)(remote.online && remote.s1 != DR16_SW_DOWN);
    if (remote.online)
    {
        input_command.vx_cmd = (float)DR16_Command_Axis(remote.ch3)
            / (float)DR16_CH_LIMIT * REMOTE_COMMAND_SCALE;
        input_command.yaw_cmd = (float)DR16_Command_Axis(remote.ch0)
            / (float)DR16_CH_LIMIT * REMOTE_COMMAND_SCALE;
        input_command.height_cmd = (float)DR16_Command_Axis(remote.wheel)
            / (float)DR16_CH_LIMIT * REMOTE_COMMAND_SCALE;
        input_command.mode = remote.s1;
    }
    else
    {
        input_command.vx_cmd = 0.0f;
        input_command.yaw_cmd = 0.0f;
        input_command.height_cmd = 0.0f;
        input_command.mode = 0u;
    }
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

    pitch_abs = fabsf(imu_state.output.euler_rad[ATTITUDE_PITCH]);
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
    if (enable_request && !robot_state.enabled)
    {
        robot_state.enabled = 1u;
        (void)Dm_All_Enable();
    }
    else if (!enable_request && robot_state.enabled)
    {
        robot_state.enabled = 0u;
        (void)Dji_All_Stop();
        (void)Dm_All_Disable();
    }
}

/* 发送调试数据 */
static void Robot_Control_Send_Vofa(void)
{
    static uint8_t vofa_div;
    static float dbg[36];
    uint8_t motor_mask;

    vofa_div++;
    if (vofa_div < 5u)
    {
        return;
    }
    vofa_div = 0u;

    dbg[0] = leg_l.input.hip_f;
    dbg[1] = leg_l.output.virtual_shank;
    dbg[2] = leg_r.input.hip_f;
    dbg[3] = leg_r.output.virtual_shank;
    dbg[4] = rl_control.torque_state.controller[0].set[NOW];
    dbg[5] = rl_control.torque_state.controller[1].set[NOW];
    dbg[6] = rl_control.torque_state.controller[3].set[NOW];
    dbg[7] = rl_control.torque_state.controller[4].set[NOW];
    dbg[8] = rl_control.torque_state.controller[0].err[NOW];
    dbg[9] = rl_control.torque_state.controller[1].err[NOW];
    dbg[10] = rl_control.torque_state.controller[3].err[NOW];
    dbg[11] = rl_control.torque_state.controller[4].err[NOW];
    dbg[12] = rl_control.torque_state.controller[0].pout;
    dbg[13] = rl_control.torque_state.controller[1].pout;
    dbg[14] = rl_control.torque_state.controller[3].pout;
    dbg[15] = rl_control.torque_state.controller[4].pout;
    dbg[16] = leg_l.output.l0;
    dbg[17] = leg_r.output.l0;
    dbg[18] = rl_control.torque_state.controller[3].dout;
    dbg[19] = rl_control.torque_state.controller[4].dout;
    dbg[20] = rl_control.torque_state.virtual_torque[0];
    dbg[21] = rl_control.torque_state.virtual_torque[1];
    dbg[22] = rl_control.torque_state.virtual_torque[3];
    dbg[23] = rl_control.torque_state.virtual_torque[4];
    dbg[24] = leg_l.output.vshank_jac[0];
    dbg[25] = leg_l.output.vshank_jac[1];
    dbg[26] = rl_control.torque_state.last_torque[RL_TQ_L_THIGH];
    dbg[27] = rl_control.torque_state.last_torque[RL_TQ_L_SHANK];
    dbg[28] = rl_control.torque_state.last_torque[RL_TQ_R_THIGH];
    dbg[29] = rl_control.torque_state.last_torque[RL_TQ_R_SHANK];
    dbg[30] = leg_l.output.phi0;           /* 大腿方向角 */
    /* dbg[31] reserved for motor_mask */
    dbg[32] = leg_l.output.virtual_shank;  /* 虚拟小腿 */
    dbg[33] = rl_control.torque_state.virtual_torque[1]; /* vshank tau_v */
    dbg[34] = rl_control.torque_state.last_torque[RL_TQ_L_THIGH]; /* tau_f */
    dbg[35] = rl_control.torque_state.last_torque[RL_TQ_L_SHANK]; /* tau_b */
    dbg[30] = (float)output_debug_dm_sent;

    motor_mask = 0u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (motor_state.dm.online[i])
        {
            motor_mask |= (uint8_t)(1u << i);
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (motor_state.dji.online[i])
        {
            motor_mask |= (uint8_t)(1u << (4u + i));
        }
    }
    if (robot_state.enabled)
    {
        motor_mask |= (uint8_t)(1u << 6);
    }
    if (torque_output_enabled)
    {
        motor_mask |= (uint8_t)(1u << 7);
    }
    dbg[31] = (float)motor_mask;
    Vofa_Send(dbg, 36u);
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
