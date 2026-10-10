#include "robot_control.h"
#include "robot_state.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"
#include "mono_ns.h"

#include <math.h>
#include <string.h>

/* 更新电机状态 */
static void Motor_State_Update(motor_state_t *next)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];

        next->dm.pos_rad[i] = feedback->pos_rad;
        next->dm.pos_zero_rad[i] = feedback->pos_zero_rad;
        next->dm.vel_rad_s[i] = feedback->vel_rad_s;
        next->dm.trq_nm[i] = feedback->trq_nm;
        next->dm.last_rx_tick[i] = feedback->last_rx_tick;
        next->dm.parsed_rx_ns[i] = feedback->parsed_rx_ns;
        next->dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

        next->dji.angle_rad[i] = feedback->angle_rad;
        next->dji.angle_total_rad[i] = feedback->angle_total_rad;
        next->dji.vel_rad_s[i] = feedback->vel_rad_s;
        next->dji.current_raw[i] = feedback->current_raw;
        next->dji.last_rx_tick[i] = feedback->last_rx_tick;
        next->dji.online[i] = (uint8_t)Dji_Is_Online(i);
    }
    next->timestamp_ms = HAL_GetTick();
    next->updated = 1u;
}

/* 更新单腿 */
static void Leg_Side_Update(const motor_state_t *motor, const leg_map_t *map, leg_state_t *leg)
{
    if (map->configured
        && (uint8_t)map->dm_front < DM_MOTOR_NUM && (uint8_t)map->dm_rear < DM_MOTOR_NUM)
    {
        leg->input.hip_f = motor->dm.pos_zero_rad[map->dm_front] + LEG_PI;
        leg->input.hip_b = motor->dm.pos_zero_rad[map->dm_rear];
        leg->input.d_hip_f = motor->dm.vel_rad_s[map->dm_front];
        leg->input.d_hip_b = motor->dm.vel_rad_s[map->dm_rear];
        (void)Leg_Solve(leg);
    }
    else
    {
        memset(&leg->output, 0, sizeof(leg->output));
    }
}

/* 更新腿部状态 */
static void Leg_State_Update(const motor_state_t *motor, leg_state_t *left, leg_state_t *right)
{
    Leg_Side_Update(motor, &leg_map_l, left);
    Leg_Side_Update(motor, &leg_map_r, right);
}

/* 更新遥控: 指令解算 + 使能 */
static void Remote_Control_Update(void)
{
    dr16_t remote;
    rc_command_t next;
    uint8_t rc_enable;
    feedback_sample_t received;

    DR16_Process();
    remote = DR16_Snapshot();
    next = rc_command;
    Rc_Command_Update(&next, &remote);
    rc_enable = strategy_rc_enable(&next);
    received.received_ns = remote.received_ns;
    received.seen = remote.online;
    Robot_Feedback_Publish_Rc(&next, rc_enable, remote.online ? remote.s1 : 0u, received);
}


/* 更新故障状态 */
static void Robot_Fault_Update(void)
{
    static uint32_t rl_ready_lost_tick;
    uint32_t fault;
    uint8_t motors_ok;

    fault = FAULT_NONE;
    motors_ok = 1u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i] || Dm_Has_Fault(i))
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
    if (!imu_state.online || !imu_state.pitch_world_valid || !isfinite(imu_state.pitch_world))
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
    if (ctrl_strategy == CTRL_STRATEGY_RL
        && (!action_state.updated || HAL_GetTick() - action_state.last_ok_tick >= 100u))
    {
        fault |= FAULT_ACTION;
    }
    /* 动作持续不可用 */
    if (output_task_rl_engaged() && action_state.rl_ready == 0)
    {
        if (rl_ready_lost_tick == 0u)
        {
            rl_ready_lost_tick = HAL_GetTick();
        }
        else if (HAL_GetTick() - rl_ready_lost_tick >= 100u)
        {
            fault |= FAULT_ACTION;
        }
    }
    else
    {
        rl_ready_lost_tick = 0u;
    }
    ctrl_fault = fault;
}

/* 更新翻倒状态 */
static void Robot_Fallen_Update(void)
{
    float pitch_abs;

    if (ctrl_fault & FAULT_IMU)
    {
        return;
    }
    taskENTER_CRITICAL();
    pitch_abs = fabsf(imu_state.pitch_world);
    taskEXIT_CRITICAL();
    if (pitch_abs > 1.4f)
    {
        robot_state.fallen = 1u;
    }
    else if (pitch_abs < 1.0f)
    {
        robot_state.fallen = 0u;
    }
}

/* 更新使能状态: 使能沿发使能, 失能沿发失能; 两个方向都有看门狗兜底 */
static void Robot_Enable_Update(void)
{
    uint8_t enable_request;

    enable_request = Robot_Control_Enable_Allowed();
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

    if (robot_state.motor_enabled)
    {
        Dm_Enable_Watchdog();
    }
    else
    {
        Dm_Disable_Watchdog();
    }
}

/* 通信单周期 */
void comm_task_body(void)
{
    motor_state_t next_motor;
    leg_state_t next_left;
    leg_state_t next_right;
    feedback_sample_t received[DM_MOTOR_NUM + DJI_MOTOR_NUM];
    uint8_t i;

    Dm_Parse();
    Dji_Parse();
    next_motor = motor_state;
    next_left = leg_l;
    next_right = leg_r;
    Motor_State_Update(&next_motor);
    Leg_State_Update(&next_motor, &next_left, &next_right);
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        received[i].received_ns = dm_motor_feedback[i].parsed_rx_ns;
        received[i].seen = dm_motor_feedback[i].parsed_seen;
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        received[DM_MOTOR_NUM + i].received_ns = dji_motor_feedback[i].parsed_rx_ns;
        received[DM_MOTOR_NUM + i].seen = dji_motor_feedback[i].parsed_seen;
    }
    Robot_Feedback_Publish_Motor(&next_motor, &next_left, &next_right, received);

    Remote_Control_Update();
    Robot_Fault_Update();
    Robot_Fallen_Update();
    Robot_Enable_Update();
    
}
