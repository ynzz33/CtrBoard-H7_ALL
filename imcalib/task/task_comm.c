/*
 * 每轮解析DM/DJI缓存、换算电机物理反馈并解算双腿，成批发布反馈。
 * 随后更新遥控、故障位和电机使能/失能；力矩计算由执行任务完成。
 */

#include "robot_control.h"
#include "robot_state.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

/* 将DM/DJI驱动反馈复制为统一电机状态，汇总在线/故障。 */
static void Motor_State_Update(motor_state_t *next)
{
    next->valid = 1u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];

        next->dm.pos_rad[i] = feedback->pos_rad;
        next->dm.pos_zero_rad[i] = feedback->pos_zero_rad;
        next->dm.pos_continuous_rad[i] = feedback->pos_continuous_rad;
        next->dm.vel_rad_s[i] = feedback->vel_rad_s;
        next->dm.trq_nm[i] = feedback->trq_nm;
        next->dm.last_rx_tick[i] = feedback->last_rx_tick;
        next->dm.parsed_rx_ns[i] = feedback->parsed_rx_ns;
        next->dm.online[i] = (uint8_t)Dm_Is_Online(i);
        next->valid &= (uint8_t)(next->dm.online[i] && !Dm_Has_Fault(i));
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

#if MACHINE_LATENCY_CAPTURE_ENABLE
        next->dji.parsed_rx_ns[i] = feedback->parsed_rx_ns;
#endif
        next->dji.angle_rad[i] = feedback->angle_rad;
        next->dji.angle_total_rad[i] = feedback->angle_total_rad;
        next->dji.vel_rad_s[i] = feedback->vel_rad_s;
        next->dji.current_raw[i] = feedback->current_raw;
        next->dji.last_rx_tick[i] = feedback->last_rx_tick;
        next->dji.online[i] = (uint8_t)Dji_Is_Online(i);
        next->valid &= next->dji.online[i];
    }
}

/* 把绑定的前后髋反馈角rad和速度rad/s送入五连杆求解；任一离线则清输出有效性。 */
static void Leg_Side_Update(const motor_state_t *motor, const leg_map_t *map, leg_state_t *leg)
{
    /* 前后电机下标由初始化固定绑定，这里只判断两路反馈是否在线。 */
    if (motor->dm.online[map->dm_front] && motor->dm.online[map->dm_rear])
    {
        //杆长向后为零点
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

/* 用同一批电机状态分别解算左右腿，写入待发布的双腿几何和映射结果。 */
static void Leg_State_Update(const motor_state_t *motor, leg_state_t *left, leg_state_t *right)
{
    Leg_Side_Update(motor, &leg_map_l, left);
    Leg_Side_Update(motor, &leg_map_r, right);
}

/* 处理DR16接收并转换为统一遥控指令，再通过快照接口发布。 */
static void Remote_Control_Update(void)
{
    dr16_t remote;
    rc_command_t rc_command_temp;
    DR16_Process();
    remote = DR16_Snapshot();
    rc_command_temp = rc_command;
    Rc_Command_Update(&rc_command_temp, &remote, MACHINE_TICK_DT);
    if (!(output_task_lqr_engaged() || output_task_rl_engaged()))
    {
        rc_command_temp.leg_length_target = machine->rc.len_init;
    }
    Robot_Feedback_Publish_Rc(&rc_command_temp);
}


/* 每轮重算IMU/遥控/电机/CAN故障位，并分别检查RL动作停更和投入后持续无动作超时。 */
static void Robot_Fault_Update(void)
{
    static uint32_t rl_ready_lost_tick;
    uint32_t fault;

    fault = FAULT_NONE;
    if (!imu_state.online || !imu_state.pitch_world_valid)
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
    if (!motor_state.valid)
    {
        fault |= FAULT_MOTOR;
    }
    /* 模型就绪且已有动作后才判停更；等待首帧由下面的持续不可用保护处理。 */
    if (ctrl_strategy == CTRL_STRATEGY_RL && rl_control.policy.ready && action_state.updated && HAL_GetTick() - action_state.last_ok_tick >= machine->rl.timing.action_timeout_ms)
    {
        fault |= FAULT_ACTION;
    }
    /* 实际投入但动作持续不可用；与已发布动作停更分别计时。 */
    if (output_task_rl_engaged() && action_state.rl_ready == 0)
    {
        if (rl_ready_lost_tick == 0u)
        {
            rl_ready_lost_tick = HAL_GetTick();
        }
        else if (HAL_GetTick() - rl_ready_lost_tick >= machine->rl.timing.ready_timeout_ms)
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

/* 更新使能状态: 使能沿发使能, 失能沿发失能; 两个方向都有看门狗兜底 */
/* 根据使能请求的上升/下降沿发送DM使能/失能及DJI零电流；每轮看门狗补发未完成命令。 */
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

/* 解析驱动缓存→整理电机反馈→解算双腿并发布→更新遥控和故障→管理硬件使能。 */
void comm_task_body(void)
{
    motor_state_t motor_state_temp;
    leg_state_t leg_l_temp,leg_r_temp;

    motor_state_temp = motor_state;
    leg_l_temp = leg_l;
    leg_r_temp = leg_r;
    Dm_Parse();
    Dji_Parse();
    Motor_State_Update(&motor_state_temp);
    Leg_State_Update(&motor_state_temp, &leg_l_temp, &leg_r_temp);
    Robot_Feedback_Publish_Motor(&motor_state_temp, &leg_l_temp, &leg_r_temp);

    Remote_Control_Update();
    Robot_Fault_Update();
    Robot_Enable_Update();
    
    
}
