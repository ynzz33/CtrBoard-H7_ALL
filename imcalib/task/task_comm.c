/*
 * 文件职责：通信刷新、腿解算、故障/跌倒/使能管理（task_comm.c）。
 * 输入/来源：DR16/DM/DJI接收数据、设备在线状态及已有IMU。
 * 输出/去向：更新物理电机量、解算双腿、发布遥控/电机反馈并维护许可。
 * 关键边界：协议解析在驱动，运动学在leg_solver；本任务管理许可，不承担策略力矩求解。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "robot_control.h"
#include "robot_state.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"
#include "jetson_uart.h"

#include <math.h>
#include <string.h>

/* 更新电机状态 */
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

        next->dji.angle_rad[i] = feedback->angle_rad;
        next->dji.angle_total_rad[i] = feedback->angle_total_rad;
        next->dji.vel_rad_s[i] = feedback->vel_rad_s;
        next->dji.current_raw[i] = feedback->current_raw;
        next->dji.last_rx_tick[i] = feedback->last_rx_tick;
        next->dji.online[i] = (uint8_t)Dji_Is_Online(i);
        next->valid &= next->dji.online[i];
    }
    next->timestamp_ms = HAL_GetTick();
    next->updated = 1u;
}

/* 更新单腿 */
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

/* 更新腿部状态 */
/*
 * 按电机物理反馈更新双腿输入，再调用leg_solver得到几何、速度和力映射有效性。
 * 腿几何用于LQR/离地估计；RL直接使用实体电机反馈，不在此调整极性、零点或设目标。
 */
static void Leg_State_Update(const motor_state_t *motor, leg_state_t *left, leg_state_t *right)
{
    Leg_Side_Update(motor, &leg_map_l, left);
    Leg_Side_Update(motor, &leg_map_r, right);
}

/* 更新遥控: 指令解算 + 使能 */
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


/* 更新故障状态 */
/*
 * 汇总传感器/遥控/DM/CAN已有故障与在线判据。
 * ctrl_fault是整机许可依据，和离地观察器自身valid/fault不是同一标志。
 */
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
    if (ctrl_strategy == CTRL_STRATEGY_RL && rl_control.policy.ready && action_state.updated && HAL_GetTick() - action_state.last_ok_tick >= machine->rl.action_timeout_ms)
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
        else if (HAL_GetTick() - rl_ready_lost_tick >= machine->rl.ready_timeout_ms)
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
/*
 * 直接读取IMU回传的俯仰/横滚角，任一绝对值超过40度写公共fallen。
 * 不包含高度条件；恢复出力许可及完成保持由执行层另行选择。
 */
static void Robot_Fallen_Update(void)
{
    float pitch_deg;
    float roll_deg;

    if (ctrl_fault & FAULT_IMU)
    {
        return;
    }
    taskENTER_CRITICAL();
    pitch_deg = imu_state.euler_deg[ATTITUDE_ROLL]; /* 沿用LQR实际俯仰取轴。 */
    roll_deg = imu_state.euler_deg[ATTITUDE_PITCH];
    taskEXIT_CRITICAL();
    robot_state.fallen = (uint8_t)(fabsf(pitch_deg) > 40.0f || fabsf(roll_deg) > 40.0f);
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
/*
 * 每拍推进通信缓存解析、物理反馈、遥控、腿解算、故障和电机使能管理。
 * 准备好的电机/腿/遥控结果通过快照发布，控制任务读取；不在通信循环内求LQR或RL力矩。
 */
void comm_task_body(void)
{
    motor_state_t motor_state_temp;
    leg_state_t leg_l_temp,leg_r_temp;

    Jetson_Uart_Process(); /* 独立探测接收，不改变控制链顺序。 */
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
    Robot_Fallen_Update();
    Robot_Enable_Update();
    
    
}
