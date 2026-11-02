/*
 * 定义跨任务共享对象；上电创建执行/推理信号量，装入腿几何并清控制器历史。
 * TIM6经Policy_Tick_Div唤醒策略任务；Action_State_Clear使旧动作及在途推理失效。
 */

#include "robot_control.h"
#include "robot_state.h"
#include "machine_config.h"
#include "fallen_process.h"
#include "air_detection.h"
#include "control_condition.h"
#include "fallen_detection.h"

#include <string.h>

imu_state_t imu_state;
motor_state_t motor_state;
leg_state_t leg_l;
leg_state_t leg_r;
action_state_t action_state;
rc_command_t rc_command;
leg_map_t leg_map_l;
leg_map_t leg_map_r;
robot_state_t robot_state;
rl_control_state_t rl_control;
lqr_state_t lqr_state;
leg_balance_t leg_balance;
volatile ctrl_strategy_t ctrl_strategy;
volatile uint32_t ctrl_fault;
volatile uint8_t output_debug_dm_sent;
volatile uint8_t output_debug_dji_sent;
volatile control_time_debug_t control_time_debug;
uint8_t torque_output_enabled;
volatile compensation_debug_t compensation_debug;

osSemaphoreDef(ctrl_tick_sem);
osSemaphoreId ctrl_tick_sem_handle = NULL;
osSemaphoreDef(policy_tick_sem);
osSemaphoreId policy_tick_sem_handle = NULL;

/* TIM6中断每累计MACHINE_POLICY_DIV个基础拍释放一次策略信号量，随后计数归零。 */
void Policy_Tick_Div(void)
{
    static uint8_t div_cnt;   /* 分频计数 */

    if (++div_cnt >= MACHINE_POLICY_DIV)
    {
        div_cnt = 0u;
        osSemaphoreRelease(policy_tick_sem_handle);
    }
}

/* 清原始action和就绪/时间标志，并增加reset_count让清零前启动的推理失效。
 * 保存/恢复原中断屏蔽状态，使策略和执行任务看不到半清零动作。 */
void Action_State_Clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(action_state.a, 0, sizeof(action_state.a));
    action_state.reset_count++;
    action_state.last_ok_tick = 0u;
    action_state.updated = 0u;
    action_state.rl_ready = 0u;
    __set_PRIMASK(primask);
}

/* 创建节拍信号量，按机器表初始化双腿几何/索引，清检测、自起及LQR/RL历史。
 * 这里只准备运行状态，CubeAI网络初始化由策略任务ctrl_task_init执行。 */
void Robot_Control_Init(void)
{
    ctrl_tick_sem_handle = osSemaphoreCreate(osSemaphore(ctrl_tick_sem), 1);
    if (ctrl_tick_sem_handle == NULL)
    {
        Error_Handler();
    }
    policy_tick_sem_handle = osSemaphoreCreate(osSemaphore(policy_tick_sem), 1);
    if (policy_tick_sem_handle == NULL)
    {
        Error_Handler();
    }
    torque_output_enabled = 1u;
    rc_command.leg_length_target = machine->rc.len_init;
    rc_command.mode.strategy = CTRL_STRATEGY_DISABLE;
    Fallen_Process_Init(&fallen_process_control);
    Air_Detection_Reset(&air_detection); /* 参数直接使用固定配置，初始化只清检测历史。 */
    Fallen_Detection_Reset(&fallen_detection);
    Control_Condition_Reset(&control_condition);

    Leg_Init(&leg_l);
    Leg_Init(&leg_r);
    leg_l.config.lu = machine->leg.leg_lu;
    leg_l.config.lg = machine->leg.leg_lg;
    leg_l.config.offset_phi0 = machine->leg.leg_off_phi0[0];
    leg_r.config.lu = machine->leg.leg_lu;
    leg_r.config.lg = machine->leg.leg_lg;
    leg_r.config.offset_phi0 = machine->leg.leg_off_phi0[1];

    leg_map_l.dm_front = DM_MOTOR_LEG_F_LFT;
    leg_map_l.dm_rear = DM_MOTOR_LEG_B_LFT;
    leg_map_r.dm_front = DM_MOTOR_LEG_F_RGT;
    leg_map_r.dm_rear = DM_MOTOR_LEG_B_RGT;

    /* 清当前RL观测、网络运行标志、力矩目标和启动/失败计时；网络实例稍后由策略任务创建。 */
    RL_Observation_Reset(&rl_control.observation);
    RL_Policy_Reset(&rl_control.policy);
    RL_Torque_Reset(&rl_control.torque_state);
    RL_Recovery_Reset(&rl_control.recovery);
    Action_State_Clear();

    LQR_Init(&lqr_state);
    Leg_Balance_Init(&leg_balance);
    ctrl_strategy = CTRL_STRATEGY_DISABLE;
}
