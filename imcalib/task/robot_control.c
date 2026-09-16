#include "robot_control.h"

#include <string.h>

imu_state_t imu_state;
motor_state_t motor_state;
leg_state_t leg_l;
leg_state_t leg_r;
action_state_t action_state;
input_command_t input_command;
leg_map_t leg_map_l;
leg_map_t leg_map_r;
robot_state_t robot_state;
rl_control_state_t rl_control;
volatile uint32_t ctrl_fault;
volatile uint8_t output_debug_dm_sent;
volatile uint8_t output_debug_dji_sent;
uint8_t torque_output_enabled;

osSemaphoreDef(ctrl_tick_sem);
osSemaphoreId ctrl_tick_sem_handle = NULL;

/* 清空动作 */
void Action_State_Clear(void)
{
    memset(action_state.a, 0, sizeof(action_state.a));
    action_state.last_ok_tick = 0u;
    action_state.updated = 0u;
    action_state.base_action_locked = 0u;
}

/* 控制初始化 */
void Robot_Control_Init(void)
{
    ctrl_tick_sem_handle = osSemaphoreCreate(osSemaphore(ctrl_tick_sem), 1);
    torque_output_enabled = 1u;

    Leg_Init(&leg_l);
    Leg_Init(&leg_r);
    leg_l.config.lu = 0.13087f;
    leg_l.config.lg = 0.15240f;
    leg_l.config.offset_f = -0.03f;
    leg_l.config.offset_b = -0.04f;
    leg_l.config.offset_phi0 = -0.13f;
    leg_l.config.mirror = 1;
    leg_l.config.configured = 1u;
    leg_r.config.lu = 0.13087f;
    leg_r.config.lg = 0.15240f;
    leg_r.config.offset_f = -0.038f;
    leg_r.config.offset_b = -0.023f;
    leg_r.config.offset_phi0 = -0.07f;
    leg_r.config.mirror = 1;
    leg_r.config.configured = 1u;

    leg_map_l.dm_front = DM_MOTOR_LEG_F_LFT;
    leg_map_l.dm_rear = DM_MOTOR_LEG_B_LFT;
    leg_map_l.configured = 1u;
    leg_map_r.dm_front = DM_MOTOR_LEG_F_RGT;
    leg_map_r.dm_rear = DM_MOTOR_LEG_B_RGT;
    leg_map_r.configured = 1u;

    RL_Observation_Init(&rl_control.observation);
    RL_Observation_Param_Init(&rl_control.param);
    RL_Policy_Reset(&rl_control.policy);
    RL_Torque_Param_Init(&rl_control.torque_param[RL_MODEL_STABLE], RL_MODEL_STABLE);
    RL_Torque_Param_Init(&rl_control.torque_param[RL_MODEL_UPSTAIRS], RL_MODEL_UPSTAIRS);
    RL_Torque_Param_Init(&rl_control.torque_param[RL_MODEL_PIN], RL_MODEL_PIN);
    RL_Torque_Param_Init(&rl_control.torque_param[RL_MODEL_JUMP], RL_MODEL_JUMP);
    RL_Torque_State_Init(&rl_control.torque_state, &rl_control.torque_param[RL_MODEL_STABLE]);
    Action_State_Clear();
}

/* 切换模型 */
uint8_t RL_Control_Select_Model(rl_model_t model)
{
    if (!RL_Policy_Select(&rl_control.policy, model))
    {
        return 0u;
    }
    RL_Observation_Reset(&rl_control.observation);
    RL_Torque_State_Init(&rl_control.torque_state, &rl_control.torque_param[model]);
    Action_State_Clear();
    return 1u;
}
