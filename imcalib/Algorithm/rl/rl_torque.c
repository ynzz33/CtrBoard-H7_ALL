/*
 * 六路原始action→四电机位置目标/双轮速度目标→模型力矩→固件电机力矩。
 * 每个RL控制子步用当前rad/rad/s反馈计算；不使用观测缩放、角差环绕或雅可比。
 * 训练已包含实体被动弹簧作用，本路径不叠加弹簧抵消或重力补偿；发送归执行任务。
 */
#include "rl_torque.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>
/* 投入/退出时清目标与诊断，不保留额外目标延迟。 */
void RL_Torque_Reset(rl_torque_state_t *state)
{
    memset(state, 0, sizeof(*state));
}
/* 原始action重排后生成目标；PD使用当前原生q/dq，全部求解成功才置torque.valid。 */
uint8_t RL_Torque_Compute(const rl_motor_state_t *motor, const machine_rl_cfg_t *config, const float action[RL_ACTION_SIZE], rl_torque_state_t *state, torque_output_t *torque)
{
    float value;
    memset(torque, 0, sizeof(*torque));
    if (!motor->valid) { return 0u; }
    /* 零action指向q_nominal，目标边界以XML的q_reference为中心，不环绕角差。 */
    for (uint8_t i = 0u; i < RL_LEG_MOTOR_NUM; i++)
    {
        state->target[i] = clampf(config->nominal_pos[i] + config->torque.pos_scale * action[RL_Action_Index(i)], config->position_limit_center[i] - config->torque.position_limit, config->position_limit_center[i] + config->torque.position_limit);
    }
    /* model1999网络action[4、5]生成左右轮轴速度目标。 */
    for (uint8_t i = 0u; i < 2u; i++)
    {
        state->target[RL_MOTOR_L_WHEEL + i] = clip(config->torque.wheel_vel_scale * action[RL_Action_Index(RL_MOTOR_L_WHEEL + i)], config->torque.wheel_speed_limit);
    }
    if (!Float_Array_Finite(state->target, RL_ACTION_SIZE)) { return 0u; }
    memcpy(state->pos, motor->pos, sizeof(state->pos));
    memcpy(state->vel, motor->vel, sizeof(state->vel));
    /* 四腿：Kp*(目标角-当前角)-Kd*当前速度；速度阻尼无误差差分或dt换算。 */
    for (uint8_t i = 0u; i < RL_LEG_MOTOR_NUM; i++)
    {
        value = config->torque.p_gains[i] * (state->target[i] - motor->pos[i]) - config->torque.d_gains[i] * motor->vel[i];
        if (!isfinite(value)) { return 0u; }
        state->motor_torque[i] = clip(value, config->torque.leg_torque_limit);
        torque->dm[i] = (float)config->motor_sign[i] * state->motor_torque[i]; /* 模型→固件，与输入映射配对。 */
    }
    /* 双轮只有速度比例环：目标轮速高于实测时加速，低于实测时制动。 */
    for (uint8_t i = 0u; i < 2u; i++)
    {
        value = config->torque.wheel_gain[i] * (state->target[RL_MOTOR_L_WHEEL + i] - motor->vel[RL_MOTOR_L_WHEEL + i]);
        if (!isfinite(value)) { return 0u; }
        state->motor_torque[RL_MOTOR_L_WHEEL + i] = clip(value, config->torque.wheel_torque_limit);
        torque->dji[i] = (float)config->motor_sign[RL_MOTOR_L_WHEEL + i] * state->motor_torque[RL_MOTOR_L_WHEEL + i];
    }
    torque->valid = 1u;
    state->last_torque = *torque;
    return 1u;
}
