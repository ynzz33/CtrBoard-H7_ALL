/*
 * 将六路原始action重排为四髋位置目标和双轮速度目标，再用当前反馈计算限幅力矩。
 * 腿用位置PD、轮用速度P，模型符号成对还原到固件坐标；不额外叠加弹簧或重力补偿。
 */
#ifndef RL_TORQUE_H
#define RL_TORQUE_H
#include "rl_types.h"
#include "machine_config.h"
#include "torque_output.h"
typedef struct {
    float motor_torque[RL_ACTION_SIZE]; /* 模型坐标限幅力矩，Nm */
    float target[RL_ACTION_SIZE];   /* 本次使用的四个角度(rad)/两个轮速(rad/s)目标 */
    float pos[RL_LEG_MOTOR_NUM];    /* 本次PD当前模型角度，rad */
    float vel[RL_ACTION_SIZE];     /* 本次PD当前模型速度，rad/s */
} rl_torque_state_t;
/* 8000动作按左右两组输入，内部目标保持四腿两轮；最终许可归执行任务。 */
void RL_Torque_Reset(rl_torque_state_t *state);
uint8_t RL_Torque_Compute(const rl_motor_state_t *motor, const machine_rl_cfg_t *config, const float action[RL_ACTION_SIZE], rl_torque_state_t *state, torque_output_t *torque);
#endif
