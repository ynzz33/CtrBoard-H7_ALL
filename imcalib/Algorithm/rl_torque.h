#ifndef RL_TORQUE_H
#define RL_TORQUE_H

#include <stdint.h>

#include "leg_solver.h"
#include "pid.h"
#include "rl_policy.h"

/* 物理通道: 与 DM/DJI 数组下标一致 */
enum {
    RL_TQ_L_THIGH = 0,
    RL_TQ_L_SHANK = 1,
    RL_TQ_R_THIGH = 2,
    RL_TQ_R_SHANK = 3,
    RL_TQ_L_WHEEL = 4,
    RL_TQ_R_WHEEL = 5,
    RL_TQ_NUM     = 6,
};

typedef struct {
    float dof_pos[6];
    float p_gains[6];
    float d_gains[6];
    float wheel_kp[2];
    uint8_t spin_mode;
    uint8_t jump_mode;
} rl_torque_param_t;

typedef struct {
    float last_torque[RL_TQ_NUM];
    float virtual_torque[RL_ACTION_SIZE];
    pid_t controller[RL_ACTION_SIZE];
    /* PID debug: [0]=L_thigh [1]=L_shank */
    float pid_target[2];
    float pid_err[2];
    float pid_output[2];
    /* encoder 解包状态 */
    float q_unwrapped[RL_ACTION_SIZE];
    float q_prev[RL_ACTION_SIZE];
    uint8_t q_init;
} rl_torque_state_t;

void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model);
void RL_Torque_State_Init(rl_torque_state_t *state,
                          const rl_torque_param_t *param);
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          float torque[RL_TQ_NUM]);

#endif
