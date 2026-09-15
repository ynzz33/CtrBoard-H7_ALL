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
    float gas_spring[2];
    float max_step[6];
    uint8_t spin_mode;
    uint8_t jump_mode;
} rl_torque_param_t;

typedef struct {
    float last_torque[RL_TQ_NUM];
    float virtual_torque[RL_ACTION_SIZE];
    pid_t controller[RL_ACTION_SIZE];
    const rl_torque_param_t *param_ref;
    uint8_t valid;
} rl_torque_state_t;

void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model);
void RL_Torque_State_Init(rl_torque_state_t *state);
void RL_Torque_Clamp_Output(float torque[RL_TQ_NUM], float leg_limit,
                            float wheel_limit);
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          float torque[RL_TQ_NUM]);

#endif
