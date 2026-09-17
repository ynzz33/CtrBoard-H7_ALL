#ifndef RL_TORQUE_H
#define RL_TORQUE_H

#include <stdint.h>

#include "leg_solver.h"
#include "pid.h"
#include "rl_policy.h"

/* 物理电机输出通道: 与 DM/DJI 电机顺序一致 */
enum {
    RL_TQ_DM_F_LFT = 0,   /* 左前髋 DM电机 */
    RL_TQ_DM_B_LFT = 1,   /* 左后髋 DM电机 */
    RL_TQ_DJI_LFT  = 2,   /* 左轮 DJI电机 */
    RL_TQ_DM_F_RGT = 3,   /* 右前髋 DM电机 */
    RL_TQ_DM_B_RGT = 4,   /* 右后髋 DM电机 */
    RL_TQ_DJI_RGT  = 5,   /* 右轮 DJI电机 */
    RL_TQ_NUM      = 6,
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
