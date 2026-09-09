#ifndef RL_OBSERVATION_H
#define RL_OBSERVATION_H

#include <stdint.h>

#define RL_OBS_SIZE           25u
#define RL_OBS_HISTORY_FRAMES 5u
#define RL_OBS_HISTORY_SIZE   (RL_OBS_SIZE * RL_OBS_HISTORY_FRAMES)
#define RL_ACTION_SIZE        6u

/* 观测索引 */
typedef enum {
    RL_OBS_GYRO_X = 0,
    RL_OBS_GYRO_Y,
    RL_OBS_GYRO_Z,
    RL_OBS_GRAV_X,
    RL_OBS_GRAV_Y,
    RL_OBS_GRAV_Z,
    RL_OBS_CMD_VX,
    RL_OBS_CMD_YAW_RATE,
    RL_OBS_CMD_HEIGHT,
    RL_OBS_L_THIGH,
    RL_OBS_L_SHANK,
    RL_OBS_R_THIGH,
    RL_OBS_R_SHANK,
    RL_OBS_L_THIGH_VEL,
    RL_OBS_L_SHANK_VEL,
    RL_OBS_L_WHEEL_VEL,
    RL_OBS_R_THIGH_VEL,
    RL_OBS_R_SHANK_VEL,
    RL_OBS_R_WHEEL_VEL,
    RL_OBS_LAST_ACTION = 19
} rl_obs_index_t;

typedef struct {
    float obs_dof_pos[4];
    float command_scale[3];
    float gyro_scale[3];
    float joint_vel_scale[6];
    uint8_t configured;
} rl_observation_param_t;

typedef struct {
    float obs[RL_OBS_SIZE];
    float history[RL_OBS_HISTORY_SIZE];
    float last_action[RL_ACTION_SIZE];
    uint8_t valid;
    uint8_t history_ready;
} rl_observation_state_t;

void RL_Observation_Init(rl_observation_state_t *state);
void RL_Observation_Param_Init(rl_observation_param_t *param);
void RL_Observation_Reset(rl_observation_state_t *state);
uint8_t RL_Observation_Build(rl_observation_state_t *state,
                             const rl_observation_param_t *param,
                             const float gyro_rad_s[3],
                             const float quat[4],
                             const float command[3],
                             const float joint_pos[4],
                             const float joint_vel[6],
                             uint8_t source_valid);
void RL_Observation_Update_History(rl_observation_state_t *state);
uint8_t RL_Observation_Set_Last_Action(rl_observation_state_t *state,
                                       const float action[RL_ACTION_SIZE]);
uint8_t RL_Observation_Gate_Action(const rl_observation_state_t *state,
                                   float action[RL_ACTION_SIZE]);

#endif
