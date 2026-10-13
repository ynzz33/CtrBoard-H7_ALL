/*
 * 文件职责：训练侧观测构建和历史队列（rl_observation.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：机体角速度/四元数、训练角色关节状态、遥控指令和上次动作。
 * 输出/去向：构建25维观测、5帧125维历史；完成实体角色与训练角色互逆转换。
 * 关键边界：缩放、裁剪、默认角和历史顺序需与训练一致；这里不推理、不选策略、不输出电机。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef RL_OBSERVATION_H
#define RL_OBSERVATION_H

#include <stdint.h>
#include "rl_types.h"
#include "machine_config.h"

#define RL_OBS_SIZE           25u
#define RL_OBS_HISTORY_FRAMES 5u
#define RL_OBS_HISTORY_SIZE   (RL_OBS_SIZE * RL_OBS_HISTORY_FRAMES)

/* 观测索引 */
typedef enum {
    RL_OBS_GYRO_X = 0,       /* 角速度 */
    RL_OBS_GYRO_Y,
    RL_OBS_GYRO_Z,
    RL_OBS_GRAV_X,           /* 投影重力 */
    RL_OBS_GRAV_Y,
    RL_OBS_GRAV_Z,
    RL_OBS_CMD_VX,           /* 控制指令 */
    RL_OBS_CMD_YAW_RATE,
    RL_OBS_CMD_HEIGHT,
    RL_OBS_L_THIGH,          /* 关节角度 */
    RL_OBS_L_SHANK,
    RL_OBS_R_THIGH,
    RL_OBS_R_SHANK,
    RL_OBS_L_THIGH_VEL,      /* 关节速度 */
    RL_OBS_L_SHANK_VEL,
    RL_OBS_L_WHEEL_VEL,
    RL_OBS_R_THIGH_VEL,
    RL_OBS_R_SHANK_VEL,
    RL_OBS_R_WHEEL_VEL,
    RL_OBS_LAST_ACTION = 19  /* 上次动作 */
} rl_obs_index_t;

typedef struct {
    float obs_dof_pos[4];        /* 观测中位 */
    float command_scale[3];      /* 指令缩放 */
    float gyro_scale[3];         /* 陀螺缩放 */
    float joint_vel_scale[6];    /* 速度缩放 */
    float obs_clip;              /* 观测裁剪 */
    uint8_t configured;          /* 参数有效 */
} rl_observation_param_t;

typedef struct {
    float obs[RL_OBS_SIZE];              /* 当前观测 */
    float history[RL_OBS_HISTORY_SIZE];  /* 五帧历史 */
    float last_action[RL_ACTION_SIZE];   /* 上次动作 */
    uint8_t valid;                       /* 观测有效 */
    uint8_t history_ready;               /* 历史有效 */
} rl_observation_state_t;

/* reference 为机器表校准后的实体 [左大腿 左膝 左轮 右大腿 右膝 右轮]。
 * training 为 [lf0 lf1 lfwheel rf0 rf1 rfwheel]；转换互逆，支持原地调用。 */
void RL_Observation_Roles_To_Training(const float reference[RL_ACTION_SIZE],
                                      float training[RL_ACTION_SIZE]);
void RL_Observation_Roles_From_Training(const float training[RL_ACTION_SIZE],
                                        float reference[RL_ACTION_SIZE]);
void RL_Observation_Init(rl_observation_state_t *state);
void RL_Observation_Param_Init(rl_observation_param_t *param,
                               const machine_rl_cfg_t *config);
void RL_Observation_Reset(rl_observation_state_t *state);
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3]);
uint8_t RL_Observation_Build(rl_observation_state_t *state,
                             const rl_observation_param_t *param,
                             const float gyro_rad_s[3],
                             const float quat[4],
                             const float command[3],
                             const float joint_pos[4],
                             const float joint_vel[6],
                             uint8_t source_valid);
void RL_Observation_Update_History(rl_observation_state_t *state);
void RL_Observation_Set_Last_Action(rl_observation_state_t *state,
                                    const float action[RL_ACTION_SIZE]);

#endif
