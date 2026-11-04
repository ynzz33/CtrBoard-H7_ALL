/*
 * 将执行任务采样的物理反馈转成模型坐标、投影重力并组装当前网络观测。
 * 同时维护RL启动保持/连续异常失败计时；last_action保存网络原始动作，不保存最终力矩。
 */
#ifndef RL_OBSERVATION_H
#define RL_OBSERVATION_H
#include "rl_types.h"
#include "machine_config.h"
#include "rc_types.h"
/* 行走模型不负责倒地起立；保留原NORMAL/FAILED诊断编号。 */
typedef enum {
    RL_RECOVERY_IDLE = 0, RL_RECOVERY_NORMAL = 3, RL_RECOVERY_FAILED = 4
} rl_recovery_phase_t;
typedef struct {
    rl_recovery_phase_t phase;
    uint16_t fallen_ms;            /* 重力Z持续超限时间；退出投入才解除失败。 */
} rl_recovery_state_t;
/* 每段起点对应训练actor拼接顺序；位置和速度均使用rl_types中的实体电机顺序。 */
typedef enum {
    RL_OBS_GYRO_X = 0, RL_OBS_GYRO_Y, RL_OBS_GYRO_Z,
    RL_OBS_GRAV_X, RL_OBS_GRAV_Y, RL_OBS_GRAV_Z,
    RL_OBS_CMD_VX, RL_OBS_CMD_YAW_RATE, RL_OBS_CMD_HEIGHT,
    RL_OBS_MOTOR_POS = 9, RL_OBS_MOTOR_VEL = 13, RL_OBS_LAST_ACTION = 19,
} rl_obs_index_t;
/* 一份模型坐标反馈，未进行观测缩放或减名义角。 */
typedef struct {
    float gyro[3];                  /* 机体XYZ角速度，rad/s */
    float gravity[3];               /* 机体系单位重力 */
    float pos[RL_LEG_MOTOR_NUM];     /* 四电机连续模型角，rad */
    float vel[RL_ACTION_SIZE];      /* 四电机与左右轮速度，rad/s */
} rl_observation_sample_t;
typedef struct {
    float obs[RL_OBS_SIZE];         /* 缩放后整体裁剪至±100，网络当前帧输入 */
    float history[RL_HISTORY_SIZE]; /* v2：裁剪前帧，最旧在前，每策略拍推进一次 */
    float last_action[RL_ACTION_SIZE]; /* 上次经±100裁剪的动作，网络顺序 */
    rl_observation_sample_t sample; /* 最新500Hz反馈，不主动添加观测延迟 */
    uint8_t sample_valid;           /* 最新反馈可用，坏反馈立即清除 */
    uint8_t warmup_steps;          /* 首次零动作已执行的有效PD子步数 */
    uint8_t history_initialized;    /* 首帧填满，首个零动作热步不运行网络 */
    uint8_t valid;                  /* 当前帧和历史均有限 */
} rl_observation_state_t;
/* 连续角首次选最近reference分支，观测与PD共用；退出投入清initialized。 */
typedef struct {
    float position_offset[RL_LEG_MOTOR_NUM];
    uint8_t initialized;
} rl_motor_map_state_t;
uint8_t RL_Motor_Map(const machine_rl_cfg_t *config, float position_period, const float position[4], const float velocity[4], const float wheel_velocity[2], uint8_t source_valid, rl_motor_map_state_t *state, rl_motor_state_t *out);
float RL_Command_Height(float leg_length_command);
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3]);
void RL_Observation_Reset(rl_observation_state_t *state);
void RL_Observation_Sample(rl_observation_state_t *state, const float gyro[3], const float quat[4], const rl_motor_state_t *motor, uint8_t source_valid);
uint8_t RL_Observation_Read_Sample(const rl_observation_state_t *state, rl_observation_sample_t *sample);
void RL_Recovery_Reset(rl_recovery_state_t *state);
void RL_Recovery_Update(rl_recovery_state_t *state, uint8_t request, uint8_t feedback_valid, float gravity_z);
uint8_t RL_Observation_Build(rl_observation_state_t *state, const machine_rl_cfg_t *config, const rl_observation_sample_t *sample, const rc_command_t *command, const rl_recovery_state_t *recovery, uint8_t source_valid);
void RL_Observation_Set_Last_Action(rl_observation_state_t *state, const float action[RL_ACTION_SIZE]);
#endif
