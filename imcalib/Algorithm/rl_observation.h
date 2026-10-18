/* RL39输入接口：模型原生反馈和最新样本；采样归执行任务，观测/原始动作历史归策略任务。 */
#ifndef RL_OBSERVATION_H
#define RL_OBSERVATION_H
#include "rl_types.h"
#include "machine_config.h"
#include "rc_types.h"
/* RL实际投入阶段；数值供Watch查看，不直接作为网络模式编号。 */
typedef enum {
    RL_RECOVERY_IDLE = 0, RL_RECOVERY_HOLD, RL_RECOVERY_RECOVER, RL_RECOVERY_NORMAL, RL_RECOVERY_FAILED
} rl_recovery_phase_t;
typedef struct {
    rl_recovery_phase_t phase;
    uint8_t ground_start;          /* 实际投入时锁存，站起/中途倒地均不改写。 */
    uint16_t hold_ms;              /* 趴地启动零力矩保持200ms。 */
    uint16_t stable_ms;            /* 连续非倒地500ms后完成恢复。 */
    uint16_t fallen_ms;            /* 连续倒地超过2000ms终止本次投入。 */
} rl_recovery_state_t;
/* 每段起点对应训练actor拼接顺序；位置和速度均使用rl_types中的实体电机顺序。 */
typedef enum {
    RL_OBS_CMD_VX = 0, RL_OBS_CMD_VY, RL_OBS_CMD_YAW_RATE, RL_OBS_CMD_HEIGHT,
    RL_OBS_GYRO_X, RL_OBS_GYRO_Y, RL_OBS_GYRO_Z,
    RL_OBS_GRAV_X, RL_OBS_GRAV_Y, RL_OBS_GRAV_Z,
    RL_OBS_MOTOR_POS = 10, RL_OBS_MOTOR_VEL = 16, RL_OBS_LAST_ACTION = 22, RL_OBS_CONTROL_MODE = 28,
    RL_OBS_GROUND_START = 35, RL_OBS_POLICY_ACTIVE, RL_OBS_AIRBORNE, RL_OBS_STEP_ACTIVE
} rl_obs_index_t;
/* 一份模型坐标反馈，未进行观测缩放或减名义角。 */
typedef struct {
    float gyro[3];                  /* 机体XYZ角速度，rad/s */
    float gravity[3];               /* 机体系单位重力 */
    float pos[RL_LEG_MOTOR_NUM];     /* 四电机连续角，rad */
    float vel[RL_ACTION_SIZE];      /* 四电机与左右轮速度，rad/s */
} rl_observation_sample_t;
typedef struct {
    float obs[RL_OBS_SIZE];         /* 唯一网络输入，已按训练裁剪/缩放 */
    float last_action[RL_ACTION_SIZE]; /* 上一次网络原始请求，保持左右两侧顺序 */
    rl_observation_sample_t sample; /* 最新500Hz反馈，不主动添加观测延迟 */
    uint8_t sample_valid;           /* 最新反馈可用，坏反馈立即清除 */
    uint8_t valid;                  /* 本次obs39构建成功 */
} rl_observation_state_t;
uint8_t RL_Motor_Map(const machine_rl_cfg_t *config, const float position[4], const float velocity[4], const float wheel_velocity[2], uint8_t source_valid, rl_motor_state_t *out);
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3]);
void RL_Observation_Reset(rl_observation_state_t *state);
void RL_Observation_Sample(rl_observation_state_t *state, const float gyro[3], const float quat[4], const rl_motor_state_t *motor, uint8_t source_valid);
uint8_t RL_Observation_Read_Sample(const rl_observation_state_t *state, rl_observation_sample_t *sample);
void RL_Recovery_Reset(rl_recovery_state_t *state);
void RL_Recovery_Update(rl_recovery_state_t *state, uint8_t request, uint8_t feedback_valid, uint8_t fallen, float height);
uint8_t RL_Observation_Build(rl_observation_state_t *state, const machine_rl_cfg_t *config, const rl_observation_sample_t *sample, const rc_command_t *command, const rl_recovery_state_t *recovery, uint8_t source_valid);
void RL_Observation_Set_Last_Action(rl_observation_state_t *state, const float action[RL_ACTION_SIZE]);
#endif
