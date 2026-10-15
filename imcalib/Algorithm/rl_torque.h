/* 六实体电机：模型坐标目标/PD/限幅，再还原固件力矩；无虚拟关节或补偿依赖。 */
#ifndef RL_TORQUE_H
#define RL_TORQUE_H
#include "rl_types.h"
#include "machine_config.h"
#include "torque_output.h"
typedef struct {
    torque_output_t last_torque;    /* 固件坐标力矩请求，尚未经过启动/故障门控 */
    float motor_torque[RL_ACTION_SIZE]; /* 模型坐标限幅力矩，Nm */
    float target[RL_ACTION_SIZE];   /* 本次使用的四个角度(rad)/两个轮速(rad/s)目标 */
    float pos[RL_LEG_MOTOR_NUM];    /* 本次PD当前模型角度，rad */
    float vel[RL_ACTION_SIZE];     /* 本次PD当前模型速度，rad/s */
} rl_torque_state_t;
/* action保持网络左右两侧顺序；内部目标/诊断为四腿两轮。只求解，最终许可归执行任务。 */
void RL_Torque_Reset(rl_torque_state_t *state);
uint8_t RL_Torque_Compute(const rl_motor_state_t *motor, const machine_rl_cfg_t *config, const float action[RL_ACTION_SIZE], rl_torque_state_t *state, torque_output_t *torque);
#endif
