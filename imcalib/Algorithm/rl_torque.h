/*
 * 文件职责：RL动作到虚拟关节及实体电机力矩（rl_torque.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：6维动作、双腿几何、实测关节/轮速度、机器RL映射与控制器参数。
 * 输出/去向：构造关节目标、计算控制力矩、映射髋电机并补弹簧/限幅，输出torque_output_t。
 * 关键边界：动作是训练角色量而非直接CAN值；轮组使用本路径自己的速度控制，不套用LQR输出。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef RL_TORQUE_H
#define RL_TORQUE_H

#include <stdint.h>

#include "motor_types.h"
#include "leg_solver.h"
#include "pid.h"
#include "rl_types.h"
#include "rl_observation.h"
#include "torque_output.h"
#include "gas_spring.h"

typedef struct {
    float dof_pos[6];
    float p_gains[4];      /* 四腿关节 */
    float d_gains[4];
    machine_pid_cfg_t wheel_pid[2]; /* 左右轮 */
    float leg_pid_max_output;
    float pos_scale;
    float wheel_vel_scale;
} rl_torque_param_t;

typedef struct {
    torque_output_t last_torque;
    float virtual_torque[RL_ACTION_SIZE];
    float pos_target[RL_ACTION_SIZE];
    pid_t controller[RL_ACTION_SIZE];
} rl_torque_state_t;

void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model,
                          const machine_rl_cfg_t *config);
void RL_Torque_State_Init(rl_torque_state_t *state,
                          const rl_torque_param_t *param);
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const gas_spring_result_t *spring,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          torque_output_t *torque);

#endif
