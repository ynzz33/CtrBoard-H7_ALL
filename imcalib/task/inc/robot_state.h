/*
 * 声明跨任务共享的反馈、遥控、原始action、LQR/RL状态和输出诊断。
 * 具体存储定义在robot_control.c；反馈批量读取使用feedback_snapshot接口，避免结构读到一半。
 */

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "robot_control.h"
#include "imu_state.h"
#include "feedback_snapshot.h"
#include "leg_solver.h"
#include "rl_torque.h"
#include "lqr_balance.h"
#include "leg_balance.h"







typedef struct {
    float a[RL_ACTION_SIZE];    /* 1999原始顺序[LF前,LF后,RF前,RF后,L轮,R轮]，未缩放 */
    uint32_t reset_count;       /* 动作清零次数：拒绝清零前已在途、清零后才返回的推理 */
    uint32_t last_ok_tick;      /* 最近一次有效动作发布时间，ms */
    uint8_t updated;            /* 动作已发布，看门狗可开始检查更新时间 */
    uint8_t rl_ready;               /* 推理动作可用 */
} action_state_t;

typedef struct {
    int8_t dm_front; /* 该腿前髋在DM数组中的索引。 */
    int8_t dm_rear; /* 该腿后髋在DM数组中的索引。 */
} leg_map_t;

typedef struct {
    uint8_t motor_enabled; /* 通信任务维护的电机使能请求状态，不等于四髋反馈均已使能。 */
    uint8_t fallen; /* 公共FALLEN选路归属，恢复完成重新选路解除；非RL倾角判据。 */
    float height; /* 公共髋轮投影间距m；通信发布，RL与LQR触地判断共用。 */
    uint8_t height_valid; /* 公共投影高度是否有效。 */
} robot_state_t;

/* 最新采样/PD归执行任务，观测/原始action历史归策略任务；参数直接读机器表。 */
typedef struct {
    rl_observation_state_t observation;
    rl_policy_t policy;
    rl_torque_state_t torque_state;
    rl_recovery_state_t recovery; /* 执行任务推进，策略只复制；阶段供Watch观察。 */
} rl_control_state_t;

/* 写入归对应任务 */
extern imu_state_t imu_state;
extern motor_state_t motor_state;
extern leg_state_t leg_l;
extern leg_state_t leg_r;
extern action_state_t action_state;
extern rc_command_t rc_command;
extern leg_map_t leg_map_l;
extern leg_map_t leg_map_r;
extern robot_state_t robot_state;
extern rl_control_state_t rl_control;
extern lqr_state_t lqr_state;
extern leg_balance_t leg_balance;
extern volatile ctrl_strategy_t ctrl_strategy;
extern volatile uint32_t ctrl_fault;
extern volatile uint8_t output_debug_dm_sent;
extern volatile uint8_t output_debug_dji_sent;
extern volatile control_time_debug_t control_time_debug;
extern volatile float output_dm_torque_nm[DM_MOTOR_NUM];
extern volatile float output_wheel_torque_nm[DJI_MOTOR_NUM];
extern uint8_t torque_output_enabled;
extern volatile compensation_debug_t compensation_debug;

#endif
