/*
 * 文件职责：跨任务共享状态的声明集合（robot_state.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：robot_control.c中定义的IMU/电机/腿/RL/LQR和许可状态。
 * 输出/去向：给各任务提供统一extern声明、原始动作/机器人/RL运行状态类型。
 * 关键边界：该头包含共享对象，算法层应接收参数而不是包含这个任务头；同帧读写使用快照接口。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
    float a[RL_ACTION_SIZE];    /* 原始网络顺序[LF前,LF后,L轮,RF前,RF后,R轮]，未缩放 */
    uint32_t reset_count;       /* 动作清零次数：拒绝清零前已在途、清零后才返回的推理 */
    uint32_t last_ok_tick;      /* 最近一次有效动作发布时间，ms */
    uint8_t updated;            /* 动作已发布，看门狗可开始检查更新时间 */
    uint8_t rl_ready;               /* 推理动作可用 */
} action_state_t;

typedef struct {
    int8_t dm_front;
    int8_t dm_rear;
} leg_map_t;

typedef struct {
    uint8_t rc_enable;
    uint8_t motor_enabled;
    uint8_t fallen; /* 公共姿态倒地：IMU俯仰或横滚角任一超过40度，不包含高度。 */
    float height; /* 公共髋轮投影间距m；通信发布，RL与LQR触地判断共用。 */
    uint8_t height_valid;
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
extern volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
extern volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];
extern uint8_t torque_output_enabled;
extern volatile compensation_debug_t compensation_debug;

#endif
