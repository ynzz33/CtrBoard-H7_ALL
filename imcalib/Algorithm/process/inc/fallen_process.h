/*
 * 文件职责：地面恢复动作状态机（fallen_process.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：IMU/pitch_world、双腿几何、执行许可和上一拍阶段。
 * 输出/去向：按阶段设置长度/摆角目标，运行独立PID，输出准备力矩及BALANCE/PREPARE/STOP路由。
 * 关键边界：阶段切换、目标生成、PID和映射分层；DONE才交给LQR；轮组准备输出保持零。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef FALLEN_PROCESS_H
#define FALLEN_PROCESS_H

#include "leg_solver.h"
#include "imu_state.h"
#include "torque_output.h"
#include "leg_compensation.h"
#include "pid.h"
#include "simple-function.h"

enum {
    FALLEN_PROCESS_IDLE = 0, /* 未开始/等待许可 */
    FALLEN_PROCESS_RETRACT, /* 收腿准备 */
    FALLEN_PROCESS_SWING, /* 摆腿使机身进入平衡姿态 */
    FALLEN_PROCESS_DONE, /* 准备完成，交给地面LQR */
    FALLEN_PROCESS_FAILED, /* 失败并按配置等待重试/停止 */
    FALLEN_PROCESS_REAR, /* 沿连续展开角路径后摆 */
    FALLEN_PROCESS_EXTEND, /* 先伸腿到准备长度 */
    FALLEN_PROCESS_SETTLE, /* 倒置恢复：先稳定姿态 */
    FALLEN_PROCESS_PRELOAD, /* 倒置恢复：建立腿长/力臂 */
    FALLEN_PROCESS_FLIP, /* 倒置恢复：翻转扫角 */
};

enum {
    FALLEN_PROCESS_BAD_INPUT = 1,
    FALLEN_PROCESS_TIMEOUT, /* 任务层统一撤力等待、重新初判；本模块不直接重试 */
    FALLEN_PROCESS_BAD_POSE,
    FALLEN_PROCESS_GATED,
    FALLEN_PROCESS_BAD_CONFIG,
};

enum {
    FALLEN_PROCESS_ROUTE_BALANCE = 0, /* 执行层选择LQR平衡结果 */
    FALLEN_PROCESS_ROUTE_PREPARE, /* 执行层选择自起独立准备力矩 */
    FALLEN_PROCESS_ROUTE_STOP, /* 执行层清输出，不能继续旧力矩 */
};

typedef struct {
    float recovery_settle_time;
    float recovery_len;
    float angle_rate;
    float recovery_pitch_tol;
    float recovery_stall_time;
    uint8_t recovery_retry_max;
    float retry_wait;
    float length_rate; /* 腿长m/s */
    float extend_len;
    float length_tol;
    float length_timeout;
    float rear_angle;
    float rear_tol;
    float angle_timeout;
    float retract_len;
    float angle_pos_kp;
    float angle_pos_kd;
    float angle_speed_kp;
    float angle_speed_kd;
    float angle_speed_max;
    float angle_tol;
    float stable_time;
    float support_time;
} fallen_process_param_t;

typedef struct {
    uint8_t enabled;
    uint8_t recovery_enabled;
    uint8_t retry;
    uint8_t recovered;
    uint8_t phase;
    uint8_t fault;
    float retry_elapsed;
    float elapsed;
    float stable_elapsed;
    float trigger_elapsed;
} fallen_process_lifecycle_t;

typedef struct {
    uint8_t rear_path_ready;
    float rear_position[2];   /* 展开角 */
    float rear_angle_target[2];
    float rear_last[2];
} fallen_process_normal_t;

typedef struct {
    float position[2];
    float last[2];
    float sweep;
    float stall_elapsed;
    float stall_upright;
} fallen_process_self_right_t;

typedef struct {
    uint8_t len_history_ready;
    uint8_t angle_history_ready;
    float support;
    float length_target[2];
    float angle_target[2];
    pid_t length_pid[2];
    ramp_t length_ramp[2]; /* 腿长目标 */
    pid_t angle_pos_pid[2];
    pid_t angle_speed_pid[2];
    compensation_debug_t compensation;
} fallen_process_actuator_t;

typedef struct {
    fallen_process_lifecycle_t lifecycle;
    fallen_process_normal_t normal;
    fallen_process_self_right_t self_right;
    fallen_process_actuator_t actuator;
} fallen_process_ctx_t;

extern fallen_process_ctx_t fallen_process_control;
extern fallen_process_param_t fallen_process_param;

void Fallen_Process_Init(fallen_process_ctx_t *st);
void Fallen_Process_Reset(fallen_process_ctx_t *st);
void Fallen_Process_Fail(fallen_process_ctx_t *st, uint8_t fault);
/* DONE输入平衡候选 */
uint8_t Fallen_Process_Update(fallen_process_ctx_t *st, const imu_state_t *imu,
                       const leg_state_t *left, const leg_state_t *right,
                       const leg_compensation_values_t *values,
                       uint8_t permit, float dt, torque_output_t *torque);

#endif


