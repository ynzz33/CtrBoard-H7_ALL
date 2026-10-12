/*
 * 文件职责：自起准备与倒置自救状态机（standup.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：IMU/pitch_world、双腿几何、执行许可和上一拍阶段。
 * 输出/去向：按阶段设置长度/摆角目标，运行独立PID，输出准备力矩及BALANCE/PREPARE/STOP路由。
 * 关键边界：阶段切换、目标生成、PID和映射分层；DONE才交给LQR；轮组准备输出保持零。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef STANDUP_H
#define STANDUP_H

#include "leg_solver.h"
#include "imu_state.h"
#include "torque_output.h"
#include "leg_compensation.h"
#include "pid.h"
#include "simple-function.h"

enum {
    STANDUP_IDLE = 0, /* 未开始/等待许可 */
    STANDUP_RETRACT, /* 收腿准备 */
    STANDUP_SWING, /* 摆腿使机身进入平衡姿态 */
    STANDUP_DONE, /* 准备完成，交给地面LQR */
    STANDUP_FAILED, /* 失败并按配置等待重试/停止 */
    STANDUP_REAR, /* 沿连续展开角路径后摆 */
    STANDUP_EXTEND, /* 先伸腿到准备长度 */
    STANDUP_SETTLE, /* 倒置恢复：先稳定姿态 */
    STANDUP_TUCK, /* 倒置恢复：收拢腿部 */
    STANDUP_FLIP, /* 倒置恢复：翻转扫角 */
};

enum {
    STANDUP_POSE_NORMAL = 0,
    STANDUP_POSE_INVERTED,
    STANDUP_POSE_INVALID = 3,
};

enum {
    STANDUP_OK = 0,
    STANDUP_BAD_INPUT,
    STANDUP_TIMEOUT,
    STANDUP_BAD_POSE,
    STANDUP_GATED,
    STANDUP_BAD_CONFIG,
};

enum {
    STANDUP_ROUTE_BALANCE = 0, /* 执行层选择LQR平衡结果 */
    STANDUP_ROUTE_PREPARE, /* 执行层选择自起独立准备力矩 */
    STANDUP_ROUTE_STOP, /* 执行层清输出，不能继续旧力矩 */
};

typedef struct {
    float recovery_settle_time;
    float recovery_tuck_time;
    float recovery_len;
    float recovery_angle_rate;
    float recovery_force_max;
    float recovery_ready_pitch;
    float recovery_timeout;
    float recovery_stall_time;
    uint8_t recovery_retry_max;
    float retry_wait;
    float length_rate; /* 腿长m/s */
    float extend_len;
    float extend_tol;
    float extend_timeout;
    float rear_angle;
    float rear_tol;
    float rear_rate;
    float rear_timeout;
    float retract_len;
    float retract_ready_len;
    float angle_pos_kp;
    float angle_pos_kd;
    float angle_speed_kp;
    float angle_speed_kd;
    float angle_speed_max;
    float tp_max;
    float trigger_angle;
    float trigger_pitch;
    float angle_tol;
    float pitch_max;
    float stable_time;
    float support_time;
    float trigger_time;
    float timeout[2];
} standup_param_t;

typedef struct {
    uint8_t enabled;
    uint8_t recovery_enabled;
    uint8_t pose;
    uint8_t retry;
    uint8_t recovered;
    uint8_t phase;
    uint8_t fault;
    uint8_t need;
    uint8_t len_history_ready;
    uint8_t angle_history_ready;
    uint8_t rear_path_ready;
    uint16_t ready_block;      /* 到位阻塞 */
    float retry_elapsed;
    float elapsed;
    float stable;
    float trigger_elapsed;
    float support;
    float length_cmd[2];
    float angle_cmd[2];
    float angle_speed_cmd[2];
    float rear_position[2];   /* 展开角 */
    float rear_goal[2];
    float rear_last[2];
    float recovery_position[2];
    float recovery_last[2];
    float recovery_sweep;
    float stall_elapsed;
    float stall_upright;
    float upright;
    float force[2];
    float tp[2];
    pid_t length_pid[2];
    ramp_t length_ramp[2]; /* 腿长目标 */
    pid_t angle_pos_pid[2];
    pid_t angle_speed_pid[2];
    torque_output_t prepare;
    compensation_debug_t compensation;
    float raw_dm[DM_MOTOR_NUM];   /* 限幅前 */
} standup_ctx_t;

extern standup_ctx_t standup_control;
extern standup_param_t standup_param;

void Standup_Init(standup_ctx_t *st);
void Standup_Reset(standup_ctx_t *st);
void Standup_Fail(standup_ctx_t *st, uint8_t fault);
/* DONE输入平衡候选 */
uint8_t Standup_Update(standup_ctx_t *st, const imu_state_t *imu,
                       const leg_state_t *left, const leg_state_t *right,
                       uint8_t permit, uint8_t allow_restart,
                       float dt, torque_output_t *torque);

#endif
