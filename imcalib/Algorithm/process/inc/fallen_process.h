/*
 * 根据倒地姿态规划普通前/后侧自起或倒置翻身，推进阶段并生成腿长/摆角目标。
 * 独立长度/角度控制器输出准备力矩；返回PREPARE/BALANCE/STOP供执行层决定输出。
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
    FALLEN_PROCESS_REAR, /* 沿连续展开角路径对齐前/后准备位 */
    FALLEN_PROCESS_EXTEND, /* 先伸腿到准备长度 */
    FALLEN_PROCESS_SETTLE, /* 倒置恢复：先稳定姿态 */
    FALLEN_PROCESS_PRELOAD, /* 倒置恢复：建立腿长/力臂 */
    FALLEN_PROCESS_FLIP, /* 倒置恢复：翻转扫角 */
    FALLEN_PROCESS_ALIGN_FRONT, /* 倒置恢复：双腿先对齐前点；保留原阶段编号 */
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
    uint8_t enabled; /* 允许普通地面自起功能，不表示正在执行。 */
    uint8_t recovery_enabled; /* 允许倒置翻身功能。 */
    uint8_t retry; /* 已消耗的地面恢复重试次数。 */
    uint8_t recovered; /* 翻身回正事件，供LQR交接时重建历史。 */
    uint8_t phase; /* 当前动作阶段，取FALLEN_PROCESS枚举。 */
    uint8_t fault; /* 失败原因，取FALLEN_PROCESS故障枚举。 */
    float retry_elapsed; /* 非超时失败的重试等待时间s；超时由任务层撤力重判。 */
    float elapsed; /* 当前阶段已运行时间s，转阶段时清零。 */
    float stable_elapsed; /* 当前到位条件连续成立时间s，中断清零。 */
    float trigger_elapsed; /* 普通异常触发连续时间s。 */
} fallen_process_lifecycle_t;

typedef struct {
    uint8_t planned; /* 普通入口/翻身回正后置1；翻身期间清0 */
    uint8_t path[2]; /* 0已在摆正窗口，1前侧，2后侧 */
    float prepare_angle[2]; /* 本次锁定准备角，几何角表示 */
    uint8_t rear_path_ready; /* 连续腿角历史是否已初始化。 */
    float rear_position[2]; /* 左右腿连续展开角rad，避免跨±pi时路径跳变。 */
    float rear_angle_target[2]; /* 左右准备位在连续角分支中的目标rad。 */
    float rear_last[2]; /* 上一拍包角反馈rad，用于累计连续角。 */
} fallen_process_normal_t;

typedef struct {
    float position[2]; /* 翻身路径左右腿连续展开反馈rad。 */
    float last[2]; /* 翻身路径上一拍包角反馈rad。 */
    float front_goal[2]; /* 前点在各腿连续角分支中的锁定目标rad */
    float sweep; /* 本次FLIP累计命令扫角rad，用于整圈上限。 */
    float stall_elapsed; /* 腿角误差大且运动不足的连续时间s。 */
    float stall_upright; /* 卡滞计时起点的重力朝向，用于检查机身是否仍在回正。 */
} fallen_process_self_right_t;

typedef struct {
    uint8_t len_history_ready; /* 长度控制器历史是否按本段实测初始化。 */
    uint8_t angle_history_ready; /* 摆角控制器历史是否按本段实测初始化。 */
    float support; /* 自起支撑前馈渐入系数，0至1。 */
    float length_target[2]; /* 左右腿长度指令m。 */
    float angle_target[2]; /* 左右腿摆角指令rad，连续/包角解释由阶段决定。 */
    pid_t length_pid[2]; /* 左右自起长度控制器，与正常平衡历史分开。 */
    ramp_t length_ramp[2]; /* 腿长目标 */
    pid_t angle_pos_pid[2]; /* 左右摆角外环，输出角速度目标。 */
    pid_t angle_speed_pid[2]; /* 左右角速度内环，输出虚拟腿摆矩。 */
    compensation_debug_t compensation;
} fallen_process_actuator_t;

typedef struct {
    fallen_process_lifecycle_t lifecycle;
    fallen_process_normal_t normal;
    fallen_process_self_right_t self_right;
    fallen_process_actuator_t actuator;
} fallen_process_ctx_t;

extern fallen_process_ctx_t fallen_process_control;

void Fallen_Process_Init(fallen_process_ctx_t *st);
void Fallen_Process_Reset(fallen_process_ctx_t *st);
/* 已在准备位：独立上下文从普通收腿接入，不重新选路、不启动倒置翻身。 */
uint8_t Fallen_Process_Begin_Retract(fallen_process_ctx_t *st, const imu_state_t *imu, const leg_state_t *left, const leg_state_t *right);
void Fallen_Process_Fail(fallen_process_ctx_t *st, uint8_t fault);
/* DONE输入平衡候选；angle_rate为调用方普通路径角速率，倒置阶段使用独立recovery_angle_rate。 */
uint8_t Fallen_Process_Update(fallen_process_ctx_t *st, const imu_state_t *imu,
                       const leg_state_t *left, const leg_state_t *right,
                       const leg_compensation_values_t *values,
                       uint8_t permit, float angle_rate, float dt, torque_output_t *torque);

#endif


