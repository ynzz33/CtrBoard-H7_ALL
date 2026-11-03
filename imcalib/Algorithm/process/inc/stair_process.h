/* 台阶单次动作及独立退出阶段；不访问任务全局、自起或AIR处理。 */
#ifndef STAIR_PROCESS_H
#define STAIR_PROCESS_H

#include "leg_solver.h"
#include "leg_compensation.h"
#include "torque_output.h"
#include "pid.h"
#include "simple-function.h"

enum {
    STAIR_PROCESS_IDLE = 0, /* 无台阶输出归属 */
    STAIR_PROCESS_SWING, /* 保持腿长/支撑，双腿摆向已确认目标 */
    STAIR_PROCESS_RETRACT, /* 收腿、释放基础摆矩 */
    STAIR_PROCESS_WAIT, /* 失败撤力，独立等待 */
    STAIR_PROCESS_RECHECK, /* 零输出，等待任务层重新初判 */
};

enum {
    STAIR_RESULT_IDLE = 0, STAIR_RESULT_RUNNING, STAIR_RESULT_DONE,
    STAIR_RESULT_FAILED, STAIR_RESULT_CANCELLED,
};

enum {
    STAIR_EXIT_NONE = 0, STAIR_EXIT_COMPLETE, STAIR_EXIT_CANCELLED,
    STAIR_EXIT_INPUT, STAIR_EXIT_MAPPING, STAIR_EXIT_SWING_TIMEOUT,
    STAIR_EXIT_RETRACT_TIMEOUT, STAIR_EXIT_TOTAL_TIMEOUT, STAIR_EXIT_STALL,
};

/* 由任务层从自起参数源组装；只共享参数，不包含自起状态或实例。 */
typedef struct {
    const machine_pid_cfg_t *length_pd;
    machine_pid_cfg_t angle_pos, angle_speed;
    float length_rate, angle_rate; /* m/s、rad/s */
    float support_time, support_angle_range; /* s、rad */
    const float *leg_trim;
    const float *support_force;
} stair_control_param_t;

typedef struct {
    const leg_state_t *leg[2]; /* 本拍左右腿，已发布几何及速度 */
    const stair_control_param_t *control; /* 本拍同源控制参数，只读 */
    const leg_compensation_values_t *compensation; /* 本拍世界姿态重力/弹簧缓存 */
    float hip_limit; /* 公共最终髋限矩N·m */
    uint32_t now_ms; /* 真实时钟，允许uint32回绕 */
    uint8_t permit; /* 基础出力、四髋使能及LQR投入许可；不检查AIR/FALLEN */
} stair_process_input_t;

typedef struct {
    uint8_t phase, result, exit_reason;
    uint8_t recheck_pending; /* 退出后只请求一次检测历史复位 */
    uint32_t sequence, begin_ms, phase_begin_ms, stall_begin_ms[2];
    float elapsed, total_elapsed, stable_elapsed; /* 阶段、动作及连续到位时间s */
    float position[2], last_angle[2]; /* 独立连续几何角及上一包角rad */
    float angle_target[2], length_target[2]; /* 轨迹给定rad、m */
    float swing_length[2]; /* 进入时锁定的实测腿长m，仅摆腿阶段保持 */
    pid_t length_pid[2], angle_pos_pid[2], angle_speed_pid[2]; /* 台阶独立历史 */
    ramp_t length_ramp[2]; /* 与自起同一斜坡算法，实例独立 */
    uint8_t controllers_ready;
    float support; /* 支撑渐入0～1 */
    float stall_remaining[2]; /* 各腿卡滞窗口起点的剩余角/长度误差 */
    float force[2], torque[2]; /* 基础轴向力N、摆矩N·m，不含物理补偿 */
    compensation_debug_t compensation;
} stair_process_t;

extern stair_process_t stair_process;

void Stair_Process_Init(stair_process_t *state);
void Stair_Process_Reset(stair_process_t *state);
uint8_t Stair_Process_Owns_Output(const stair_process_t *state);
uint8_t Stair_Process_Begin(stair_process_t *state, const stair_process_param_t *param, const stair_process_input_t *input);
void Stair_Process_Abort(stair_process_t *state, uint8_t reason, uint32_t now_ms);
/* WAIT到RECHECK/完成到RECHECK只返回一次1，调用者据此复位公共检测历史。 */
uint8_t Stair_Process_Advance_Exit(stair_process_t *state, const stair_process_param_t *param, uint32_t now_ms);
/* 每个动作执行拍一次；先清候选，成功才发布有效结果，退出当拍保持零候选。 */
void Stair_Process_Update(stair_process_t *state, const stair_process_param_t *param, const stair_process_input_t *input, float dt, torque_output_t *output);

#endif
