/*
 * 公共触地候选与LQR三态选路；只发布判断，不生成目标或力矩。
 * 几何高度使用公共height，恢复触发沿用standup_param；状态与处理阶段分开维护。
 */
#ifndef CONTROL_CONDITION_H
#define CONTROL_CONDITION_H

#include "air_detection.h"
#include "imu_state.h"
#include "leg_solver.h"

typedef enum {
    CONTROL_NORMAL = 0,
    CONTROL_FALLEN,
    CONTROL_FLIGHT
} control_condition_type_t;

typedef struct {
    float body_contact_height; /* 公共投影高度门槛，m */
    float confirm_time; /* 零出力触地候选连续时间，s */
} control_condition_param_t;

typedef struct {
    control_condition_type_t condition;
    float contact_elapsed; /* 连续触地候选时间，s */
    float trigger_elapsed; /* NORMAL原姿态异常连续时间，s */
    float upright; /* 原自起四元数重力朝向指标 */
    uint8_t pose_valid;
    uint8_t hint_available;
    uint8_t body_contact_candidate;
    uint8_t body_contact_confirmed;
    uint8_t inverted;
    uint8_t abnormal; /* 原腿角/俯仰门槛瞬时结果，用于初判 */
    uint8_t prepare_required; /* NORMAL原异常持续到时 */
    uint8_t started; /* 已完成本次投入的初判，与反馈valid不同 */
    uint8_t valid; /* 本拍判断可用于选路；0不代表NORMAL */
    uint8_t normal_ready; /* 上拍已生成有效NORMAL候选，不表示电机已执行 */
} control_condition_t;

extern control_condition_param_t control_condition_param;
extern control_condition_t control_condition;

void Control_Condition_Reset(control_condition_t *state);
void Control_Condition_Observe(control_condition_t *state, const imu_state_t *imu,
                               const leg_state_t *left, const leg_state_t *right,
                               float height, uint8_t height_valid,
                               uint8_t zero_effort, uint8_t normal_running);
void Control_Condition_Select(control_condition_t *state, const air_detection_t *air,
                              uint8_t request, uint8_t drive,
                              uint8_t fallen, uint8_t recovery_done);
/* 原恢复动作返回BALANCE后调用；检测尚未确认则等待，不重启动作。 */
void Control_Condition_Recovery_Ready(control_condition_t *state, const air_detection_t *air);

#endif
