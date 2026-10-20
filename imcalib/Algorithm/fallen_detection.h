/*
 * 公共倒地姿态检测：发布反馈有效性、重力朝向、倒置和普通异常姿态。
 * 只检测，不推进恢复阶段、不生成力矩。
 */
#ifndef FALLEN_DETECTION_H
#define FALLEN_DETECTION_H

#include "imu_state.h"
#include "leg_solver.h"

typedef struct {
    float trigger_angle; /* 相对站立腿角异常阈值rad */
    float trigger_pitch; /* 相对站立俯仰异常阈值rad */
    float trigger_time;  /* 普通异常连续确认时间s */
    float pitch_max;     /* 倒置及恢复姿态保护阈值rad */
} fallen_detection_param_t;

typedef struct {
    float upright;       /* 四元数重力朝向，负值表示倒置 */
    uint8_t valid;       /* IMU、世界pitch和双腿几何均有效 */
    uint8_t inverted;    /* 倒置或世界pitch超过原翻身阈值 */
    uint8_t abnormal;    /* 原普通起立腿角/俯仰瞬时异常 */
} fallen_detection_t;

extern fallen_detection_t fallen_detection;
extern fallen_detection_param_t fallen_detection_param;

void Fallen_Detection_Reset(fallen_detection_t *state);
void Fallen_Detection_Update(fallen_detection_t *state, const imu_state_t *imu,
                             const leg_state_t *left, const leg_state_t *right);

#endif
