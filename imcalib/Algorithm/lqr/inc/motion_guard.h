/*
 * 用实际车速、腿长/高度及增益比估计NORMAL偏航参考上限。
 * 收紧立即生效，解除按release_rate逐步放宽；只修改参考，不直接裁剪电机力矩。
 */
#ifndef MOTION_GUARD_H
#define MOTION_GUARD_H
#include <stdint.h>
typedef struct {
    uint8_t enabled;
    float budget_fraction; /* 试用预算倍率，非物理标定 */
    float release_rate; /* 允许偏航参考上限放宽速率rad/s² */
} motion_guard_param_t;
typedef struct {
    float length[2], length_target;
    float velocity, velocity_request, yaw_rate;
    float yaw_request, heading_error;
    float heading_gain_diff, rate_gain_diff;
    float track, height_offset, yaw_max, dt;
    uint8_t permit;
} motion_guard_input_t;
enum { MOTION_GUARD_OFF=0, MOTION_GUARD_READY=1, MOTION_GUARD_LIMITED=2,
       MOTION_GUARD_RELEASING=4, MOTION_GUARD_INVALID=8 };
typedef struct {
    float length_guard, height_reference, speed_guard, lateral_budget;
    float yaw_limit_raw, yaw_limit, yaw_equivalent_raw, yaw_equivalent;
    float reference_scale, heading_ratio, lateral_actual;
    uint8_t initialized, valid, limited, releasing, reason;
} motion_guard_t;
extern motion_guard_t motion_guard;
void Motion_Guard_Reset(motion_guard_t *state);
/* 0表示新计算无效，调用方沿原无效候选路径停止本拍；退出/关闭返回1且不干预。 */
uint8_t Motion_Guard_Update(motion_guard_t *state, const motion_guard_param_t *param,
                            const motion_guard_input_t *input);
#endif
