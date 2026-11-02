/*
 * 根据双腿角、世界俯仰和四元数重力朝向，发布valid、abnormal、inverted。
 * abnormal是瞬时异常候选；连续确认和公共FALLEN归属由control_condition及执行层维护。
 */
/* 公共倒地检测；判据参数由本模块维护，不依赖恢复动作模块。 */
#include "fallen_detection.h"
#include "machine_config.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>

fallen_detection_t fallen_detection;
fallen_detection_param_t fallen_detection_param = {
    .trigger_angle = 55.0f * LEG_PI / 180.0f,
    .trigger_pitch = 55.0f * LEG_PI / 180.0f,
    .trigger_time = 0.1f,
    .pitch_max = 1.0f,
};

void Fallen_Detection_Reset(fallen_detection_t *state)
{
    memset(state, 0, sizeof(*state));
}

/* 校验IMU和双腿，计算重力朝向upright、倒置inverted及腿角/俯仰瞬时异常abnormal。
 * 本函数不累计trigger_time，不决定何时撤力或启动自起。 */
void Fallen_Detection_Update(fallen_detection_t *state, const imu_state_t *imu,
                             const leg_state_t *left, const leg_state_t *right)
{
    float norm;
    float left_error;
    float right_error;
    float pitch_error;

    state->valid = 0u;
    if (!imu->online || !imu->pitch_world_valid || !left->output.valid || !right->output.valid)
    {
        return;
    }
    norm = imu->quat[0] * imu->quat[0] + imu->quat[1] * imu->quat[1] + imu->quat[2] * imu->quat[2] + imu->quat[3] * imu->quat[3];
    state->upright = 1.0f - 2.0f * (imu->quat[1] * imu->quat[1] + imu->quat[2] * imu->quat[2]) / norm;
    if (!isfinite(state->upright))
    {
        return;
    }
    state->valid = 1u;
    state->inverted = (uint8_t)(state->upright < 0.0f || fabsf(imu->pitch_world) > fallen_detection_param.pitch_max);
    left_error = Angle_Wrap_Pi(left->output.virtual_leg_angle - machine->lqr.reference.leg_trim[0]);
    right_error = Angle_Wrap_Pi(right->output.virtual_leg_angle - machine->lqr.reference.leg_trim[1]);
    pitch_error = Angle_Wrap_Pi(imu->pitch_world - machine->lqr.reference.pitch_trim);
    state->abnormal = (uint8_t)(fabsf(left_error) > fallen_detection_param.trigger_angle || fabsf(right_error) > fallen_detection_param.trigger_angle || fabsf(pitch_error) > fallen_detection_param.trigger_pitch);
}


