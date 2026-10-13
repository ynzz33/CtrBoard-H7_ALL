/*
 * 文件职责：姿态归一化与竖直加速度计算（Attitude_Algorithm.c）。
 * 输入/来源：已由task_imu按机器表映射的四元数、欧拉角和加速度。
 * 输出/去向：输出归一化姿态、rad角度和世界竖直加速度，供LQR/离地估计使用。
 * 关键边界：本文件不解析HI229字节；两机竖直映射沿用源码，不在此重新标定符号。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "Attitude_Algorithm.h"
#include "machine_config.h"
#include <math.h>
#include <string.h>

#define ATTITUDE_DEG_TO_RAD  0.01745329251994f
#define ATTITUDE_QUAT_NORM_MIN  0.001f

/* 世界竖直加速度 */
bool Attitude_Accel_Vertical(const imu_state_t *state, float *acceleration)
{
    float r20;
    float r21;
    float r22;
    float specific;
    const float *q;

    *acceleration = 0.0f;
    q = state->quat;
    r20 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    r21 = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    r22 = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    specific = r20 * state->acc_g[0] - r21 * state->acc_g[1] - r22 * state->acc_g[2];
    *acceleration = (specific - 1.0f) * 9.81f;
#else
    specific = r20 * state->acc_g[0] + r21 * state->acc_g[1] + r22 * state->acc_g[2];
    *acceleration = -(specific + 1.0f) * 9.81f;
#endif
    return isfinite(*acceleration);
}

/* 初始化姿态 */
void Attitude_Init(imu_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->quat[0] = 1.0f;
}

/* 归一化、单位换算、竖直加速度及校验；调用方按返回值发布online。 */
bool Attitude_Update(imu_state_t *state)
{
    float q_norm;


    /* 归一化四元数 */
    q_norm = sqrtf(state->quat[0] * state->quat[0] + state->quat[1] * state->quat[1] + state->quat[2] * state->quat[2] + state->quat[3] * state->quat[3]);
    if (!isfinite(q_norm) || q_norm < ATTITUDE_QUAT_NORM_MIN)
        return false;

    for (uint8_t axis = 0u; axis < 4u; axis++)
    {
        state->quat[axis] /= q_norm;
    }

    /* 欧拉角 deg → rad */
    for (uint8_t axis = 0u; axis < 3u; axis++)
    {
        state->euler_rad[axis] = state->euler_deg[axis] * ATTITUDE_DEG_TO_RAD;
        if (!isfinite(state->euler_rad[axis]) || !isfinite(state->gyro_rad_s[axis]) || !isfinite(state->acc_g[axis]))
        {
            return false;
        }
    }

    return Attitude_Accel_Vertical(state, &state->accel_vertical);
}
