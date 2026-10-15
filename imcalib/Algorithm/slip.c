/*
 * 文件职责：带加速度状态的速度融合与异常轮速降权（slip.c）。
 * 输入/来源：轮速运动学观测、IMU前向加速度、dt和接触许可。
 * 输出/去向：维护速度/加速度及协方差；AIR停用轮速修正但保留预测/IMU修正。
 * 关键边界：不发布独立打滑状态；降权不是主动轮速制动，接触标志由执行层传入。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

/* 两状态速度补偿：加速度观测、轮速异常降权，不判打滑状态。 */
#include "slip.h"
#include <math.h>
#include <string.h>

#define SLIP_NOISE_DT 0.001f

slip_param_t slip_param = {
    .p0_velocity     = 0.1f,   /* (m/s)² */
    .p0_acceleration = 0.25f,  /* (m/s²)² */
    .q_velocity      = 0.007f, /* 每1ms */
    .q_acceleration  = 0.01f,  /* 每1ms */
    .r_velocity      = 0.01f,  /* (m/s)² */
    .r_acceleration  = 0.25f,  /* (m/s²)² */
    .gate            = 3.0f,   /* σ倍数 */
    .gate_min        = 0.4f,   /* m/s */
};

/* 重新等待有效先验 */
void Slip_Reset(slip_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

/* 已验证反馈建先验；初始方差直接取固定参数，只检查新求出的门限。 */
void Slip_Init(slip_state_t *st, float v0, float a0)
{
    Slip_Reset(st);
    st->velocity = v0;
    st->acceleration = a0;
    st->covariance[0][0] = slip_param.p0_velocity;
    st->covariance[1][1] = slip_param.p0_acceleration;
    st->noise_scale = 1.0f;
    st->limit = fmaxf(slip_param.gate_min, slip_param.gate * sqrtf(st->covariance[0][0] + slip_param.r_velocity));
    st->initialized = 1u;
    st->valid = (uint8_t)(isfinite(st->limit) && st->limit > 0.0f);
}

/* 单观测Joseph修正 */
static uint8_t Slip_Correct(slip_state_t *st, uint8_t axis,
                            float innovation, float noise)
{
    float gain[2];
    float transform[2][2];
    float result[2][2];
    float variance;
    float cross;

    variance = st->covariance[axis][axis] + noise;
    /* 方差由当前协方差和噪声计算，是实际除数；这里只检查运算是否可用。 */
    if (!isfinite(variance) || variance <= 0.0f)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        gain[i] = st->covariance[i][axis] / variance;
        for (uint8_t j = 0u; j < 2u; j++)
        {
            transform[i][j] = (i == j ? 1.0f : 0.0f) - (j == axis ? gain[i] : 0.0f);
        }
    }
    st->velocity += gain[0] * innovation;
    st->acceleration += gain[1] * innovation;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        for (uint8_t j = 0u; j < 2u; j++)
        {
            result[i][j] = gain[i] * noise * gain[j];
            for (uint8_t m = 0u; m < 2u; m++)
            {
                for (uint8_t n = 0u; n < 2u; n++)
                {
                    result[i][j] += transform[i][m] * st->covariance[m][n] * transform[j][n];
                }
            }
        }
    }
    cross = 0.5f * (result[0][1] + result[1][0]);
    result[0][1] = cross;
    result[1][0] = cross;
    memcpy(st->covariance, result, sizeof(result));
    return (uint8_t)(isfinite(st->velocity) && isfinite(st->acceleration) && isfinite(result[0][0]) && isfinite(result[1][1]) && isfinite(cross) && result[0][0] >= 0.0f && result[1][1] >= 0.0f);
}

/* 预测、加速度修正、轮速降权修正 */
uint8_t Slip_Update_Contact(slip_state_t *st, float wheel_v, float imu_acc,
                            float dt, uint8_t wheel_contact)
{
    float p00;
    float p01;
    float p10;
    float p11;
    float scale;
    float ratio;
    float noise;

    st->valid = 0u;
    if (!st->initialized)
    {
        Slip_Init(st, wheel_contact ? wheel_v : 0.0f, imu_acc);
        return st->valid;
    }
    scale = dt / SLIP_NOISE_DT;
    p00 = st->covariance[0][0] + dt * (st->covariance[0][1] + st->covariance[1][0]) + dt * dt * st->covariance[1][1] + slip_param.q_velocity * scale;
    p01 = st->covariance[0][1] + dt * st->covariance[1][1];
    p10 = st->covariance[1][0] + dt * st->covariance[1][1];
    p11 = st->covariance[1][1] + slip_param.q_acceleration * scale;
    st->velocity += st->acceleration * dt;
    st->covariance[0][0] = p00;
    st->covariance[0][1] = p01;
    st->covariance[1][0] = p10;
    st->covariance[1][1] = p11;
    if (!Slip_Correct(st, 1u, imu_acc - st->acceleration, slip_param.r_acceleration))
    {
        Slip_Reset(st);
        return 0u;
    }
    if (!wheel_contact)
    {
        st->innovation = 0.0f;
        st->noise_scale = 1.0f;
        st->valid = 1u;
        return 1u;
    }
    st->innovation = wheel_v - st->velocity;
    st->limit = fmaxf(slip_param.gate_min, slip_param.gate * sqrtf(st->covariance[0][0] + slip_param.r_velocity));
    ratio = fabsf(st->innovation) / st->limit;
    st->noise_scale = fmaxf(1.0f, ratio * ratio);
    noise = slip_param.r_velocity * st->noise_scale;
    if (!isfinite(st->limit) || st->limit <= 0.0f || !Slip_Correct(st, 0u, st->innovation, noise))
    {
        Slip_Reset(st);
        return 0u;
    }
    st->valid = 1u;
    return 1u;
}
