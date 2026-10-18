/*
 * 文件职责：利用重力方向区分正常/倒置的俯仰补角（pitch_world.c）。
 * 输入/来源：已由Attitude_Update验证的欧拉pitch和四元数。
 * 输出/去向：返回[-pi,pi)内pitch_world及成功标志，供自起/跌倒相关处理。
 * 关键边界：先按最大分量缩放再归一化；重力z决定补角；不重写LQR原pitch或IMU零点。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

// 版本：v1.2  日期：2026-10-05
#include "pitch_world.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define PITCH_WORLD_PI                 3.14159265358979323846f
#define PITCH_WORLD_2PI                6.28318530717958647692f
#define PITCH_WORLD_HALF_PI            1.5707963267948966192f

/* 重力判向补角 */
bool Pitch_World_Calc(float pitch, const float quat[4], float *pitch_world)
{
    float q[4];
    float scale;
    float norm;
    float gravity_z;
    float angle;

    if (fabsf(pitch) > PITCH_WORLD_HALF_PI)
    {
        return false;
    }

    scale = 0.0f;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        scale = fmaxf(scale, fabsf(quat[i]));
    }

    /* 缩放防溢出 */
    norm = 0.0f;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        q[i] = quat[i] / scale;
        norm += q[i] * q[i];
    }
    norm = sqrtf(norm);
    for (uint8_t i = 0u; i < 4u; i++)
    {
        q[i] /= norm;
    }

    /* 重力转机体 */
    gravity_z = 2.0f * (q[1] * q[1] + q[2] * q[2]) - 1.0f;
    angle = pitch;
    if (gravity_z > 0.0f)
    {
        if (pitch >= 0.0f)
        {
            angle = PITCH_WORLD_PI - pitch;
        }
        else
        {
            angle = -PITCH_WORLD_PI - pitch;
        }
    }
    if (angle >= PITCH_WORLD_PI)
    {
        angle -= PITCH_WORLD_2PI;
    }
    *pitch_world = angle;
    return true;
}
