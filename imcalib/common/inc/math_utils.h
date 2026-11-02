/*
 * 提供对称/区间限幅、弧度包角及浮点数组有限性检查。
 * 这些函数不保存历史；NaN检查由调用方或Float_Array_Finite完成。
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>
#include <stdint.h>
#include <stddef.h>

/* 区间限幅，保留先上界后下界的原比较顺序；NaN原样返回，由结果生成端检查。 */
static inline float clampf(float value, float min, float max)
{
    if (value > max) { return max; }
    if (value < min) { return min; }
    return value;
}

/* 对称限幅[-limit,limit]；沿用原短实现，低优化等级下也不增加嵌套函数调用。 */
static inline float clip(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

/* 弧度角逐圈环绕到[-π,π]，沿用原PID算法及float常量；不改为fmod/remainder。 */
static inline float Angle_Wrap_Pi(float angle)
{
    while (angle > 3.14159265f) { angle -= 6.28318530f; }
    while (angle < -3.14159265f) { angle += 6.28318530f; }
    return angle;
}

/* 有限数数组 */
static inline uint8_t Float_Array_Finite(const float *data, uint32_t count)
{
    for (uint32_t i = 0u; i < count; i++)
    {
        if (!isfinite(data[i]))
        {
            return 0u;
        }
    }
    return 1u;
}

#endif
