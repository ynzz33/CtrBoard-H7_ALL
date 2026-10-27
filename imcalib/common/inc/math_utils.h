/*
 * 文件职责：公共限幅、角度环绕和有限性检查（math_utils.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：浮点值/边界或数值数组及长度。
 * 输出/去向：供控制/观测/几何共用纯数学操作及NaN/Inf检查。
 * 关键边界：这里是无HAL/RTOS依赖的纯辅助接口，不维护策略或设备状态。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
