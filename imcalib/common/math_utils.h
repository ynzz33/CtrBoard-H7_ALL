/*
 * 文件职责：公共有限性检查等轻量数学辅助（math_utils.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：数值数组及长度。
 * 输出/去向：供RL/控制路径验证NaN/Inf等无效量。
 * 关键边界：这里是无HAL/RTOS依赖的纯辅助接口，不维护策略或设备状态。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>
#include <stdint.h>
#include <stddef.h>

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
