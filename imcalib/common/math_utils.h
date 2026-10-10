#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>
#include <stdint.h>
#include <stddef.h>

/* 有限数数组 */
static inline uint8_t Float_Array_Finite(const float *data, uint32_t count)
{
    uint32_t i;

    if (data == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < count; i++)
    {
        if (!isfinite(data[i]))
        {
            return 0u;
        }
    }
    return 1u;
}

#endif
