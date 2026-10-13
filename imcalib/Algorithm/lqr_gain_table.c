/*
 * 文件职责：两机LQR增益表编译选择与求值（lqr_gain_table.c）。
 * 输入/来源：编译选机、machine配置和两腿长度。
 * 输出/去向：按本机实测腿长求40个状态优先排列的K系数。
 * 关键边界：不能把某机K用于另一机；只夹增益求值域，不把实测腿长改成边界值。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "lqr_gain_table.h"
#include <math.h>
#include <string.h>

#if MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
static const lqr_gain_desc_t *const selected_gain = &lqr_gain_small;
#else
static const lqr_gain_desc_t *const selected_gain = &lqr_gain_big;
#endif

const lqr_gain_desc_t *LQR_Gain_Info(void)
{
    return selected_gain;
}

uint8_t LQR_Gain_Eval(float lL, float lR, float K[40])
{
    memset(K, 0, 40u * sizeof(float));
    lL = fminf(fmaxf(lL, selected_gain->len_min), selected_gain->len_max);
    lR = fminf(fmaxf(lR, selected_gain->len_min), selected_gain->len_max);
    selected_gain->eval(lL, lR, K);
    for (uint8_t i = 0u; i < 40u; i++)
    {
        if (!isfinite(K[i]))
        {
            memset(K, 0, 40u * sizeof(float));
            return 0u;
        }
    }
    return 1u;
}

void LQR_K_WBR(float lL, float lR, float K[40])
{
    (void)LQR_Gain_Eval(lL, lR, K);
}
