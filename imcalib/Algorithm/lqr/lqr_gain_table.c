/*
 * 按编译机型选择增益表，将左右腿长夹到表域后求40个K系数并检查有限性。
 * 夹紧仅作用于查表输入，不改写实际反馈腿长；系数排列由增益接口定义。
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
