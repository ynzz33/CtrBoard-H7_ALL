#include "lowpass.h"

/* 初始化 */
void Lowpass_Init(lowpass1d_t *lp, float alpha)
{
    lp->out = 0.0f;
    lp->alpha = alpha;
}

/* 单周期: y = a*x + (1-a)*y */
float Lowpass_Update(lowpass1d_t *lp, float in)
{
    lp->out = lp->alpha * in + (1.0f - lp->alpha) * lp->out;
    return lp->out;
}

/* 清零 */
void Lowpass_Reset(lowpass1d_t *lp)
{
    lp->out = 0.0f;
}
