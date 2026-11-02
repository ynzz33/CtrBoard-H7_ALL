/*
 * 一阶低通和限速斜坡工具：按调用方传入的系数/dt更新保存在结构中的历史值。
 * Ramp_Target_Update可用实测值初始化轨迹，再按rate限制每拍目标变化。
 */

#include "simple-function.h"
#include <math.h>
#include <stddef.h>

/* 初始化 */
void LPF_Init(lpf_t *lp, float alpha)
{
    lp->out = 0.0f;
    lp->alpha = alpha;
}

/* 用保存的上一输出和alpha对本次in做一阶低通，返回并保存滤波结果。 */
float LPF_Update(lpf_t *lp, float in)
{
    lp->out = lp->alpha * in + (1.0f - lp->alpha) * lp->out;
    return lp->out;
}

/* 清零 */
void LPF_Reset(lpf_t *lp)
{
    lp->out = 0.0f;
}

/* 初始化 */
void Ramp_Init(ramp_t *r, float rate)
{
    r->out = 0.0f;
    if (rate < 0.0f)
    {
        rate = -rate;
    }
    r->rate = rate;
}

/* 将r->out向target移动，每拍最大变化由rate和dt决定，返回本拍斜坡值。 */
float Ramp_Update(ramp_t *r, float target, float dt)
{
    float step;

    step = r->rate * dt;
    if ((target - r->out) > step)
    {
        r->out += step;
    }
    else if ((target - r->out) < -step)
    {
        r->out -= step;
    }
    else
    {
        r->out = target;
    }
    return r->out;
}

/* 直接置值 (初始化/复位用, 也用于第一次上电对齐实测值) */
float Ramp_Reset(ramp_t *r, float value)
{
    r->out = value;
    return r->out;
}

/* 必要时从measured初始化参考，再按指定rate和dt向最终target推进。 */
float Ramp_Target_Update(ramp_t *r, float measured, float target, float rate, float dt)
{
    if (!isfinite(target))
    {
        return NAN;
    }
    if (r->rate == 0.0f)
    {
        Ramp_Reset(r, measured);
    }
    r->rate = rate;
    return Ramp_Update(r, target, dt);
}
