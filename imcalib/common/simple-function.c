/*
 * 文件职责：一阶低通与原目标轨迹斜坡（simple-function.c）。
 * 输入/来源：滤波输入或实测/最终目标、斜率和dt。
 * 输出/去向：返回滤波量或逐拍受速率限制的给定值。
 * 关键边界：Ramp_Target_Update仅第一次从实测起步，之后沿用上拍输出；控制刚度不能用每拍重置起点替代。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "simple-function.h"
#include <math.h>
#include <stddef.h>

/* ================= 一阶低通 ================= */

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

/* ================= 斜坡 (斜率限制) ================= */

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

/* 单周期: 每个 dt 最多走 rate*dt, 到目标即等于目标 */
/*
 * 只限制给定轨迹每拍的增量，不限制机器人实际运动速度或PID输出力。
 * 输出存在r->out，目标在一步内时直接到目标；rate和dt由调用方提供。
 */
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

/* 实测值起步 */
/*
 * 原轨迹斜坡：首次使用时把输出置为measured，随后从内部保存的上拍输出向target移动。
 * 每拍最多变化rate*dt；measured不是每拍重置命令，因而PID能保持实际长度偏差和阻尼。
 * 调用方保证实测值与周期有效，速率直接取机器/自起参数；目标无效返回NAN，交接清历史。
 */
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
