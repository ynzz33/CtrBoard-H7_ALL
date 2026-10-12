/*
 * 文件职责：带加速度预测的一维速度卡尔曼（kalman.c）。
 * 输入/来源：加速度、速度观测、dt及标量协方差/噪声参数。
 * 输出/去向：预测速度并执行观测修正，返回估计值。
 * 关键边界：这是保留的通用标量接口；当前执行层接触门控速度融合使用slip.c的两状态实现。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "kalman.h"

/* ================= 带加速度输入的一维卡尔曼 ================= */

/* 初始化 */
void Kalman_Accel_Init(kalman_accel_t *kf, float v0, float p0, float q,
                       float r, float p_max)
{
    kf->vel   = v0;
    kf->p     = p0;
    kf->q     = q;
    kf->r     = r;
    kf->p_max = p_max;
}

/* 单周期: 加速度预测 → 协方差钳位 → 观测修正 */
float Kalman_Accel_Update(kalman_accel_t *kf, float accel, float z_vel, float dt)
{
    float k;

    kf->vel += accel * dt;
    kf->p   += kf->q;
    if (kf->p > kf->p_max)
    {
        kf->p = kf->p_max;      /* 防静止后 p 无限涨 */
    }
    k = kf->p / (kf->p + kf->r);
    kf->vel += k * (z_vel - kf->vel);
    kf->p    = (1.0f - k) * kf->p;
    return kf->vel;
}
