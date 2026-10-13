/*
 * 文件职责：带加速度预测的一维速度卡尔曼（kalman.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：加速度、速度观测、dt及标量协方差/噪声参数。
 * 输出/去向：预测速度并执行观测修正，返回估计值。
 * 关键边界：这是保留的通用标量接口；当前执行层接触门控速度融合使用slip.c的两状态实现。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef KALMAN_H
#define KALMAN_H

/* ================= 带加速度输入的一维卡尔曼速度估计 (同 Leg2_v1) =================
 * 过程: v = v + a·dt ; 观测: z = v ; 详见 md/file-guide.md 的数学工具说明 */

typedef struct {
    float vel;      /* 速度估计 */
    float p;        /* 误差协方差 */
    float q;        /* 过程噪声 */
    float r;        /* 观测噪声 */
    float p_max;    /* 协方差上限 */
} kalman_accel_t;

void  Kalman_Accel_Init(kalman_accel_t *kf, float v0, float p0, float q,
                        float r, float p_max);
float Kalman_Accel_Update(kalman_accel_t *kf, float accel, float z_vel, float dt);

#endif
