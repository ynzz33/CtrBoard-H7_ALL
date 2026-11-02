/*
 * 一阶低通和限速斜坡工具：按调用方传入的系数/dt更新保存在结构中的历史值。
 * Ramp_Target_Update可用实测值初始化轨迹，再按rate限制每拍目标变化。
 */

#ifndef SIMPLE_FUNCTION_H
#define SIMPLE_FUNCTION_H

/* ================= 简单函数库 (一阶低通 / 斜坡等) ================= */

/* 一阶低通 */
typedef struct {
    float out;
    float alpha;
} lpf_t;

void  LPF_Init(lpf_t *lp, float alpha);
float LPF_Update(lpf_t *lp, float in);
void  LPF_Reset(lpf_t *lp);

/* 斜坡 (斜率限制): 按最大斜率线性逼近目标, 到位后直接跟随 */
typedef struct {
    float out;      /* 当前输出 */
    float rate;     /* 最大斜率, 单位/秒 */
} ramp_t;

void  Ramp_Init(ramp_t *r, float rate);
float Ramp_Update(ramp_t *r, float target, float dt);
float Ramp_Reset(ramp_t *r, float value);
float Ramp_Target_Update(ramp_t *r, float measured, float target, float rate, float dt);


#endif
