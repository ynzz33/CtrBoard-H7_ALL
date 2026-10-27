/*
 * 文件职责：一阶低通与原目标轨迹斜坡（simple-function.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：滤波输入或实测/最终目标、斜率和dt。
 * 输出/去向：返回滤波量或逐拍受速率限制的给定值。
 * 关键边界：Ramp_Target_Update仅第一次从实测起步，之后沿用上拍输出；控制刚度不能用每拍重置起点替代。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
