/*
 * 文件职责：位置式/增量式PID、误差历史与前馈（pid.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：实测值、给定值、dt、增益/限幅/死区和角度环绕配置。
 * 输出/去向：更新P/I/D、误差历史和输出，提供首拍历史预置。
 * 关键边界：位置式D沿用误差逐拍差分，未除dt；改给定会影响D，首拍预置只消除初始化突变。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "math_utils.h"

enum {
    POSITION_PID,
    DELTA_PID,
};

enum {
    NOW,
    LAST,
    LLAST,
};

typedef struct __FFC_t {
    float FFCTar;
    float rin;
    float lastRin;
    float perRin;
    float FFC_pos_out;
    float set_point;
    float num1;
    float num2;
} FFC_t;

typedef struct __pid_t {
    float p;
    float i;
    float d;
    uint8_t pid_mode;
    float MaxOutput;
    float IntegralLimit;
    FFC_t xFeedForward;
    float target[3];
    float measured[3];
    float err[3];
    float pout;
    float iout;
    float dout;
    float position_mode_output;
    float last_position_mode_output;
    float delta_u;
    float delta_out;
    float last_delta_out;
    float max_err;
    float deadband;
    float vel_err;
    uint8_t angle_wrap;
    void (*f_param_init)(struct __pid_t *pid, uint8_t pid_mode,
                         float maxOutput, float integralLimit,
                         float p, float i, float d);
} pid_t;

void PID_struct_init(pid_t *pid, uint8_t mode, float maxout,
                     float intergral_limit, float kp, float ki, float kd,
                     float ff_param1, float ff_param2);
void FeedForwardParamInit(FFC_t *ffc, float param1, float param2);
float pid_calc(pid_t *pid, float measured, float target, float delta_time);
void PID_History_Set(pid_t *pid, float measured, float target, float error);
float Deadband_Soften(float err, float deadband);
float FeedForwardController(FFC_t *ffc, float target, float num1, float num2);
void abs_limit(float *value, float abs_max);

#endif
