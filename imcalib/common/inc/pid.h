/*
 * 通用PID和历史预置工具；pid_calc的D项使用误差差分，调用方决定系数和dt。
 * 显式物理速度阻尼由各控制模块自行计算，不应直接把两种Kd按相同单位解释。
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

typedef struct __pid_t {
    float p;
    float i;
    float d;
    uint8_t pid_mode;
    float MaxOutput;
    float IntegralLimit;
    float err[3];
    float pout;
    float iout;
    float dout;
    float position_mode_output;
    float delta_u;
    float delta_out;
    float last_delta_out;
    float max_err;
    float deadband;
    float vel_err;
    uint8_t angle_wrap;
} pid_t;

void PID_struct_init(pid_t *pid, uint8_t mode, float maxout,
                     float intergral_limit, float kp, float ki, float kd);
float pid_calc(pid_t *pid, float measured, float target, float delta_time);
void PID_History_Set(pid_t *pid, float error);
float Deadband_Soften(float err, float deadband);
void abs_limit(float *value, float abs_max);

#endif
