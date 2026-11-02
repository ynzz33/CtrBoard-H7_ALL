/*
 * 通用PID、前馈和历史预置工具；pid_calc的D项使用误差差分，调用方决定系数和dt。
 * 显式物理速度阻尼由各控制模块自行计算，不应直接把两种Kd按相同单位解释。
 */

#include "pid.h"
#include <math.h>

static void pid_param_init(pid_t *pid, uint8_t mode, float maxout,
                           float intergral_limit, float kp, float ki, float kd);

static float ABS(float num)
{
    return (num < 0.0f) ? -num : num;
}

/* 将本拍测量/目标/误差写入PID历史，供新控制段避免误差差分突变。 */
void PID_History_Set(pid_t *pid, float measured, float target, float error)
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        pid->measured[i] = measured;
        pid->target[i] = target;
        pid->err[i] = error;
    }
}

/* 限制输出 */
void abs_limit(float *value, float abs_max)
{
    if (!(*value * 0.0f == 0.0f))
    {
        *value = 0.0f;
        return;
    }
    if (*value > abs_max) *value = abs_max;
    if (*value < -abs_max) *value = -abs_max;
}

/* 软化死区 */
float Deadband_Soften(float err, float deadband)
{
    if (deadband <= 0.0f) return err;
    if (err > deadband) return err - deadband;
    if (err < -deadband) return err + deadband;
    return 0.0f;
}

/* 按mode计算位置式/增量式PID，保存误差历史；D项取离散误差差，不是直接测量速度阻尼。 */
float pid_calc(pid_t *pid, float measured, float target, float delta_time)
{
    pid->measured[NOW] = measured;
    pid->target[NOW] = target;
    pid->err[NOW] = target - measured;
    if (pid->angle_wrap) pid->err[NOW] = Angle_Wrap_Pi(pid->err[NOW]);
    if (pid->max_err != 0.0f && ABS(pid->err[NOW]) > pid->max_err)
        return 0.0f;

    float e_now = Deadband_Soften(pid->err[NOW], pid->deadband);
    float e_last = Deadband_Soften(pid->err[LAST], pid->deadband);

    if (pid->pid_mode == POSITION_PID)
    {
        pid->pout = pid->p * e_now;
        pid->iout += pid->i * e_now * delta_time;
        if (pid->d == 0.0f)
        {
            pid->dout = 0.0f;
        }
        else
        {
            pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
        }

        abs_limit(&pid->iout, pid->IntegralLimit);
        pid->position_mode_output = pid->pout + pid->iout + pid->dout;
        abs_limit(&pid->position_mode_output, pid->MaxOutput);
        pid->last_position_mode_output = pid->position_mode_output;
    }
    else if (pid->pid_mode == DELTA_PID)
    {
        float e_llast = Deadband_Soften(pid->err[LLAST], pid->deadband);
        pid->pout = pid->p * (e_now - e_last);
        pid->iout = pid->i * e_now * delta_time;
        pid->dout = pid->d * (e_now - 2.0f * e_last + e_llast);
        abs_limit(&pid->iout, pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&pid->delta_out, pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->measured[LLAST] = pid->measured[LAST];
    pid->measured[LAST] = pid->measured[NOW];
    pid->target[LLAST] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];
    return (pid->pid_mode == POSITION_PID) ? pid->position_mode_output : pid->delta_out;
}

/* 计算前馈 */
float FeedForwardController(FFC_t *ffc, float target, float num1, float num2)
{
    ffc->rin = target;
    float result = num1 * (ffc->rin - ffc->lastRin) + num2 * (ffc->rin - 2.0f * ffc->lastRin + ffc->perRin);
    float old_last = ffc->lastRin;
    ffc->lastRin = ffc->rin;
    ffc->perRin = old_last;
    ffc->FFC_pos_out = result;
    return result;
}

/* 初始化参数 */
void PID_struct_init(pid_t *pid, uint8_t mode, float maxout,
                     float intergral_limit, float kp, float ki, float kd,
                     float ff_param1, float ff_param2)
{
    pid->f_param_init = pid_param_init;
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
    FeedForwardParamInit(&pid->xFeedForward, ff_param1, ff_param2);
}

/* 初始化前馈 */
void FeedForwardParamInit(FFC_t *ffc, float param1, float param2)
{
    ffc->num1 = param1;
    ffc->num2 = param2;
}

/* 保存参数 */
static void pid_param_init(pid_t *pid, uint8_t mode, float maxout,
                           float intergral_limit, float kp, float ki, float kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
