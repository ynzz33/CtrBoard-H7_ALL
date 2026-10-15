/*
 * 文件职责：位置式/增量式PID、误差历史与前馈（pid.c）。
 * 输入/来源：实测值、给定值、dt、增益/限幅/死区和角度环绕配置。
 * 输出/去向：更新P/I/D、误差历史和输出，提供首拍历史预置。
 * 关键边界：位置式D沿用误差逐拍差分，未除dt；改给定会影响D，首拍预置只消除初始化突变。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "pid.h"
#include <math.h>

static void pid_param_init(pid_t *pid, uint8_t mode, float maxout,
                           float intergral_limit, float kp, float ki, float kd);

static float ABS(float num)
{
    return (num < 0.0f) ? -num : num;
}

/* 预置历史 */
/*
 * 将测量/给定/误差三拍历史同步预置，避免投入或模式切换的首拍D突变。
 * 只改历史，不改变增益、积分或输出限幅；运行中不能每拍预置，否则会消除真实阻尼。
 */
void PID_History_Set(pid_t *pid, float measured, float target, float error)
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        pid->get[i] = measured;
        pid->set[i] = target;
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

/* 计算PID */
/*
 * 位置式误差=set-get；P响应当前误差，I按dt积分，D沿用误差的逐拍差分。
 * D没有除dt，其增益对应现有离散节拍；给定突变和实测突变都能进入D。
 * 计算后移位get/set/err历史，并分别限积分和总输出；角度环绕/死区按pid配置选择。
 */
float pid_calc(pid_t *pid, float get, float set, float delta_time)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;
    if (pid->angle_wrap) pid->err[NOW] = Angle_Wrap_180(pid->err[NOW]);
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
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&pid->pos_out, pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;
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
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return (pid->pid_mode == POSITION_PID) ? pid->pos_out : pid->delta_out;
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
