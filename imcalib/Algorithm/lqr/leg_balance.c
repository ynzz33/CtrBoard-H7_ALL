/*
 * 文件职责：NORMAL固定四态LQR、非NORMAL原PD及腿部输出组合（leg_balance.c）。
 * 输入/来源：LQR状态/最终腿长目标、双腿实测几何、显式运行配置和dt。
 * 输出/去向：生成左右轴向力/摆矩，经公共补偿与限幅后填torque_output_t。
 * 关键边界：AIR释放时轴向主动出力为零；非释放时用原轨迹斜坡；只求解，不发送CAN。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "leg_balance.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

/* PD实例仅供非NORMAL调用；正常模式不使用。 */
void Leg_Balance_Init(leg_balance_t *lb)
{
    memset(lb, 0, sizeof(*lb));

    for (uint8_t i = 0u; i < 2u; i++)
    {
        PID_struct_init(&lb->leg_len[i], POSITION_PID, machine->lqr.leg_len[i].max_output, 0.0f, machine->lqr.leg_len[i].kp, 0.0f, machine->lqr.leg_len[i].kd * MACHINE_LQR_DT, 0.0f, 0.0f);
    }
    PID_struct_init(&lb->roll, POSITION_PID, machine->lqr.roll.max_output, 0.0f, machine->lqr.roll.kp, 0.0f, machine->lqr.roll.kd * MACHINE_LQR_DT, 0.0f, 0.0f);
    Roll_Height_Reset(&lb->roll_height);
}

/* 清控制器历史与调试观测值 */
/*
 * 清非NORMAL PD历史、四态观测/输出、力/摆矩、补偿和轨迹斜坡，保留配置。
 * 进入/退出轴向释放都会调用；退出时下一拍从实测重新播种给定。
 */
void Leg_Balance_Reset(leg_balance_t *lb)
{
    pid_t *pid[3];

    pid[0] = &lb->leg_len[0];
    pid[1] = &lb->leg_len[1];
    pid[2] = &lb->roll;
    for (uint8_t i = 0u; i < 3u; i++)
    {
        memset(pid[i]->err, 0, sizeof(pid[i]->err));
        memset(pid[i]->target, 0, sizeof(pid[i]->target));
        memset(pid[i]->measured, 0, sizeof(pid[i]->measured));
        pid[i]->pout = 0.0f;
        pid[i]->iout = 0.0f;
        pid[i]->dout = 0.0f;
        pid[i]->position_mode_output = 0.0f;
        pid[i]->last_position_mode_output = 0.0f;
        pid[i]->vel_err = 0.0f;
    }
    memset(lb->F, 0, sizeof(lb->F));
    memset(lb->Tp, 0, sizeof(lb->Tp));
    memset(&lb->cmd, 0, sizeof(lb->cmd));
    memset(&lb->compensation, 0, sizeof(lb->compensation));
    memset(lb->length_ramp, 0, sizeof(lb->length_ramp));
    memset(lb->length_ref_rate, 0, sizeof(lb->length_ref_rate));
    lb->length_ref_ready = 0u;
    lb->turn_ff = 0.0f;
    lb->air_release = 0u;
    Roll_Height_Reset(&lb->roll_height);
}

/* P响应位置误差，D直接使用实际速度；不积分、不差分给定或误差。 */
static void Leg_PD_Update(pid_t *pd, float measured, float target, float velocity)
{
    pd->measured[NOW] = measured;
    pd->target[NOW] = target;
    pd->err[NOW] = target - measured;
    pd->vel_err = -velocity;
    pd->pout = pd->p * pd->err[NOW];
    pd->dout = pd->d * pd->vel_err;
    pd->position_mode_output = pd->pout + pd->dout;
    abs_limit(&pd->position_mode_output, pd->MaxOutput);
    pd->last_position_mode_output = pd->position_mode_output;
}

/* 力域映射 + 限幅: (足端力, 髋扭矩) → 前后髋电机力矩 (虚功原理), 轮扭矩直接限幅 */
/*
 * 先把本腿F/Tp用雅可比映射成两髋基础力矩，再统一组合弹簧/重力并限幅。
 * 补偿前F/Tp记录在lb，输出电机力矩已包含补偿；两种量不能直接混作同一受力值。
 * 轮输出这里只夹物理力矩，发送电流与输出极性在dji驱动统一处理。
 */
static uint8_t Leg_Balance_Output(leg_balance_t *lb, const leg_state_t *leg_l,
                                  const leg_state_t *leg_r, const float F[2],
                                  const float Tp[2], const float wheel[2],
                                  const leg_compensation_values_t *values,
                                  const lqr_runtime_config_t *config,
                                  torque_output_t *torque)
{
    float base_dm[4];
    memset(&lb->cmd, 0, sizeof(lb->cmd));   /* 失败时与零力矩一致 */
    memset(&lb->compensation, 0, sizeof(lb->compensation));
    if (!Leg_Force_Map_Forward(leg_l, F[0], Tp[0], &base_dm[0]) || !Leg_Force_Map_Forward(leg_r, F[1], Tp[1], &base_dm[2]))
    {
        return 0u;
    }
    if (!Leg_Compensation_Apply_Mode(&values->spring, &values->gravity, base_dm, config->trq_max_hip, (uint8_t)!lb->air_release, torque->dm, &lb->compensation))
    {
        return 0u;
    }
    lb->F[0] = F[0];
    lb->F[1] = F[1];
    lb->Tp[0] = Tp[0];
    lb->Tp[1] = Tp[1];

    /* 轮扭矩 (输出极性在 dji.c 驱动边界统一处理) */
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(wheel[0], -config->trq_max_wheel, config->trq_max_wheel);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(wheel[1], -config->trq_max_wheel, config->trq_max_wheel);
    lb->cmd = *torque;
    lb->cmd.valid = 1u;
    return 1u;
}

/* NORMAL仅四状态反馈，非NORMAL保留原PD；统一映射为电机力矩。 */
/*
 * NORMAL：加速度受限的长度位置/速度参考→四态反馈；非NORMAL沿用原斜坡/PD。
 * 释放：轴向F=0且不推进长度/roll PD与斜坡；摆角LQR和完整重力仍保留。
 * 轴向/摆矩映射和补偿在输出辅助函数中完成，本函数返回成功与否，发送由执行任务负责。
 */
/* NORMAL参考：速度/加速度有界，离散制动距离避免到位时强行切断速度。
 * 目标中途反向时先减速；不拿实测噪声重置每拍参考，也不改公共Ramp函数。
 */
static float Normal_Length_Target_Update(ramp_t *ramp, float *velocity, float target, float speed_max, float accel_max, float dt)
{
    float error;
    float dv;
    float brake_speed;
    float wanted;
    float next_velocity;
    if (!isfinite(target))
    {
        return NAN;
    }
    error = target - ramp->out;
    dv = accel_max * dt;
    brake_speed = sqrtf(2.0f * accel_max * fabsf(error) + 0.25f * dv * dv) - 0.5f * dv;
    wanted = copysignf(fminf(speed_max, brake_speed), error);
    next_velocity = clampf(wanted, *velocity - dv, *velocity + dv);
    if (fabsf(error) <= dv * dt && fabsf(error / dt - *velocity) <= dv)
    {
        next_velocity = error / dt;
        ramp->out = target;
    }
    else
    {
        ramp->out += next_velocity * dt;
    }
    *velocity = next_velocity;
    ramp->rate = speed_max;
    return ramp->out;
}

uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            const leg_compensation_values_t *values,
                            const lqr_runtime_config_t *config,
                            uint8_t normal_mode,
                            float dt, torque_output_t *torque)
{
    float Tp[2];
    float F[2];
    float wheel[2];
    float length[2];
    float target[2];
    float height;

    lb->turn_ff = 0.0f;
    length[0] = leg_l->output.virtual_leg_length;
    length[1] = leg_r->output.virtual_leg_length;
    target[0] = st->leg_length_target[0];
    target[1] = st->leg_length_target[1];

    if (lb->air_release != st->air_release)
    {
        Leg_Balance_Reset(lb);
        lb->air_release = st->air_release;
    }
    F[0] = 0.0f;
    F[1] = 0.0f;
    if (!st->air_release)
    {
        if (normal_mode)
        {
            if (MACHINE_NORMAL_LEG_CONTROL == 1u && !Roll_Height_Available())
            {
                return 0u;
            }
            if (!lb->length_ref_ready)
            {
                for (uint8_t i = 0u; i < 2u; i++)
                {
                    Ramp_Reset(&lb->length_ramp[i], length[i]);
                    lb->length_ref_rate[i] = 0.0f;
                    target[i] = length[i];
                }
                lb->length_ref_ready = 1u;
            }
            else
            {
                for (uint8_t i = 0u; i < 2u; i++)
                {
                    target[i] = Normal_Length_Target_Update(&lb->length_ramp[i], &lb->length_ref_rate[i], target[i], machine->rc.len_rate, machine->lqr.length_ref_accel_max, dt);
                }
            }
        }
        else
        {
            lb->length_ref_ready = 0u;
            lb->length_ref_rate[0] = 0.0f;
            lb->length_ref_rate[1] = 0.0f;
            target[0] = Ramp_Target_Update(&lb->length_ramp[0], length[0], target[0], machine->rc.len_rate, dt);
            target[1] = Ramp_Target_Update(&lb->length_ramp[1], length[1], target[1], machine->rc.len_rate, dt);
        }
        if (!isfinite(target[0]) || !isfinite(target[1]) || !isfinite(lb->length_ref_rate[0]) || !isfinite(lb->length_ref_rate[1]))
        {
            return 0u;
        }

        if (normal_mode)
        {
            height = 0.5f * (length[0] + length[1]) + machine->lqr.turn_comp.height_offset;
            lb->turn_ff = machine->lqr.turn_comp.gain * machine->lqr.turn_comp.mass * height * st->ds_kf * st->x[LQR_X_DPHI] / machine->lqr.turn_comp.track;
        }
        if (normal_mode && MACHINE_NORMAL_LEG_CONTROL == 1u)
        {
            lb->roll_height.permit = 1u;
            if (!Roll_Height_Update(&lb->roll_height, st, leg_l, leg_r, target, lb->length_ref_rate, lb->turn_ff))
            {
                return 0u;
            }
            F[0] = lb->roll_height.force[0];
            F[1] = lb->roll_height.force[1];
        }
        else
        {
            Roll_Height_Reset(&lb->roll_height);
            Leg_PD_Update(&lb->leg_len[0], length[0], target[0], leg_l->output.d_virtual_leg_length);
            Leg_PD_Update(&lb->leg_len[1], length[1], target[1], leg_r->output.d_virtual_leg_length);
            Leg_PD_Update(&lb->roll, st->roll, 0.0f, st->roll_rate);
            F[0] = lb->leg_len[0].position_mode_output + lb->roll.position_mode_output + machine->lqr.support_force[0] - lb->turn_ff;
            F[1] = lb->leg_len[1].position_mode_output - lb->roll.position_mode_output + machine->lqr.support_force[1] + lb->turn_ff;
        }
    }
    else
    {
        Roll_Height_Reset(&lb->roll_height);
    }
    Tp[0] = -st->u[LQR_U_BL];
    Tp[1] = -st->u[LQR_U_BR];
    if (!config->leg_swing_enable)
    {
        Tp[0] = 0.0f;
        Tp[1] = 0.0f;
    }
    wheel[0] = config->wheel_enable ? st->u[LQR_U_WL] : 0.0f;
    wheel[1] = config->wheel_enable ? st->u[LQR_U_WR] : 0.0f;

    /* 3. 力域映射 + 限幅 */
    return Leg_Balance_Output(lb, leg_l, leg_r, F, Tp, wheel, values, config, torque);
}
