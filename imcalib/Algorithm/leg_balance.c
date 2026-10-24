/*
 * 文件职责：LQR腿长/横滚显式PD及腿部输出组合（leg_balance.c）。
 * 输入/来源：LQR状态/最终腿长目标、双腿实测几何、显式运行配置和dt。
 * 输出/去向：生成左右轴向力/摆矩，经公共补偿与限幅后填torque_output_t。
 * 关键边界：AIR释放时轴向主动出力为零；非释放时用原轨迹斜坡；只求解，不发送CAN。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "leg_balance.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

/* 复用PID容器保存P/D分项；正常PD的D增益由原差分增益换算为速度阻尼。 */
void Leg_Balance_Init(leg_balance_t *lb)
{
    memset(lb, 0, sizeof(*lb));

    for (uint8_t i = 0u; i < 2u; i++)
    {
        PID_struct_init(&lb->leg_len[i], POSITION_PID, machine->lqr.leg_len[i].max_output, 0.0f, machine->lqr.leg_len[i].kp, 0.0f, machine->lqr.leg_len[i].kd * MACHINE_LQR_DT, 0.0f, 0.0f);
    }
    PID_struct_init(&lb->roll, POSITION_PID, machine->lqr.roll.max_output, 0.0f, machine->lqr.roll.kp, 0.0f, machine->lqr.roll.kd * MACHINE_LQR_DT, 0.0f, 0.0f);
}

/* 清控制器历史与调试观测值 */
/*
 * 清PD观测/输出、力/摆矩、补偿和轨迹斜坡，保留已配置增益。
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
        memset(pid[i]->set, 0, sizeof(pid[i]->set));
        memset(pid[i]->get, 0, sizeof(pid[i]->get));
        pid[i]->pout = 0.0f;
        pid[i]->iout = 0.0f;
        pid[i]->dout = 0.0f;
        pid[i]->pos_out = 0.0f;
        pid[i]->last_pos_out = 0.0f;
        pid[i]->vel_err = 0.0f;
    }
    memset(lb->F, 0, sizeof(lb->F));
    memset(lb->Tp, 0, sizeof(lb->Tp));
    memset(&lb->cmd, 0, sizeof(lb->cmd));
    memset(&lb->compensation, 0, sizeof(lb->compensation));
    memset(lb->length_ramp, 0, sizeof(lb->length_ramp));
    lb->turn_ff = 0.0f;
    lb->air_release = 0u;
}

/* P响应位置误差，D直接使用实际速度；不积分、不差分给定或误差。 */
static void Leg_PD_Update(pid_t *pd, float measured, float target, float velocity)
{
    pd->get[NOW] = measured;
    pd->set[NOW] = target;
    pd->err[NOW] = target - measured;
    pd->vel_err = -velocity;
    pd->pout = pd->p * pd->err[NOW];
    pd->dout = pd->d * pd->vel_err;
    pd->pos_out = pd->pout + pd->dout;
    abs_limit(&pd->pos_out, pd->MaxOutput);
    pd->last_pos_out = pd->pos_out;
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

/* 腿长/横滚显式PD + 力向量 + 雅可比映射 → 电机力矩 */
/*
 * 非释放：最终目标 → 原轨迹斜坡 → 位置误差P与实际速度D → 支撑前馈。
 * 释放：轴向F=0且不推进长度/roll PD与斜坡；摆角LQR和完整重力仍保留。
 * 轴向/摆矩映射和补偿在输出辅助函数中完成，本函数返回成功与否，发送由执行任务负责。
 */
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
    target[0] = st->leg_len_tgt[0];
    target[1] = st->leg_len_tgt[1];

    if (lb->air_release != st->air_release)
    {
        Leg_Balance_Reset(lb);
        lb->air_release = st->air_release;
    }
    F[0] = 0.0f;
    F[1] = 0.0f;
    if (!st->air_release)
    {
        target[0] = Ramp_Target_Update(&lb->length_ramp[0], length[0], target[0], machine->rc.len_rate, dt);
        target[1] = Ramp_Target_Update(&lb->length_ramp[1], length[1], target[1], machine->rc.len_rate, dt);
        if (!isfinite(target[0]) || !isfinite(target[1]))
        {
            return 0u;
        }

        Leg_PD_Update(&lb->leg_len[0], length[0], target[0], leg_l->output.d_virtual_leg_length);
        Leg_PD_Update(&lb->leg_len[1], length[1], target[1], leg_r->output.d_virtual_leg_length);
        Leg_PD_Update(&lb->roll, st->roll, 0.0f, st->roll_rate);

        if (normal_mode)
        {
            /* 机身近似向心前馈：实际v与yaw角速，直接参与腿力合成。 */
            height = 0.5f * (length[0] + length[1]) + machine->lqr.turn_comp.height_offset;
            lb->turn_ff = machine->lqr.turn_comp.gain * machine->lqr.turn_comp.mass * height * st->ds_kf * st->x[LQR_X_DPHI] / machine->lqr.turn_comp.track;
        }
        /* 模型力矩转力域 */
        F[0] = lb->leg_len[0].pos_out + lb->roll.pos_out + machine->lqr.support_force[0] - lb->turn_ff;
        F[1] = lb->leg_len[1].pos_out - lb->roll.pos_out + machine->lqr.support_force[1] + lb->turn_ff;
    }
    Tp[0] = -st->u[LQR_U_BL];
    Tp[1] = -st->u[LQR_U_BR];
    if (!config->hip_enable)
    {
        Tp[0] = 0.0f;
        Tp[1] = 0.0f;
    }
    if (!st->air_release && !config->len_pid_enable)
    {
        lb->turn_ff = 0.0f;
        F[0] = machine->lqr.support_force[0];
        F[1] = machine->lqr.support_force[1];
    }
    wheel[0] = config->wheel_enable ? st->u[LQR_U_WL] : 0.0f;
    wheel[1] = config->wheel_enable ? st->u[LQR_U_WR] : 0.0f;

    /* 3. 力域映射 + 限幅 */
    return Leg_Balance_Output(lb, leg_l, leg_r, F, Tp, wheel, values, config, torque);
}
