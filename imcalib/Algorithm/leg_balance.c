#include "leg_balance.h"

#include <math.h>
#include <string.h>

/* 初始化 — PID 初始化不覆盖 max_err/deadband/angle_wrap, 必须先清零 */
void Leg_Balance_Init(leg_balance_t *lb)
{
    memset(lb, 0, sizeof(*lb));

    PID_struct_init(&lb->leg_len[0], POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_LEN_KP, 0.0f, LEG_BALANCE_LEN_KD, 0.0f, 0.0f);
    PID_struct_init(&lb->leg_len[1], POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_LEN_KP, 0.0f, LEG_BALANCE_LEN_KD, 0.0f, 0.0f);
    PID_struct_init(&lb->leg_sym, POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_SYM_KP, 0.0f, LEG_BALANCE_SYM_KD, 0.0f, 0.0f);
    PID_struct_init(&lb->roll, POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_ROLL_KP, 0.0f, LEG_BALANCE_ROLL_KD, 0.0f, 0.0f);
}

/* 腿长/防劈叉/横滚 PID + 力向量 + 雅可比映射 → 电机力矩 */
uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            float dt, torque_output_t *torque)
{
    float Tp[2];
    float F[2];
    float tau[2];
    uint8_t i;

    if (lb == NULL || st == NULL || leg_l == NULL || leg_r == NULL
        || torque == NULL)
    {
        return 0u;
    }
    if (!leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    /* 1. 辅助 PID */
    (void)pid_calc(&lb->leg_len[0], leg_l->output.virtual_leg_length,
                   st->leg_len_tgt[0], dt);
    (void)pid_calc(&lb->leg_len[1], leg_r->output.virtual_leg_length,
                   st->leg_len_tgt[1], dt);
    (void)pid_calc(&lb->leg_sym, st->x[LQR_X_THL] - st->x[LQR_X_THR], 0.0f, dt);
    (void)pid_calc(&lb->roll, st->roll, 0.0f, dt);

    /* 2. 力向量: 轮扭矩/虚拟髋扭矩来自 LQR, 足端力来自腿长+横滚+前馈
     * Tp 取反: 本工程腿摆角与模型 θ_ll 反号, 广义力随之反号
     * (Leg_Force_Map_Forward(F,Tp) ≡ Leg_Tougue(F,-Tp)) */
    Tp[0] = -(st->u[LQR_U_BL] + lb->leg_sym.pos_out);
    Tp[1] = -(st->u[LQR_U_BR] - lb->leg_sym.pos_out);
    F[0] = lb->leg_len[0].pos_out + lb->roll.pos_out + LEG_BALANCE_F_FEEDFORWARD;
    F[1] = lb->leg_len[1].pos_out - lb->roll.pos_out + LEG_BALANCE_F_FEEDFORWARD;

    for (i = 0u; i < 2u; i++)
    {
        if (!isfinite(F[i]) || !isfinite(Tp[i]))
        {
            return 0u;
        }
    }
    lb->F[0] = F[0];
    lb->F[1] = F[1];
    lb->Tp[0] = Tp[0];
    lb->Tp[1] = Tp[1];

    /* 3. 力域映射: (足端力, 髋扭矩) → 前后髋电机力矩 (虚功原理) */
    (void)Leg_Force_Map_Forward(leg_l, F[0], Tp[0], tau);
    torque->dm[DM_MOTOR_LEG_F_LFT] = clampf(tau[0], -LQR_HIP_TRQ_MAX, LQR_HIP_TRQ_MAX);
    torque->dm[DM_MOTOR_LEG_B_LFT] = clampf(tau[1], -LQR_HIP_TRQ_MAX, LQR_HIP_TRQ_MAX);

    (void)Leg_Force_Map_Forward(leg_r, F[1], Tp[1], tau);
    torque->dm[DM_MOTOR_LEG_F_RGT] = clampf(tau[0], -LQR_HIP_TRQ_MAX, LQR_HIP_TRQ_MAX);
    torque->dm[DM_MOTOR_LEG_B_RGT] = clampf(tau[1], -LQR_HIP_TRQ_MAX, LQR_HIP_TRQ_MAX);

    /* 4. 轮扭矩: 右轮取反 (dji.c 不做符号, 与 rl_torque 一致) */
    torque->dji[DJI_MOTOR_WHEEL_LFT] =
        clampf(st->u[LQR_U_WL], -LQR_WHEEL_TRQ_MAX, LQR_WHEEL_TRQ_MAX);
    torque->dji[DJI_MOTOR_WHEEL_RGT] =
        clampf(-st->u[LQR_U_WR], -LQR_WHEEL_TRQ_MAX, LQR_WHEEL_TRQ_MAX);

    return 1u;
}
