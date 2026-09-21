#include "lqr_balance.h"
#include "lqr_gain_table.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

/* 一阶低通系数 */
#define LQR_LPF_ALPHA       0.3f
/* 速度卡尔曼: 同 Leg2_v1 Body.h (P0 / Q / R / P 上限) */
#define LQR_KF_P0           0.1f
#define LQR_KF_Q            0.005f
#define LQR_KF_R            0.01f
#define LQR_KF_P_MAX        0.5f
#define LQR_GRAVITY         9.81f
/* 腿长变化超此阈值才重算增益 (m) */
#define LQR_K_RECALC_THRESH 0.0005f

/* 站立目标 */
#define LQR_POS_TARGET      (-0.0f)
#define LQR_LEG_ANG_TARGET  (-0.00f)
#define LQR_LEG_LEN_INIT    0.18f    /* 投入腿长目标 */


/*
 * IMU 轴索引 — 台架第一步必须确认
 * 手法: 手把机头缓慢抬起/压下, 看 pitch 与角速度哪个分量响应最大、符号是否符合
 * 若 pitch 实际落在滚转槽, 只改这两行
 */
#define LQR_IMU_PITCH_IDX    ATTITUDE_PITCH
#define LQR_IMU_ROLL_IDX     ATTITUDE_ROLL
#define LQR_IMU_YAW_IDX      ATTITUDE_YAW
#define LQR_IMU_GYRO_PITCH   1u
#define LQR_IMU_GYRO_YAW     2u

lqr_debug_t lqr_debug;

/* 角度环绕 [-π, π] */
static float LQR_Wrap_Pi(float angle)
{
    while (angle > LEG_PI)  { angle -= LEG_2PI; }
    while (angle < -LEG_PI) { angle += LEG_2PI; }
    return angle;
}

/* 摇杆归一化: 死区 + 限幅 → [-1,1] */
static float LQR_RC_Axis(int16_t raw, uint16_t deadband)
{
    int16_t value;

    value = DR16_Deadline(raw, deadband);
    if (value > DR16_CH_LIMIT)
    {
        value = DR16_CH_LIMIT;
    }
    else if (value < -DR16_CH_LIMIT)
    {
        value = -DR16_CH_LIMIT;
    }
    return (float)value / (float)DR16_CH_LIMIT;
}

/* 腿长目标区间: 手动腿测取机器区间, LQR 再与 K 表域求交 */
static void LQR_Len_Range(uint8_t manual, float *len_min, float *len_max)
{
    *len_min = machine->leg_len_min;
    *len_max = machine->leg_len_max;
    if (!manual)
    {
        *len_min = fmaxf(*len_min, LQR_K_LEN_MIN);
        *len_max = fminf(*len_max, LQR_K_LEN_MAX);
    }
}

/* 初始化 */
void LQR_Init(lqr_state_t *st)
{
    memset(st, 0, sizeof(*st));
    lqr_debug.vel_leg_comp_sign = -1.0f;
    lqr_debug.vel_src = 1u;
    lqr_debug.yaw_hold = 1u;
    lqr_debug.acc_fwd_sign = 1.0f;
    lqr_debug.wheel_enable = 1u;
    lqr_debug.hip_enable = 1u;
    lqr_debug.len_pid_enable = 1u;
    lqr_debug.trq_max_wheel = machine->dji_trq_clamp;
    lqr_debug.trq_max_hip = machine->dm_trq_clamp;
    st->len_eval[0] = -1.0f;
    st->len_eval[1] = -1.0f;
    Lowpass_Init(&st->lpf_vel, LQR_LPF_ALPHA);
    Lowpass_Init(&st->lpf_vel_alt, LQR_LPF_ALPHA);
    Lowpass_Init(&st->lpf_omg_pitch, LQR_LPF_ALPHA);
    Lowpass_Init(&st->lpf_omg_yaw, LQR_LPF_ALPHA);
    Kalman_Accel_Init(&st->kf_vel, 0.0f, LQR_KF_P0, LQR_KF_Q, LQR_KF_R,
                      LQR_KF_P_MAX);
}

/* 前向加速度: 四元数把机体加速度转到世界系, 去重力, 投影到车头水平方向 */
static float LQR_Accel_Forward(const imu_state_t *imu)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float r00;
    float r01;
    float r02;
    float r10;
    float r11;
    float r12;
    float wx;
    float wy;
    float norm;

    q0 = imu->quat[0];
    q1 = imu->quat[1];
    q2 = imu->quat[2];
    q3 = imu->quat[3];
    r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    r01 = 2.0f * (q1 * q2 - q0 * q3);
    r02 = 2.0f * (q1 * q3 + q0 * q2);
    r10 = 2.0f * (q1 * q2 + q0 * q3);
    r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    r12 = 2.0f * (q2 * q3 - q0 * q1);

    /* 世界系水平分量 (重力只在 z, 水平不用减) */
    wx = r00 * imu->acc_g[0] + r01 * imu->acc_g[1] + r02 * imu->acc_g[2];
    wy = r10 * imu->acc_g[0] + r11 * imu->acc_g[1] + r12 * imu->acc_g[2];
    /* 车头方向 = 机体 x 轴在水平面的投影 */
    norm = sqrtf(r00 * r00 + r10 * r10);
    if (norm < 1.0e-3f)
    {
        return 0.0f;
    }
    return (wx * r00 + wy * r10) / norm * LQR_GRAVITY;
}

/* 使能边沿: 腿长目标锁到当前实测, 位移积分清零 (滤波器常跑不复位)
 * LQR 要求实测腿长在 K 表域内才投入, 否则返回 0 保持零力矩——
 * 直接夹到域内会让腿长 PID 瞬间产生几十牛的伸长力, 在使能瞬间把车弹起来
 * 手动腿测任意腿长都投入 (目标锁当前值, 拨轮再拨回区间), 架空/趴地也能先看命令 */
uint8_t LQR_Enable_Latch(lqr_state_t *st, const leg_state_t *leg_l,
                         const leg_state_t *leg_r, uint8_t manual)
{
    float len_min;
    float len_max;

    if (!manual)
    {
        LQR_Len_Range(0u, &len_min, &len_max);
        if (leg_l->output.virtual_leg_length < len_min
            || leg_l->output.virtual_leg_length > len_max
            || leg_r->output.virtual_leg_length < len_min
            || leg_r->output.virtual_leg_length > len_max)
        {
            return 0u;
        }
    }
    st->leg_len_tgt[0] = LQR_LEG_LEN_INIT;
    st->leg_len_tgt[1] = LQR_LEG_LEN_INIT;
    st->leg_ang_tgt[0] = 0.0f;
    st->leg_ang_tgt[1] = 0.0f;
    st->yaw_tgt = st->x[LQR_X_PHI];    /* 朝向锁当前 */
    /* 只清位移积分; 滤波器每拍都在跑, 已是热态, 不复位 */
    st->pos = 0.0f;
    st->x[LQR_X_S] = 0.0f;
    return 1u;
}

/* 遥控 → 目标: 右摇杆Y 前后速度, 右摇杆X 转向, 拨轮 升降; 手动腿测: 左摇杆Y 摆角 */
uint8_t LQR_Target_Update(lqr_state_t *st, const dr16_t *rc, float dt,
                          uint8_t manual)
{
    float axis_vel;
    float axis_yaw;
    float axis_len;
    float axis_ang;
    float len_min;
    float len_max;
    float lo;
    float hi;
    uint8_t i;

    if (rc == NULL || !rc->online)
    {
        return 0u;
    }

    axis_vel = LQR_RC_Axis(rc->ch1, LQR_RC_DEADBAND);
    axis_yaw = LQR_RC_Axis(rc->ch0, LQR_RC_DEADBAND);
    axis_len = LQR_RC_Axis(rc->wheel, LQR_RC_DEADBAND);
    axis_ang = manual ? LQR_RC_Axis(rc->ch3, LQR_RC_DEADBAND) : 0.0f;
    LQR_Len_Range(manual, &len_min, &len_max);

    st->target[LQR_X_S]     = LQR_POS_TARGET;
    st->target[LQR_X_DS]    = axis_vel * LQR_RC_VEL_MAX;
    /* 偏航: 摇杆有输入时目标跟随当前角 (不回正), 回中后锁住; 转向通道取负 (同 Leg2, 作者台架定) */
    if (axis_yaw != 0.0f)
    {
        st->yaw_tgt = st->x[LQR_X_PHI];
    }
    st->target[LQR_X_PHI]   = st->yaw_tgt;
    st->target[LQR_X_DPHI]  = -axis_yaw * LQR_RC_YAW_MAX;
    st->target[LQR_X_THL]   = LQR_LEG_ANG_TARGET;
    st->target[LQR_X_DTHL]  = 0.0f;
    st->target[LQR_X_THR]   = LQR_LEG_ANG_TARGET;
    st->target[LQR_X_DTHR]  = 0.0f;
    st->target[LQR_X_THB]   = 0.0f;
    st->target[LQR_X_DTHB]  = 0.0f;

    /* 腿长目标: 拨轮按速率积分, 限制在腿长工作区间 (手动腿测从区间外投入时只许往区间里拨);
     * 摆角目标: 摇杆直接给 */
    for (i = 0u; i < 2u; i++)
    {
        lo = len_min;
        hi = len_max;
        if (manual)
        {
            lo = fminf(lo, st->leg_len_tgt[i]);
            hi = fmaxf(hi, st->leg_len_tgt[i]);
        }
        st->leg_len_tgt[i] += axis_len * LQR_RC_LEN_RATE * dt;
        st->leg_len_tgt[i] = clampf(st->leg_len_tgt[i], lo, hi);
        st->leg_ang_tgt[i] = axis_ang * LQR_RC_ANG_MAX;
    }
    return 1u;
}

/* 状态估计: 每拍必算, 有效性写 st->valid 并返回 */
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,
                         const leg_state_t *leg_l, const leg_state_t *leg_r,
                         const float wheel_vel[2], float dt)
{
    float pitch;
    float omg_pitch;
    float whl[2];
    float vel[2];

    st->valid = 0u;
    if (imu == NULL || leg_l == NULL || leg_r == NULL || wheel_vel == NULL)
    {
        return 0u;
    }
    if (!imu->online || !leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    st->len[0] = leg_l->output.virtual_leg_length;
    st->len[1] = leg_r->output.virtual_leg_length;

    pitch = imu->euler_rad[LQR_IMU_PITCH_IDX];
    omg_pitch = Lowpass_Update(&st->lpf_omg_pitch,
                               imu->gyro_rad_s[LQR_IMU_GYRO_PITCH]);
    st->roll = imu->euler_rad[LQR_IMU_ROLL_IDX];

    /* 腿摆角/角速度世界系 = 解算输出 + 机体俯仰
     * 本工程 virtual_leg_angle 前摆为正, 模型 θ_ll 前摆为负, 故取反后再加 pitch */
    st->x[LQR_X_THL]  = -leg_l->output.virtual_leg_angle + pitch;
    st->x[LQR_X_DTHL] = -leg_l->output.d_virtual_leg_angle + omg_pitch;
    st->x[LQR_X_THR]  = -leg_r->output.virtual_leg_angle + pitch;
    st->x[LQR_X_DTHR] = -leg_r->output.d_virtual_leg_angle + omg_pitch;
    st->x[LQR_X_THB]  = pitch;
    st->x[LQR_X_DTHB] = omg_pitch;
    st->x[LQR_X_PHI]  = imu->euler_rad[LQR_IMU_YAW_IDX];
    st->x[LQR_X_DPHI] = Lowpass_Update(&st->lpf_omg_yaw,
                                       imu->gyro_rad_s[LQR_IMU_GYRO_YAW]);

    /* 轮子相对地面角速度: 反馈已扣减速比, 再补偿腿摆与俯仰 */
    whl[0] = wheel_vel[0] + lqr_debug.vel_leg_comp_sign
             * leg_l->output.d_virtual_leg_angle - omg_pitch;
    whl[1] = wheel_vel[1] + lqr_debug.vel_leg_comp_sign
             * leg_r->output.d_virtual_leg_angle - omg_pitch;
    st->whl[0] = whl[0];
    st->whl[1] = whl[1];

    /* 机体水平速度: 轮心线速度 + 摆杆摆动 + 摆杆伸缩 */
    vel[0] = whl[0] * machine->wheel_r
           + leg_l->output.virtual_leg_length * st->x[LQR_X_DTHL]
             * cosf(st->x[LQR_X_THL])
           + leg_l->output.d_virtual_leg_length * sinf(st->x[LQR_X_THL]);
    vel[1] = whl[1] * machine->wheel_r
           + leg_r->output.virtual_leg_length * st->x[LQR_X_DTHR]
             * cosf(st->x[LQR_X_THR])
           + leg_r->output.d_virtual_leg_length * sinf(st->x[LQR_X_THR]);
    /* 速度估计两条并行: 低通 / 卡尔曼 (加速度预测 + 运动学观测), vel_src 选一条进 x[1] */
    st->ds_raw = (vel[0] + vel[1]) * 0.5f;
    st->ds_lpf = Lowpass_Update(&st->lpf_vel, st->ds_raw);
    st->a_fwd  = lqr_debug.acc_fwd_sign * LQR_Accel_Forward(imu);
    st->ds_kf  = Kalman_Accel_Update(&st->kf_vel, st->a_fwd, st->ds_raw, dt);
    st->x[LQR_X_DS] = lqr_debug.vel_src ? st->ds_kf : st->ds_lpf;

    /* 对照: 补偿符号取反再算一遍, 只供台架 A/B 看哪条平 */
    whl[0] = wheel_vel[0] - lqr_debug.vel_leg_comp_sign
             * leg_l->output.d_virtual_leg_angle - omg_pitch;
    whl[1] = wheel_vel[1] - lqr_debug.vel_leg_comp_sign
             * leg_r->output.d_virtual_leg_angle - omg_pitch;
    vel[0] = whl[0] * machine->wheel_r
           + leg_l->output.virtual_leg_length * st->x[LQR_X_DTHL]
             * cosf(st->x[LQR_X_THL])
           + leg_l->output.d_virtual_leg_length * sinf(st->x[LQR_X_THL]);
    vel[1] = whl[1] * machine->wheel_r
           + leg_r->output.virtual_leg_length * st->x[LQR_X_DTHR]
             * cosf(st->x[LQR_X_THR])
           + leg_r->output.d_virtual_leg_length * sinf(st->x[LQR_X_THR]);
    st->ds_alt = Lowpass_Update(&st->lpf_vel_alt, (vel[0] + vel[1]) * 0.5f);

    /* 位移积分: 有速度指令时不积分, 避免跟着指令漂 */
    if (st->target[LQR_X_DS] == 0.0f)
    {
        st->pos += st->x[LQR_X_DS] * dt;
    }
    else
    {
        st->pos = 0.0f;
    }
    st->x[LQR_X_S] = st->pos;
    st->valid = 1u;
    return 1u;
}

/* 增益求值 + 状态反馈求和 */
void LQR_Control_Update(lqr_state_t *st)
{
    float K_sym[40];
    float sum;
    uint8_t i;
    uint8_t j;

    /* 腿长变化超阈值才重算: 静态腿长时 40 项多项式是白算 */
    if (fabsf(st->len[0] - st->len_eval[0]) > LQR_K_RECALC_THRESH
        || fabsf(st->len[1] - st->len_eval[1]) > LQR_K_RECALC_THRESH)
    {
        LQR_K_WBR(st->len[0], st->len[1], K_sym);
        for (i = 0u; i < LQR_U_NUM; i++)
        {
            for (j = 0u; j < LQR_X_NUM; j++)
            {
                st->K[i][j] = K_sym[j * LQR_U_NUM + i];
            }
        }
        st->len_eval[0] = st->len[0];
        st->len_eval[1] = st->len[1];
    }

    for (i = 0u; i < LQR_U_NUM; i++)
    {
        sum = 0.0f;
        for (j = 0u; j < LQR_X_NUM; j++)
        {
            if (j == LQR_X_PHI)
            {
                if (!lqr_debug.yaw_hold)
                {
                    continue;   /* 关: 只控角速度 (Leg2 原样) */
                }
                sum += st->K[i][j] * LQR_Wrap_Pi(st->target[j] - st->x[j]);
                continue;
            }
            sum += st->K[i][j] * (st->target[j] - st->x[j]);
        }
        if (!isfinite(sum))
        {
            sum = 0.0f;
        }
        if (i == LQR_U_WL || i == LQR_U_WR)
        {
            st->u[i] = clampf(sum, -lqr_debug.trq_max_wheel,
                              lqr_debug.trq_max_wheel);
        }
        else
        {
            st->u[i] = clampf(sum, -lqr_debug.trq_max_hip,
                              lqr_debug.trq_max_hip);
        }
    }
}
