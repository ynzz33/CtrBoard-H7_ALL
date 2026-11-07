/*
 * 组装十维平衡状态和目标，按左右实测腿长求K，计算双轮力矩与双腿摆矩。
 * 位移积分、速度/偏航目标在此维护；轴向腿力由leg_balance选择四态控制或非NORMAL PD。
 */

#include "lqr_balance.h"
#include "lqr_gain_table.h"
#include "machine_config.h"
#include "math_utils.h"

#include <math.h>
#include <string.h>

#define LQR_GRAVITY         9.81f
#define LQR_K_RECALC_THRESH 0.0005f

/*
 * 世界系腿角平衡点偏移：CKP用LQR_OFF_WBR按腿长给目标，不是固定0。
 * 本机5001实测静止中点随腿长移动，固定0会把腿角调偏。
 */
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
static void LQR_Offset_Eval(const float len[2], float off[3])
{
    float lL = clampf(len[0], 0.15f, 0.31f);
    float lR = clampf(len[1], 0.15f, 0.31f);
    /* 5001静止中点：0.155m≈-0.027、0.219m≈+0.005(左)；0.148m≈-0.028、0.215m≈+0.014(右)。 */
    off[0] = 0.500f * lL - 0.1045f;
    off[1] = 0.627f * lR - 0.1208f;
    off[2] = 0.0f;
}
#else
static void LQR_Offset_Eval(const float len[2], float off[3])
{
    (void)len;
    off[0] = 0.0f;
    off[1] = 0.0f;
    off[2] = 0.0f;
}
#endif

/*
 * IMU 轴索引 — 台架第一步必须确认
 * 手法: 手把机头缓慢抬起/压下, 看 pitch 与角速度哪个分量响应最大、符号是否符合
 * 下列索引属于现有安装映射；发现疑点先由作者台架确认，不依据注释直接改轴或符号。
 */
#define LQR_IMU_PITCH_IDX    ATTITUDE_ROLL
#define LQR_IMU_ROLL_IDX     ATTITUDE_PITCH
#define LQR_IMU_YAW_IDX      ATTITUDE_YAW
#define LQR_IMU_GYRO_PITCH   1u
#define LQR_IMU_GYRO_ROLL    0u
#define LQR_IMU_GYRO_YAW     2u

lqr_debug_t lqr_debug;
/* 初始化 */
void LQR_Init(lqr_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->control_reference_scale=1.0f;
    lqr_debug.vel_leg_comp_sign = -1.0f;
    lqr_debug.yaw_hold = machine->lqr.reference.yaw_hold;
    lqr_debug.yaw_rate_hold = machine->lqr.reference.yaw_rate_hold;
    lqr_debug.pos_hold = machine->lqr.position_hold.pos_hold;
    lqr_debug.acc_fwd_sign = 1.0f;
    lqr_debug.pitch_comp_sign = -1.0f;  /* 现行公式 */
    lqr_debug.pos_arm_vel = machine->lqr.position_hold.pos_arm_vel;
    lqr_debug.pos_leg_tol = machine->lqr.position_hold.pos_leg_tol;
    lqr_debug.wheel_enable = machine->lqr.output.wheel_enable;
    lqr_debug.leg_swing_enable = machine->lqr.output.leg_swing_enable;
    lqr_debug.air_release_enable = 1u;
    lqr_debug.trq_max_wheel = machine->wheel.dji_trq_clamp;
    lqr_debug.trq_max_hip = machine->hip.dm_trq_clamp;
    st->len_eval[0] = -1.0f;
    st->len_eval[1] = -1.0f;
    LPF_Init(&st->lpf_omg_pitch, machine->lqr.filter.lpf_alpha[0]);
    LPF_Init(&st->lpf_omg_yaw, machine->lqr.filter.lpf_alpha[1]);
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
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    wx = r00 * imu->acc_g[0] - r01 * imu->acc_g[1] - r02 * imu->acc_g[2];
    wy = r10 * imu->acc_g[0] - r11 * imu->acc_g[1] - r12 * imu->acc_g[2];
#else
    wx = r00 * imu->acc_g[0] + r01 * imu->acc_g[1] + r02 * imu->acc_g[2];
    wy = r10 * imu->acc_g[0] + r11 * imu->acc_g[1] + r12 * imu->acc_g[2];
#endif
    /* 车头方向 = 机体 x 轴在水平面的投影 */
    norm = sqrtf(r00 * r00 + r10 * r10);
    if (norm < 1.0e-3f)
    {
        return 0.0f;
    }
    return (wx * r00 + wy * r10) / norm * LQR_GRAVITY;
}

/*
 * 投入时建立位置/朝向基准并读取公共腿长；腿几何由执行层检查。
 * 允许从较短实测腿长撑起；目标只按现有机械范围裁剪，K求值仍在表范围取边界，不伪造实测长度。
 */
uint8_t LQR_Enable_Latch(lqr_state_t *st, const rc_command_t *cmd)
{
    st->leg_length_target[0] = clampf(cmd->leg_length_target, machine->leg.leg_len_min, machine->leg.leg_len_max);
    st->leg_length_target[1] = st->leg_length_target[0];
    st->yaw_target = st->x[LQR_X_PHI];    /* 朝向锁当前 */
    st->velocity_target = 0.0f;
    /* 只清位移积分; 滤波器每拍都在跑, 已是热态, 不复位 */
    st->pos = 0.0f;
    st->pos_armed = 0u;
    st->target[LQR_X_S] = 0.0f;
    st->x[LQR_X_S] = 0.0f;
    return 1u;
}

/*
 * 将统一遥控命令变成速度/偏航及最终腿长目标。
 * RC只限制公共拨轮范围；LQR在此按现有机械范围裁剪腿长，不回写公共命令。
 */
uint8_t LQR_Target_Update(lqr_state_t *st, const rc_command_t *cmd)
{
    float off[3];

    if (!cmd->online)
    {
        return 0u;
    }
    LQR_Offset_Eval(st->len, off);
    /* 只发布原目标采用的静态工作点，原LQR目标/反馈计算保持不变。 */
    memcpy(st->operating_point,LQR_Gain_Info()->x_eq,sizeof(st->operating_point));
    if(!st->air_control)
    {
        st->operating_point[LQR_X_THL]+=off[0];
        st->operating_point[LQR_X_THR]+=off[1];
        st->operating_point[LQR_X_THB]+=off[2];
    }

    /* 位置参考仅在停车积分已锁定后生效；当前指令优先，行进立即归零。 */
    st->target[LQR_X_S] = cmd->velocity_target == 0.0f && st->pos_armed && !st->air_control ? machine->lqr.position_hold.pos_target : 0.0f;
    st->velocity_target            = cmd->velocity_target;
    st->target[LQR_X_DS]    = cmd->velocity_target;
    /* 转向保持朝向 */
    if (cmd->yaw_rate_target != 0.0f)
    {
        st->yaw_target = st->x[LQR_X_PHI];
    }
    st->target[LQR_X_PHI]   = st->yaw_target;
    st->target[LQR_X_DPHI]  = cmd->yaw_rate_target;
    st->target[LQR_X_THL]   = machine->lqr.reference.leg_trim[0] + (st->air_control ? 0.0f : off[0]);
    st->target[LQR_X_DTHL]  = 0.0f;
    st->target[LQR_X_THR]   = machine->lqr.reference.leg_trim[1] + (st->air_control ? 0.0f : off[1]);
    st->target[LQR_X_DTHR]  = 0.0f;
    st->target[LQR_X_THB]   = machine->lqr.reference.pitch_trim + (st->air_control ? 0.0f : off[2]);
    st->target[LQR_X_DTHB]  = 0.0f;

    st->leg_length_target[0] = clampf(cmd->leg_length_target, machine->leg.leg_len_min, machine->leg.leg_len_max);
    st->leg_length_target[1] = st->leg_length_target[0];
    return 1u;
}

/*
 * 将融合速度写入LQR状态，并按低速/腿姿态条件建立停车位置积分。
 * AIR保留真实速度状态但冻结停车积分；恢复地面时是否使用轮速观测由执行层决定。
 */
void LQR_Velocity_Apply(lqr_state_t *st, float velocity, float dt)
{
    uint8_t legs_ready;

    st->pos_trace = 0u;
    st->ds_kf = velocity;
    st->x[LQR_X_DS] = velocity;
    if (st->air_control)
    {
        st->pos_trace |= 128u;
        st->x[LQR_X_S] = st->pos;
        return;
    }
    legs_ready = (uint8_t)(fabsf(Angle_Wrap_Pi(st->x[LQR_X_THL] - machine->lqr.reference.leg_trim[0])) <= lqr_debug.pos_leg_tol && fabsf(Angle_Wrap_Pi(st->x[LQR_X_THR] - machine->lqr.reference.leg_trim[1])) <= lqr_debug.pos_leg_tol);
    st->pos_trace |= st->target[LQR_X_DS] == 0.0f ? 1u : 0u;
    st->pos_trace |= legs_ready ? 2u : 0u;
    st->pos_trace |= lqr_debug.pos_arm_vel <= 0.0f || fabsf(st->x[LQR_X_DS]) < lqr_debug.pos_arm_vel ? 4u : 0u;
    if (st->target[LQR_X_DS] != 0.0f)
    {
        st->pos = 0.0f;
        st->pos_armed = 0u;
    }
    else if (fabsf(st->pos) >= machine->lqr.position_hold.pos_reset_dist)
    {
        st->pos_trace |= 64u;
        st->pos = 0.0f;
        st->pos_armed = 0u;
    }
    else if (legs_ready)
    {
        if (!st->pos_armed && (lqr_debug.pos_arm_vel <= 0.0f || fabsf(st->x[LQR_X_DS]) < lqr_debug.pos_arm_vel))
        {
            /* 此刻设为位置基准 */
            st->pos = 0.0f;
            st->pos_armed = 1u;
            st->pos_trace |= 32u;
        }
        if (st->pos_armed)
        {
            st->pos += st->x[LQR_X_DS] * dt;
            st->pos_trace |= 16u;
        }
    }
    st->pos_trace |= st->pos_armed ? 8u : 0u;
    st->pos_trace |= st->target[LQR_X_DS] == 0.0f && st->pos_armed && !legs_ready ? 256u : 0u;
    /* 腿姿态偏离只冻结，不清除已累计位移。 */
    st->x[LQR_X_S] = st->pos;
    if (!isfinite(st->pos))
    {
        st->valid = 0u;
    }
}

/*
 * 从IMU和双腿几何构建10维状态及原始轮速运动学观测。
 * 输入数值已由IMU/腿发布端验证，这里读取可用标志；融合速度随后由执行层回填。
 * 本函数不根据倒置补角修改LQR pitch，pitch_world与原状态需分开观察。
 */
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,const leg_state_t *leg_l, const leg_state_t *leg_r,const float wheel_vel[2])
{
    float pitch;
    float omg_pitch;
    float whl[2];
    float vel[2];

    st->valid = 0u;
    if (!imu->online || !leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    st->len[0] = leg_l->output.virtual_leg_length;
    st->len[1] = leg_r->output.virtual_leg_length;

    pitch = imu->euler_rad[LQR_IMU_PITCH_IDX];
    omg_pitch = LPF_Update(&st->lpf_omg_pitch, imu->gyro_rad_s[LQR_IMU_GYRO_PITCH]);
    st->roll = imu->euler_rad[LQR_IMU_ROLL_IDX];
    st->roll_rate = imu->gyro_rad_s[LQR_IMU_GYRO_ROLL];

    /* 前摆正，转世界系 */
    st->x[LQR_X_THL]  = leg_l->output.virtual_leg_angle - pitch;
    st->x[LQR_X_DTHL] = leg_l->output.d_virtual_leg_angle - omg_pitch;
    st->x[LQR_X_THR]  = leg_r->output.virtual_leg_angle - pitch;
    st->x[LQR_X_DTHR] = leg_r->output.d_virtual_leg_angle - omg_pitch;
    st->x[LQR_X_THB]  = pitch;
    st->x[LQR_X_DTHB] = omg_pitch;
    st->x[LQR_X_PHI]  = imu->euler_rad[LQR_IMU_YAW_IDX];
    st->x[LQR_X_DPHI] = LPF_Update(&st->lpf_omg_yaw, imu->gyro_rad_s[LQR_IMU_GYRO_YAW]);

    /* 轮子相对地面角速度: 反馈已扣减速比, 再补偿腿摆与俯仰 (俯仰项符号 A/B, 见 LQR_PLAN §六 ①e) */
    whl[0] = wheel_vel[0] + lqr_debug.vel_leg_comp_sign * leg_l->output.d_virtual_leg_angle + lqr_debug.pitch_comp_sign * omg_pitch;
    whl[1] = wheel_vel[1] + lqr_debug.vel_leg_comp_sign * leg_r->output.d_virtual_leg_angle + lqr_debug.pitch_comp_sign * omg_pitch;
    st->whl[0] = whl[0];
    st->whl[1] = whl[1];

    /* 机体水平速度: 轮心线速度 + 摆杆摆动 + 摆杆伸缩 */
    vel[0] = whl[0] * machine->wheel.wheel_r - leg_l->output.virtual_leg_length * st->x[LQR_X_DTHL] * cosf(st->x[LQR_X_THL]) - leg_l->output.d_virtual_leg_length * sinf(st->x[LQR_X_THL]);
    vel[1] = whl[1] * machine->wheel.wheel_r - leg_r->output.virtual_leg_length * st->x[LQR_X_DTHR] * cosf(st->x[LQR_X_THR]) - leg_r->output.d_virtual_leg_length * sinf(st->x[LQR_X_THR]);
    /* 组装速度观测，融合由执行任务调用slip模块。 */
    st->ds_raw = (vel[0] + vel[1]) * 0.5f;
    st->a_fwd  = lqr_debug.acc_fwd_sign * LQR_Accel_Forward(imu);
    if (!isfinite(st->ds_raw) || !isfinite(st->a_fwd))
    {
        return 0u;
    }
    st->valid = 1u;
    return st->valid;
}

/* 按腿长更新K，计算目标与状态的反馈力矩；NORMAL可约束偏航参考，AIR按专用列选择输出。 */
void LQR_Control_Update(lqr_state_t *st, uint8_t guard_permit)
{
    float K_sym[40];
    float len_l;
    float len_r;
    float sum;
    float term;
    float hip_limit;
    float reference_scale = 1.0f;
    st->control_reference_scale=1.0f;

    st->gain_valid = 0u;
    memset(st->u, 0, sizeof(st->u));
    if (!st->valid)
    {
        return;
    }
    /* 增益多项式只在生成网格内有效，实测腿长越界时取边界增益。 */
    len_l = clampf(st->len[0], LQR_Gain_Info()->len_min, LQR_Gain_Info()->len_max);
    len_r = clampf(st->len[1], LQR_Gain_Info()->len_min, LQR_Gain_Info()->len_max);
    if (fabsf(len_l - st->len_eval[0]) > LQR_K_RECALC_THRESH || fabsf(len_r - st->len_eval[1]) > LQR_K_RECALC_THRESH)
    {
        if (!LQR_Gain_Eval(len_l, len_r, K_sym))
        {
            return;
        }
        for (uint8_t i = 0u; i < LQR_U_NUM; i++)
        {
            for (uint8_t j = 0u; j < LQR_X_NUM; j++)
            {
                st->K[i][j] = K_sym[j * LQR_U_NUM + i];
            }
        }
        st->len_eval[0] = len_l;
        st->len_eval[1] = len_r;
    }

    /* 当前K已就绪：只缩放偏航参考部分，测量反馈和其他八列保持原值。 */
    if (guard_permit && !st->air_control && machine->lqr.motion_guard.enabled)
    {
        const motion_guard_input_t input = {
            .length = {st->len[0], st->len[1]},
            .length_target = fmaxf(st->leg_length_target[0], st->leg_length_target[1]),
            .velocity = st->ds_kf, .velocity_request = st->velocity_target,
            .yaw_rate = st->x[LQR_X_DPHI],
            .yaw_request = lqr_debug.yaw_rate_hold ? st->target[LQR_X_DPHI] + LQR_Gain_Info()->x_eq[LQR_X_DPHI] : 0.0f,
            .heading_error = lqr_debug.yaw_hold ? Angle_Wrap_Pi(st->target[LQR_X_PHI] + LQR_Gain_Info()->x_eq[LQR_X_PHI] - st->x[LQR_X_PHI]) : 0.0f,
            .heading_gain_diff = st->K[LQR_U_WL][LQR_X_PHI] - st->K[LQR_U_WR][LQR_X_PHI],
            .rate_gain_diff = st->K[LQR_U_WL][LQR_X_DPHI] - st->K[LQR_U_WR][LQR_X_DPHI],
            .track = machine->lqr.turn_comp.track, .height_offset = machine->lqr.turn_comp.height_offset,
            .yaw_max = machine->rc.yaw_max, .dt = MACHINE_LQR_DT, .permit = 1u,
        };
        if (!Motion_Guard_Update(&motion_guard, &machine->lqr.motion_guard, &input)) { return; }
        reference_scale = motion_guard.reference_scale;
    }
    else
    {
        Motion_Guard_Reset(&motion_guard);
    }
    st->control_reference_scale=reference_scale;
    for (uint8_t i = 0u; i < LQR_U_NUM; i++)
    {
        sum = st->air_control ? 0.0f : LQR_Gain_Info()->u_eq[i];
        for (uint8_t j = 0u; j < LQR_X_NUM; j++)
        {
            if (st->air_control && (i == LQR_U_WL || i == LQR_U_WR || j < LQR_X_THL || j > LQR_X_DTHR))
            {
                term = 0.0f;
            }
            else if (j == LQR_X_PHI)
            {
                term = lqr_debug.yaw_hold ? st->K[i][j] * (reference_scale * Angle_Wrap_Pi(st->target[j] + LQR_Gain_Info()->x_eq[j] - st->x[j])) : 0.0f;
            }
            else if ((j == LQR_X_DPHI && !lqr_debug.yaw_rate_hold) || (j == LQR_X_S && !lqr_debug.pos_hold))
            {
                term = 0.0f;                        /* 关: 该列不参与 */
            }
            else if (st->air_control && (j == LQR_X_THL || j == LQR_X_THR))
            {
                term = st->K[i][j] * remainderf(st->target[j] + LQR_Gain_Info()->x_eq[j] - st->x[j], LEG_2PI);
            }
            else
            {
                term = st->K[i][j] * ((j == LQR_X_DPHI ? reference_scale : 1.0f) * (st->target[j] + LQR_Gain_Info()->x_eq[j]) - st->x[j]);
            }
            sum += term;
        }
        if (!isfinite(sum))
        {
            memset(st->u, 0, sizeof(st->u));
            return;
        }
        if (i == LQR_U_WL || i == LQR_U_WR)
        {
            st->u[i] = clampf(sum, -lqr_debug.trq_max_wheel, lqr_debug.trq_max_wheel);
        }
        else
        {
            hip_limit = st->air_control ? fminf(st->air_hip_max, lqr_debug.trq_max_hip) : lqr_debug.trq_max_hip;
            st->u[i] = clampf(sum, -hip_limit, hip_limit);
        }
    }
    st->gain_valid = 1u;
}
