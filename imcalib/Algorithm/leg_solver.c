#include "leg_solver.h"
#include <math.h>
#include <string.h>

#define LEG_EPS         1.0e-6f
#define LEG_MIN_LENGTH  1.0e-3f

/* 角度环绕 [-π, π] */
static float Leg_Wrap(float angle)
{
    while (angle > LEG_PI)  { angle -= LEG_2PI; }
    while (angle < -LEG_PI) { angle += LEG_2PI; }
    return angle;
}

/* 检查输入有效性 */
static uint8_t Leg_Input_Valid(const leg_state_t *leg)
{
    const leg_input_t *in = &leg->input;
    const leg_config_t *cfg = &leg->config;

    if (!cfg->configured) { return 0u; }
    if (cfg->lu <= LEG_EPS || cfg->lg <= LEG_EPS) { return 0u; }
    if (!isfinite(in->hip_f) || !isfinite(in->hip_b)
        || !isfinite(in->d_hip_f) || !isfinite(in->d_hip_b))
    {
        return 0u;
    }
    return 1u;
}

/* 闭链几何: A/B点 → P点 → 腿长/方向角/虚拟小腿 */
static uint8_t Leg_Solve_Geometry(leg_state_t *leg, leg_solver_cache_t *cache)
{
    float x_a;
    float y_a;
    float x_b;
    float y_b;
    float x_p;
    float y_p;
    float dx;
    float dy;
    float lg_dx2;       /* 2·lg·dx */
    float lg_dy2;       /* 2·lg·dy */
    float ab_sq;        /* |AB|² */
    float disc;
    float root;
    float vs_raw;

    cache->mirror = (leg->config.mirror >= 0) ? 1.0f : -1.0f;
    cache->lu     = leg->config.lu;
    cache->lg     = leg->config.lg;
    cache->qf     = cache->mirror * leg->input.hip_f;
    cache->qb     = cache->mirror * leg->input.hip_b;
    cache->vf     = cache->mirror * leg->input.d_hip_f;
    cache->vb     = cache->mirror * leg->input.d_hip_b;

    /* A = 前杆端点, B = 后杆端点 */
    x_a = cache->lu * cosf(cache->qf);
    y_a = cache->lu * sinf(cache->qf);
    x_b = cache->lu * cosf(cache->qb);
    y_b = cache->lu * sinf(cache->qb);

    /* 求 P 点: 以 A 为圆心 lg 为半径的圆, 与以 B 为圆心 lg 为半径的圆的交点 */
    dx     = x_b - x_a;
    dy     = y_b - y_a;
    lg_dx2 = 2.0f * cache->lg * dx;
    lg_dy2 = 2.0f * cache->lg * dy;
    ab_sq  = dx * dx + dy * dy;
    disc   = lg_dx2 * lg_dx2 + lg_dy2 * lg_dy2 - ab_sq * ab_sq;
    if (disc < -LEG_EPS)
    {
        return 0u;
    }
    if (disc < 0.0f)
    {
        disc = 0.0f;
    }

    /* φ_a = 前杆绝对角, P = A + lg 方向 */
    root         = sqrtf(disc);
    cache->phi_a = 2.0f * atan2f(lg_dy2 + root, lg_dx2 + ab_sq);
    x_p          = x_a + cache->lg * cosf(cache->phi_a);
    y_p          = y_a + cache->lg * sinf(cache->phi_a);
    cache->phi_b = atan2f(y_p - y_b, x_p - x_b);

    /* 虚拟腿长 = |OP| */
    leg->output.virtual_leg_length = sqrtf(x_p * x_p + y_p * y_p);
    if (leg->output.virtual_leg_length < LEG_MIN_LENGTH)
    {
        return 0u;
    }

    /* 虚拟腿摆角 (相对竖直方向) */
    cache->virtual_leg_angle_abs  = atan2f(y_p, x_p);
    leg->output.virtual_leg_angle = Leg_Wrap(LEG_HALF_PI - cache->virtual_leg_angle_abs
                            + leg->config.offset_phi0);

    /* 大腿角 = φ_a + π/2 (竖直向下为0°, 环绕点在竖直处消除跳变) */
    leg->output.thigh_angle = Leg_Wrap(cache->phi_a + LEG_HALF_PI);

    /* 虚拟小腿角 = φ_a - qf - π/2 (小腿相对大腿) */
    vs_raw = Leg_Wrap(cache->phi_a - cache->qf - LEG_HALF_PI);
    leg->output.virtual_shank_angle = Leg_Wrap(cache->mirror * vs_raw);
    return 1u;
}

/* 速度与雅可比: 关节速度 → 腿长/方向角/虚拟小腿速度 */
static uint8_t Leg_Solve_Velocity(leg_state_t *leg, const leg_solver_cache_t *cache)
{
    float sin_ab;
    float sin_fa;
    float sin_fb;
    float sin_bb;
    float sin_0a;
    float sin_0b;
    float cos_0a;
    float cos_0b;
    float jac_a;
    float jac_b;
    float d_vs;

    sin_ab = sinf(cache->phi_a - cache->phi_b);
    if (fabsf(sin_ab) < LEG_EPS)
    {
        return 0u;
    }
    sin_fa = sinf(cache->qf - cache->phi_a);
    sin_fb = sinf(cache->qf - cache->phi_b);
    sin_bb = sinf(cache->qb - cache->phi_b);
    sin_0a = sinf(cache->virtual_leg_angle_abs - cache->phi_a);
    sin_0b = sinf(cache->virtual_leg_angle_abs - cache->phi_b);
    cos_0a = cosf(cache->virtual_leg_angle_abs - cache->phi_a);
    cos_0b = cosf(cache->virtual_leg_angle_abs - cache->phi_b);

    /* P点位置雅可比 */
    leg->output.point_jac[0][0] =  cache->lu * sin_fa * sinf(cache->phi_b) / sin_ab;
    leg->output.point_jac[0][1] = -cache->lu * sin_bb * sinf(cache->phi_a) / sin_ab;
    leg->output.point_jac[1][0] = -cache->lu * sin_fa * cosf(cache->phi_b) / sin_ab;
    leg->output.point_jac[1][1] =  cache->lu * sin_bb * cosf(cache->phi_a) / sin_ab;

    /* 虚拟腿长/摆角雅可比 */
    leg->output.leg_jac[0][0] = -cache->lu * sin_0b * sin_fa / sin_ab;
    leg->output.leg_jac[0][1] =  cache->lu * sin_0a * sin_bb / sin_ab;
    leg->output.leg_jac[1][0] =  cache->lu * cos_0b * sin_fa / (leg->output.virtual_leg_length * sin_ab);
    leg->output.leg_jac[1][1] = -cache->lu * cos_0a * sin_bb / (leg->output.virtual_leg_length * sin_ab);
    leg->output.d_virtual_leg_length = leg->output.leg_jac[0][0] * cache->vf
                                     + leg->output.leg_jac[0][1] * cache->vb;
    leg->output.d_virtual_leg_angle  = leg->output.leg_jac[1][0] * cache->vf
                                     + leg->output.leg_jac[1][1] * cache->vb;

    /* 虚拟小腿雅可比: vshank_jac[0]=后髋, [1]=前髋 */
    jac_a   = cache->lu * sin_bb / (cache->lg * sin_ab);
    jac_b   = -cache->lu * sin_fb / (cache->lg * sin_ab) - 1.0f;
    d_vs    = jac_a * cache->vb + jac_b * cache->vf;
    leg->output.vshank_jac[0] = jac_a;
    leg->output.vshank_jac[1] = jac_b;
    leg->output.d_virtual_shank_angle = cache->mirror * d_vs;
    return 1u;
}

/* 力矩映射: leg_jac 转置 → force_map */
static void Leg_Solve_Force_Map(leg_state_t *leg)
{
    float tau_f_test;
    float tau_b_test;
    float force_test;
    float torque_test;

    leg->output.force_map[0][0] = leg->output.leg_jac[0][0];
    leg->output.force_map[0][1] = leg->output.leg_jac[1][0];
    leg->output.force_map[1][0] = leg->output.leg_jac[0][1];
    leg->output.force_map[1][1] = leg->output.leg_jac[1][1];
    leg->output.force_det = leg->output.force_map[0][0] * leg->output.force_map[1][1]
                          - leg->output.force_map[0][1] * leg->output.force_map[1][0];
    leg->output.force_valid = (uint8_t)(fabsf(leg->output.force_det) >= LEG_EPS);
    if (!leg->output.force_valid)
    {
        return;
    }

    /* 自检: 给定力=1,力矩=0.5 反算误差 */
    tau_f_test = leg->output.force_map[0][0] + 0.5f * leg->output.force_map[0][1];
    tau_b_test = leg->output.force_map[1][0] + 0.5f * leg->output.force_map[1][1];
    force_test =  ( leg->output.force_map[1][1] * tau_f_test
                  - leg->output.force_map[0][1] * tau_b_test) / leg->output.force_det;
    torque_test = (-leg->output.force_map[1][0] * tau_f_test
                  + leg->output.force_map[0][0] * tau_b_test) / leg->output.force_det;
    leg->output.force_test_error = fmaxf(fabsf(force_test - 1.0f),
                                         fabsf(torque_test - 0.5f));
}

/* 虚拟力/力矩 → 电机力矩 */
uint8_t Leg_Force_Map_Forward(const leg_state_t *leg, float force,
                              float torque, float output[2])
{
    if (leg == NULL || output == NULL || !leg->output.force_valid)
    {
        return 0u;
    }
    if (!isfinite(force) || !isfinite(torque))
    {
        return 0u;
    }

    output[0] = leg->output.force_map[0][0] * force + leg->output.force_map[0][1] * torque;
    output[1] = leg->output.force_map[1][0] * force + leg->output.force_map[1][1] * torque;
    return 1u;
}

/* 初始化 */
void Leg_Init(leg_state_t *leg)
{
    if (leg == NULL) { return; }
    memset(leg, 0, sizeof(*leg));
}

/* 三层求解: 几何 → 速度 → 力矩映射 */
uint8_t Leg_Solve(leg_state_t *leg)
{
    leg_solver_cache_t cache;

    if (leg == NULL)
    {
        return 0u;
    }
    memset(&leg->output, 0, sizeof(leg->output));
    if (!Leg_Input_Valid(leg))
    {
        return 0u;
    }
    if (!Leg_Solve_Geometry(leg, &cache))
    {
        return 0u;
    }
    if (!Leg_Solve_Velocity(leg, &cache))
    {
        return 0u;
    }
    Leg_Solve_Force_Map(leg);
    leg->output.valid = 1u;
    return 1u;
}
