#include "leg_solver.h"
#include <math.h>
#include <string.h>

#define LEG_PI          3.14159265358979f
#define LEG_HALF_PI     1.57079632679490f
#define LEG_EPS         1.0e-6f
#define LEG_MIN_LENGTH  1.0e-3f

/* 环绕角度 */
static float Leg_Wrap(float angle)
{
    while (angle > LEG_PI) angle -= 2.0f * LEG_PI;
    while (angle < -LEG_PI) angle += 2.0f * LEG_PI;
    return angle;
}

/* 检查输入 */
static uint8_t Leg_Input_Valid(const leg_state_t *leg)
{
    const leg_input_t *in = &leg->input;
    const leg_config_t *cfg = &leg->config;

    if (!cfg->configured) return 0u;
    if (cfg->lu <= LEG_EPS || cfg->lg <= LEG_EPS) return 0u;
    if (!isfinite(in->hip_f) || !isfinite(in->hip_b)|| !isfinite(in->d_hip_f) || !isfinite(in->d_hip_b))
    {
        return 0u;
    }
    return 1u;
}

typedef struct {
    float qf;
    float qb;
    float vf;
    float vb;
    float mirror;
    float lu;
    float lg;
    float phi_a;
    float phi_b;
    float phi0_abs;
} leg_solver_cache_t;

/* 闭链几何 */
static uint8_t Leg_Solve_Geometry(leg_state_t *leg, leg_solver_cache_t *cache)
{
    float s_f;
    float c_f;
    float s_b;
    float c_b;
    float x_a;
    float y_a;
    float x_b;
    float y_b;
    float dx;
    float dy;
    float a;
    float b;
    float c;
    float disc;
    float root;
    float x_p;
    float y_p;
    float vs_raw;

    cache->mirror   = (leg->config.mirror >= 0) ? 1.0f : -1.0f;
    cache->lu       = leg->config.lu;
    cache->lg       = leg->config.lg;
    cache->qf       = cache->mirror * leg->input.hip_f;
    cache->qb       = cache->mirror * leg->input.hip_b;
    cache->vf       = cache->mirror * leg->input.d_hip_f;
    cache->vb       = cache->mirror * leg->input.d_hip_b;
    s_f             = sinf(cache->qf);
    c_f             = cosf(cache->qf);
    s_b             = sinf(cache->qb);
    c_b             = cosf(cache->qb);

    x_a             = cache->lu * c_f;
    y_a             = cache->lu * s_f;
    x_b             = cache->lu * c_b;
    y_b             = cache->lu * s_b;
    dx              = x_b - x_a;
    dy              = y_b - y_a;
    a               = 2.0f * cache->lg * dx;
    b               = 2.0f * cache->lg * dy;
    c               = dx * dx + dy * dy;
    disc            = a * a + b * b - c * c;
    if (disc < -LEG_EPS)
    {
        return 0u;
    }
    if (disc < 0.0f)
    {
        disc = 0.0f;
    }

    root            = sqrtf(disc);
    cache->phi_a    = 2.0f * atan2f(b + root, a + c);
    x_p             = x_a + cache->lg * cosf(cache->phi_a);
    y_p             = y_a + cache->lg * sinf(cache->phi_a);
    cache->phi_b    = atan2f(y_p - y_b, x_p - x_b);
    leg->output.l0  = sqrtf(x_p * x_p + y_p * y_p);
    if (leg->output.l0 < LEG_MIN_LENGTH)
    {
        return 0u;
    }

    cache->phi0_abs                     = atan2f(y_p, x_p);
    leg->output.phi0                    = Leg_Wrap(LEG_HALF_PI - cache->phi0_abs + leg->config.offset_phi0);
    leg->output.lower_angle             = cache->phi_a;
    vs_raw                              = Leg_Wrap(cache->phi_a - cache->qf - LEG_HALF_PI);
    leg->output.formula_virtual_shank   = vs_raw;
    leg->output.virtual_shank           = Leg_Wrap(cache->mirror * vs_raw);
    return 1u;
}

/* 速度与雅可比 */
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
    sin_0a = sinf(cache->phi0_abs - cache->phi_a);
    sin_0b = sinf(cache->phi0_abs - cache->phi_b);
    cos_0a = cosf(cache->phi0_abs - cache->phi_a);
    cos_0b = cosf(cache->phi0_abs - cache->phi_b);

    leg->output.point_jac[0][0] =  cache->lu * sin_fa * sinf(cache->phi_b) / sin_ab;
    leg->output.point_jac[0][1] = -cache->lu * sin_bb * sinf(cache->phi_a) / sin_ab;
    leg->output.point_jac[1][0] = -cache->lu * sin_fa * cosf(cache->phi_b) / sin_ab;
    leg->output.point_jac[1][1] =  cache->lu * sin_bb * cosf(cache->phi_a) / sin_ab;

    leg->output.leg_jac[0][0] = -cache->lu * sin_0b * sin_fa / sin_ab;
    leg->output.leg_jac[0][1] =  cache->lu * sin_0a * sin_bb / sin_ab;
    leg->output.leg_jac[1][0] =  cache->lu * cos_0b * sin_fa / (leg->output.l0 * sin_ab);
    leg->output.leg_jac[1][1] = -cache->lu * cos_0a * sin_bb / (leg->output.l0 * sin_ab);
    leg->output.dl0           = leg->output.leg_jac[0][0] * cache->vf + leg->output.leg_jac[0][1] * cache->vb;
    leg->output.dphi0         = leg->output.leg_jac[1][0] * cache->vf + leg->output.leg_jac[1][1] * cache->vb;

    jac_a   = cache->lu * sin_bb / (cache->lg * sin_ab);
    jac_b   = -cache->lu * sin_fb / (cache->lg * sin_ab) - 1.0f;
    d_vs    = jac_a * cache->vb + jac_b * cache->vf;
    leg->output.vshank_jac[0]   = jac_a;
    leg->output.vshank_jac[1]   = jac_b;
    leg->output.d_virtual_shank = cache->mirror * d_vs;
    return 1u;
}

/* 力矩映射 */
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
    leg->output.force_det = leg->output.force_map[0][0] * leg->output.force_map[1][1]- leg->output.force_map[0][1] * leg->output.force_map[1][0];
    leg->output.force_valid = (uint8_t)(fabsf(leg->output.force_det) >= LEG_EPS);
    if (!leg->output.force_valid)
    {
        return;
    }

    tau_f_test = leg->output.force_map[0][0] + 0.5f * leg->output.force_map[0][1];
    tau_b_test = leg->output.force_map[1][0] + 0.5f * leg->output.force_map[1][1];
    force_test =  ( leg->output.force_map[1][1] * tau_f_test - leg->output.force_map[0][1] * tau_b_test) / leg->output.force_det;
    torque_test = (-leg->output.force_map[1][0] * tau_f_test + leg->output.force_map[0][0] * tau_b_test) / leg->output.force_det;
    leg->output.force_test_error = fmaxf(fabsf(force_test - 1.0f),fabsf(torque_test - 0.5f));
}

/* 虚拟力到电机 */
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

/* 初始化状态 */
void Leg_Init(leg_state_t *leg)
{
    if (leg == NULL) return;
    memset(leg, 0, sizeof(*leg));
}

/* 三层求解 */
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
