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

    if (cfg->lu <= LEG_EPS || cfg->lg <= LEG_EPS) return 0u;
    if (!isfinite(in->hip_f) || !isfinite(in->hip_b)
        || !isfinite(in->d_hip_f) || !isfinite(in->d_hip_b))
        return 0u;
    return 1u;
}

/* 初始化状态 */
void Leg_Init(leg_state_t *leg)
{
    if (leg == NULL) return;
    memset(leg, 0, sizeof(*leg));
}

/* 求解运动学 */
uint8_t Leg_Solve(leg_state_t *leg)
{
    float qf;
    float qb;
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
    float phi_a;
    float phi_b;
    float x_p;
    float y_p;
    float sin_ab;
    float sin_fa;
    float sin_bb;
    float sin_0a;
    float sin_0b;
    float cos_0a;
    float cos_0b;
    float phi0_abs;
    float lu;
    float lg;

    if (leg == NULL || !Leg_Input_Valid(leg)) return 0u;
    memset(&leg->output, 0, sizeof(leg->output));

    lu = leg->config.lu;
    lg = leg->config.lg;
    qf = leg->input.hip_f + leg->config.offset_f;
    qb = leg->input.hip_b + leg->config.offset_b;
    s_f = sinf(qf);
    c_f = cosf(qf);
    s_b = sinf(qb);
    c_b = cosf(qb);

    x_a = lu * c_f;
    y_a = lu * s_f;
    x_b = lu * c_b;
    y_b = lu * s_b;
    /* A、B 为两侧上连杆端点。 */
    dx = x_b - x_a;
    dy = y_b - y_a;
    a = 2.0f * lg * dx;
    b = 2.0f * lg * dy;
    c = dx * dx + dy * dy;
    disc = a * a + b * b - c * c;
    if (disc < -LEG_EPS) return 0u;
    if (disc < 0.0f) disc = 0.0f;

    /* 闭链几何求解足端 P。 */
    root = sqrtf(disc);
    phi_a = 2.0f * atan2f(b + root, a + c);
    x_p = x_a + lg * cosf(phi_a);
    y_p = y_a + lg * sinf(phi_a);
    phi_b = atan2f(y_p - y_b, x_p - x_b);
    leg->output.l0 = sqrtf(x_p * x_p + y_p * y_p);
    if (leg->output.l0 < LEG_MIN_LENGTH) return 0u;

    phi0_abs = atan2f(y_p, x_p);
    leg->output.phi0 = Leg_Wrap(phi0_abs - LEG_HALF_PI);
    leg->output.virtual_shank = Leg_Wrap(phi_b - qb - LEG_HALF_PI);

    /* 两条下连杆共线时雅可比奇异。 */
    sin_ab = sinf(phi_a - phi_b);
    if (fabsf(sin_ab) < LEG_EPS) return 0u;
    sin_fa = sinf(qf - phi_a);
    sin_bb = sinf(qb - phi_b);
    sin_0a = sinf(phi0_abs - phi_a);
    sin_0b = sinf(phi0_abs - phi_b);
    cos_0a = cosf(phi0_abs - phi_a);
    cos_0b = cosf(phi0_abs - phi_b);

    leg->output.point_jac[0][0] = lu * sin_fa * sinf(phi_b) / sin_ab;
    leg->output.point_jac[0][1] = -lu * sin_bb * sinf(phi_a) / sin_ab;
    leg->output.point_jac[1][0] = -lu * sin_fa * cosf(phi_b) / sin_ab;
    leg->output.point_jac[1][1] = lu * sin_bb * cosf(phi_a) / sin_ab;

    /* 极坐标雅可比：髋角速度到腿长和腿角速度。 */
    leg->output.leg_jac[0][0] = -lu * sin_0b * sin_fa / sin_ab;
    leg->output.leg_jac[0][1] = lu * sin_0a * sin_bb / sin_ab;
    leg->output.leg_jac[1][0] = -lu * cos_0b * sin_fa
        / (leg->output.l0 * sin_ab);
    leg->output.leg_jac[1][1] = lu * cos_0a * sin_bb
        / (leg->output.l0 * sin_ab);

    leg->output.force_map[0][0] = leg->output.leg_jac[0][0];
    leg->output.force_map[0][1] = leg->output.leg_jac[1][0];
    leg->output.force_map[1][0] = leg->output.leg_jac[0][1];
    leg->output.force_map[1][1] = leg->output.leg_jac[1][1];
    leg->output.force_det = leg->output.force_map[0][0] * leg->output.force_map[1][1]
        - leg->output.force_map[0][1] * leg->output.force_map[1][0];
    leg->output.force_valid = (uint8_t)(fabsf(leg->output.force_det) >= LEG_EPS);

    /* 雅可比前向映射得到虚拟腿速度。 */
    leg->output.dl0 = leg->output.leg_jac[0][0] * leg->input.d_hip_f
        + leg->output.leg_jac[0][1] * leg->input.d_hip_b;
    leg->output.dphi0 = leg->output.leg_jac[1][0] * leg->input.d_hip_f
        + leg->output.leg_jac[1][1] * leg->input.d_hip_b;
    leg->output.d_virtual_shank = lu * sin_fa
        / (lg * sinf(phi_b - phi_a)) * leg->input.d_hip_f
        + (-lu * sinf(qb - phi_a) / (lg * sinf(phi_b - phi_a)) - 1.0f)
        * leg->input.d_hip_b;
    leg->output.valid = 1u;
    return 1u;
}
