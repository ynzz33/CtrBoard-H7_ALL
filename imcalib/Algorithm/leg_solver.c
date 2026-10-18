/*
 * 文件职责：五连杆运动学、速度和力映射（leg_solver.c）。
 * 输入/来源：机器表几何与左右腿前/后髋角度、角速度。
 * 输出/去向：求虚拟腿长/腿角、关节速度、雅可比，并在轴向力/摆矩和两髋力矩间转换。
 * 关键边界：几何有效与力映射有效分开；奇异/非有限输入拒绝映射；实体极性和零点在驱动/配置层处理。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "leg_solver.h"
#include <math.h>
#include <string.h>
#include "../../Drivers/CMSIS/DSP/Include/arm_math.h"   /* 相对路径: 不依赖包含路径 */

#define LEG_EPS         1.0e-6f
#define LEG_MIN_LENGTH  1.0e-3f

/* 公共几何估计只发布数值，不决定LQR/RL的控制模式或触地状态。 */
uint8_t Leg_Height_Estimate(const leg_state_t *left, const leg_state_t *right, float *height)
{
    *height = 0.0f;
    if (!left->output.valid || !right->output.valid) { return 0u; }
    *height = 0.5f * (left->output.virtual_leg_length * cosf(left->output.virtual_leg_angle) + right->output.virtual_leg_length * cosf(right->output.virtual_leg_angle));
    return (uint8_t)isfinite(*height);
}

/* 杆长/零偏直接使用初始化配置，只检查本次反馈；几何奇异性在求解时判断。 */
static uint8_t Leg_Input_Valid(const leg_state_t *leg)
{
    const leg_input_t *in = &leg->input;
    if (!isfinite(in->hip_f) || !isfinite(in->hip_b) || !isfinite(in->d_hip_f) || !isfinite(in->d_hip_b))
    {
        return 0u;
    }
    return 1u;
}



static uint8_t Leg_Solve_Geometry_z(leg_state_t *leg, leg_solver_cache_t *cache)
{
    float A[2],B[2],M[2];
    float AB[2],MP[2],OP[2],n_AB[2],n_MP[2];
    float dist_AB,dist_MP;
    
    cache->lu     = leg->config.lu;
    cache->lg     = leg->config.lg;
    cache->qf     = leg->input.hip_f;
    cache->qb     = leg->input.hip_b;
    cache->vf     = leg->input.d_hip_f;
    cache->vb     = leg->input.d_hip_b;
    //得到辅助点坐标
    A[0] = cache->lu * arm_cos_f32(cache->qf);
    A[1] = cache->lu * arm_sin_f32(cache->qf);
    B[0] = cache->lu * arm_cos_f32(cache->qb);
    B[1] = cache->lu * arm_sin_f32(cache->qb);
    M[0] = (A[0] + B[0])/2.0f;
    M[1] = (A[1] + B[1])/2.0f;
    //求ab
    AB[0] = B[0] - A[0];
    AB[1] = B[1] - A[1];
    //求ab距离
    if (arm_sqrt_f32(AB[0] * AB[0] + AB[1] * AB[1], &dist_AB) != ARM_MATH_SUCCESS|| dist_AB <= LEG_EPS)
    {
        return 0u; /* 圆心重合 */
    }
    //求mp距离
    if (arm_sqrt_f32(cache->lg * cache->lg - (dist_AB * dist_AB) / 4.0f, &dist_MP)!= ARM_MATH_SUCCESS)
    {
        return 0u; /* 无实数交点 */
    }
    //求单位向量
    n_AB[0] = AB[0]/dist_AB;
    n_AB[1] = AB[1]/dist_AB;
    n_MP[0] = -n_AB[1];
    n_MP[1] = n_AB[0];
    //求mp
    MP[0] = dist_MP * n_MP[0];
    MP[1] = dist_MP * n_MP[1];
    //求op距离, 腿长
    OP[0] = MP[0] + M[0];
    OP[1] = MP[1] + M[1];
    if (arm_sqrt_f32(OP[0] * OP[0] + OP[1] * OP[1], &leg->output.virtual_leg_length)!= ARM_MATH_SUCCESS || leg->output.virtual_leg_length < LEG_MIN_LENGTH)
    {
        return 0u;
    }
    //求腿角
    cache->virtual_leg_angle_abs  = atan2f(OP[1], OP[0]);
    //原本是后点为零点，像逆时针是正的，现在取负，并且加上偏置就变成竖直向下90°是一个零点然后向顺时针是正方向
    leg->output.virtual_leg_angle =Angle_Wrap_180(LEG_HALF_PI - cache->virtual_leg_angle_abs+ leg->config.offset_phi0);
    //前下杆角度
    cache->phi_a = atan2f(OP[1] - A[1],OP[0] - A[0]);
    //后下杆角度
    cache->phi_b = atan2f(OP[1] - B[1], OP[0] - B[0]);
    //前上杆，大腿角
    leg->output.thigh_angle = Angle_Wrap_180(cache->qf);
    //虚拟小腿角
    leg->output.virtual_shank_angle = Angle_Wrap_180(cache->phi_a - cache->qf - LEG_HALF_PI);
    return 1u;
}


static uint8_t Leg_Solve_Velocity_z(leg_state_t *leg, const leg_solver_cache_t *cache)
{
    //下杆的参数
    float ca = arm_cos_f32(cache->phi_a), sa = arm_sin_f32(cache->phi_a), cb = arm_cos_f32(cache->phi_b), sb = arm_sin_f32(cache->phi_b), cl = arm_cos_f32(cache->virtual_leg_angle_abs), sl = arm_sin_f32(cache->virtual_leg_angle_abs);
    float length = leg->output.virtual_leg_length;
    float alpha = ca * sb - sa * cb;//公共分母
    float ka,kb; /* 投影系数 */
    /* 分母无效或接近零时，无法可靠求解速度。 */
    if (!isfinite(alpha) || fabsf(alpha) < LEG_EPS)
    {
        return 0u;
    }
    //角速度前的因数
    ka = cache->lu * arm_sin_f32(cache->phi_a - cache->qf);
    kb = cache->lu * arm_sin_f32(cache->phi_b - cache->qb);
    //实现p点速度转op方向的因数雅可比
    leg->output.point_jac[0][0] =  ka*sb/alpha;
    leg->output.point_jac[0][1] = -kb*sa/alpha;
    leg->output.point_jac[1][0] = -ka*cb/alpha;
    leg->output.point_jac[1][1] =  kb*ca/alpha;
    //实现端点雅可比转摆角于腿长腿摆角雅可比，做投影
    for (uint8_t j = 0u; j < 2u; j++)
    {
        /* 腿长雅可比 */
        leg->output.leg_jac[0][j] = cl * leg->output.point_jac[0][j] + sl * leg->output.point_jac[1][j];
        /* 摆角雅可比 */
        leg->output.leg_jac[1][j] = (sl * leg->output.point_jac[0][j] - cl * leg->output.point_jac[1][j]) / length;
    }
        leg->output.vshank_jac[0] = kb / (cache->lg * alpha);
        leg->output.vshank_jac[1] = cache->lu * arm_sin_f32(cache->qf - cache->phi_b) / (cache->lg * alpha) - 1.0f;

        leg->output.d_virtual_leg_length  = leg->output.leg_jac[0][0] * cache->vf + leg->output.leg_jac[0][1] * cache->vb;
        leg->output.d_virtual_leg_angle   = leg->output.leg_jac[1][0] * cache->vf + leg->output.leg_jac[1][1] * cache->vb;
        leg->output.d_virtual_shank_angle = leg->output.vshank_jac[0] * cache->vb + leg->output.vshank_jac[1] * cache->vf;

        return 1u;
}
#if 0
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

    cache->lu     = leg->config.lu;
    cache->lg     = leg->config.lg;
    cache->qf     = leg->input.hip_f;
    cache->qb     = leg->input.hip_b;
    cache->vf     = leg->input.d_hip_f;
    cache->vb     = leg->input.d_hip_b;

    /* A = 前杆端点, B = 后杆端点 */
    x_a = cache->lu * arm_cos_f32(cache->qf);
    y_a = cache->lu * arm_sin_f32(cache->qf);
    x_b = cache->lu * arm_cos_f32(cache->qb);
    y_b = cache->lu * arm_sin_f32(cache->qb);

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
    arm_sqrt_f32(disc, &root);
    cache->phi_a = 2.0f * atan2f(lg_dy2 + root, lg_dx2 + ab_sq);
    x_p          = x_a + cache->lg * arm_cos_f32(cache->phi_a);
    y_p          = y_a + cache->lg * arm_sin_f32(cache->phi_a);
    cache->phi_b = atan2f(y_p - y_b, x_p - x_b);

    /* 虚拟腿长 = |OP| */
    arm_sqrt_f32(x_p * x_p + y_p * y_p, &leg->output.virtual_leg_length);
    if (leg->output.virtual_leg_length < LEG_MIN_LENGTH)
    {
        return 0u;
    }

    /* 虚拟腿摆角 (相对竖直方向) */
    cache->virtual_leg_angle_abs  = atan2f(y_p, x_p);
    leg->output.virtual_leg_angle = Angle_Wrap_180(LEG_HALF_PI - cache->virtual_leg_angle_abs + leg->config.offset_phi0);

    /* 大腿角 = qf (前髋上连杆, 去镜像后与 hip_f 一致) */
    leg->output.thigh_angle = Angle_Wrap_180(cache->qf);

    /* 虚拟小腿角 = φ_a - qf - π/2 (小腿相对大腿) */
    vs_raw = Angle_Wrap_180(cache->phi_a - cache->qf - LEG_HALF_PI);
    leg->output.virtual_shank_angle = Angle_Wrap_180(vs_raw);
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

    sin_ab = arm_sin_f32(cache->phi_a - cache->phi_b);
    if (fabsf(sin_ab) < LEG_EPS)
    {
        return 0u;
    }
    sin_fa = arm_sin_f32(cache->qf - cache->phi_a);
    sin_fb = arm_sin_f32(cache->qf - cache->phi_b);
    sin_bb = arm_sin_f32(cache->qb - cache->phi_b);
    sin_0a = arm_sin_f32(cache->virtual_leg_angle_abs - cache->phi_a);
    sin_0b = arm_sin_f32(cache->virtual_leg_angle_abs - cache->phi_b);
    cos_0a = arm_cos_f32(cache->virtual_leg_angle_abs - cache->phi_a);
    cos_0b = arm_cos_f32(cache->virtual_leg_angle_abs - cache->phi_b);

    /* P点位置雅可比 */
    leg->output.point_jac[0][0] =  cache->lu * sin_fa * arm_sin_f32(cache->phi_b) / sin_ab;
    leg->output.point_jac[0][1] = -cache->lu * sin_bb * arm_sin_f32(cache->phi_a) / sin_ab;
    leg->output.point_jac[1][0] = -cache->lu * sin_fa * arm_cos_f32(cache->phi_b) / sin_ab;
    leg->output.point_jac[1][1] =  cache->lu * sin_bb * arm_cos_f32(cache->phi_a) / sin_ab;

    /* 虚拟腿长/摆角雅可比 */
    leg->output.leg_jac[0][0] = -cache->lu * sin_0b * sin_fa / sin_ab;
    leg->output.leg_jac[0][1] =  cache->lu * sin_0a * sin_bb / sin_ab;
    leg->output.leg_jac[1][0] =  cache->lu * cos_0b * sin_fa / (leg->output.virtual_leg_length * sin_ab);
    leg->output.leg_jac[1][1] = -cache->lu * cos_0a * sin_bb / (leg->output.virtual_leg_length * sin_ab);
    leg->output.d_virtual_leg_length = leg->output.leg_jac[0][0] * cache->vf + leg->output.leg_jac[0][1] * cache->vb;
    leg->output.d_virtual_leg_angle  = leg->output.leg_jac[1][0] * cache->vf + leg->output.leg_jac[1][1] * cache->vb;

    /* 虚拟小腿雅可比: vshank_jac[0]=后髋, [1]=前髋 */
    jac_a   = cache->lu * sin_bb / (cache->lg * sin_ab);
    jac_b   = -cache->lu * sin_fb / (cache->lg * sin_ab) - 1.0f;
    d_vs    = jac_a * cache->vb + jac_b * cache->vf;
    leg->output.vshank_jac[0] = jac_a;
    leg->output.vshank_jac[1] = jac_b;
    leg->output.d_virtual_shank_angle = d_vs;
    return 1u;
}
#endif
/* 力矩映射: leg_jac 转置 → force_map */
static void Leg_Solve_Force_Map(leg_state_t *leg)
{
    leg->output.force_map[0][0] = leg->output.leg_jac[0][0];
    leg->output.force_map[0][1] = leg->output.leg_jac[1][0];
    leg->output.force_map[1][0] = leg->output.leg_jac[0][1];
    leg->output.force_map[1][1] = leg->output.leg_jac[1][1];
    leg->output.force_det = leg->output.force_map[0][0] * leg->output.force_map[1][1] - leg->output.force_map[0][1] * leg->output.force_map[1][0];
    leg->output.force_valid = (uint8_t)(isfinite(leg->output.force_det) && fabsf(leg->output.force_det) >= LEG_EPS);
}

/* 虚拟力/力矩 → 电机力矩 */
/*
 * 使用当前腿雅可比，把虚拟腿轴向力F和摆矩Tp映射成前/后髋力矩。
 * 输出顺序属于同一条腿前后髋；电机发送极性不在这里处理。
 */
uint8_t Leg_Force_Map_Forward(const leg_state_t *leg, float force,
                              float torque, float output[2])
{
    if (!leg->output.force_valid)
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

/* 反馈力矩反解 */
/*
 * 由同腿前/后髋反馈力矩反解轴向力和摆矩，供支持力观测使用。
 * 奇异/非有限输入在入口拒绝（奇异由 force_valid 门控），非有限结果由末次输出检查兜住；
 * 这是电机贡献，实体弹簧与自重由观测层按模型另行计入。
 */
uint8_t Leg_Force_Map_Inverse(const leg_state_t *leg, const float input[2],
                              float *force, float *torque)
{
    float a;
    float b;
    float c;
    float d;
    float det;
    float f;
    float t;

    *force = 0.0f;
    *torque = 0.0f;
    if (!leg->output.force_valid)
    {
        return 0u;
    }
    a = leg->output.force_map[0][0];
    b = leg->output.force_map[0][1];
    c = leg->output.force_map[1][0];
    d = leg->output.force_map[1][1];
    det = a * d - b * c;
    f = (d * input[0] - b * input[1]) / det;
    t = (a * input[1] - c * input[0]) / det;
    if (!isfinite(f) || !isfinite(t))
    {
        return 0u;
    }
    *force = f;
    *torque = t;
    return 1u;
}

/* 初始化 */
void Leg_Init(leg_state_t *leg)
{
    memset(leg, 0, sizeof(*leg));
}

/* 三层求解: 几何 → 速度 → 力矩映射 */
/*
 * 总入口依次计算几何、速度，统一验证反馈和雅可比，再生成力映射有效性。
 * 几何有效不保证力映射不奇异；控制力输出必须同时检查映射有效。
 */
uint8_t Leg_Solve(leg_state_t *leg)
{
    leg_solver_cache_t cache;

    memset(&leg->output, 0, sizeof(leg->output));
    if (!Leg_Input_Valid(leg))
    {
        return 0u;
    }
    /* 使用中点法_z解算；原半角实现已由#if 0禁用保留 */
    if (!Leg_Solve_Geometry_z(leg, &cache))
    {
        return 0u;
    }
    if (!Leg_Solve_Velocity_z(leg, &cache))
    {
        return 0u;
    }
    /* 发布前校验 */
    if (!isfinite(leg->output.thigh_angle) || !isfinite(leg->output.virtual_leg_length) || !isfinite(leg->output.virtual_leg_angle) || !isfinite(leg->output.virtual_shank_angle) || !isfinite(leg->output.d_virtual_leg_length) || !isfinite(leg->output.d_virtual_leg_angle) || !isfinite(leg->output.d_virtual_shank_angle))
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!isfinite(leg->output.vshank_jac[i]))
        {
            return 0u;
        }
        for (uint8_t j = 0u; j < 2u; j++)
        {
            if (!isfinite(leg->output.point_jac[i][j]) || !isfinite(leg->output.leg_jac[i][j]))
            {
                return 0u;
            }
        }
    }
    Leg_Solve_Force_Map(leg);
    leg->output.valid = 1u;
    return 1u;
}
