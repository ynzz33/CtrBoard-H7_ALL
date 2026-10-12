/*
 * 文件职责：五连杆运动学、速度和力映射（leg_solver.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：机器表几何与左右腿前/后髋角度、角速度。
 * 输出/去向：求虚拟腿长/腿角、关节速度、雅可比，并在轴向力/摆矩和两髋力矩间转换。
 * 关键边界：几何有效与力映射有效分开；奇异/非有限输入拒绝映射；实体极性和零点在驱动/配置层处理。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef LEG_SOLVER_H
#define LEG_SOLVER_H

#include <stdint.h>

/* 数学常量 */
#define LEG_PI       3.14159265358979f
#define LEG_2PI      6.28318530717959f
#define LEG_HALF_PI  1.57079632679490f

/* 通用限幅 */
static inline float clampf(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* 几何参数 */
typedef struct {
    float lu;           /* 上杆长 */
    float lg;           /* 下杆长 */
    float offset_phi0;  /* 摆角零位偏置 */
    uint8_t configured;
} leg_config_t;

/* 髋关节输入 */
typedef struct {
    float hip_f;        /* 前髋角 */
    float hip_b;        /* 后髋角 */
    float d_hip_f;      /* 前髋速度 */
    float d_hip_b;      /* 后髋速度 */
} leg_input_t;

/* 求解输出 */
typedef struct {
    float thigh_angle;              /* 大腿角 (电机+偏置, 不经解算) */
    float virtual_leg_length;       /* 虚拟腿长 |OP| */
    float virtual_leg_angle;        /* 虚拟腿摆角 (相对竖直) */
    float virtual_shank_angle;      /* 虚拟小腿角 (小腿相对大腿) */
    float d_virtual_leg_length;     /* 虚拟腿长速度 */
    float d_virtual_leg_angle;      /* 虚拟腿摆角速度 */
    float d_virtual_shank_angle;    /* 虚拟小腿角速度 */
    float vshank_jac[2];            /* 虚拟小腿雅可比 [0]=b, [1]=a */
    float point_jac[2][2];          /* P点雅可比 */
    float leg_jac[2][2];            /* 腿雅可比 */
    float force_map[2][2];          /* 力矩映射 */
    float force_det;
    uint8_t force_valid;
    uint8_t valid;
} leg_output_t;

/* 求解中间量 */
typedef struct {
    float qf;           /* 前髋角 */
    float qb;           /* 后髋角 */
    float vf;           /* 前髋速度 */
    float vb;           /* 后髋速度 */
    float lu;           /* 上杆长 */
    float lg;           /* 下杆长 */
    float phi_a;        /* 前下杆AP角 */
    float phi_b;        /* 后下杆BP角 */
    float virtual_leg_angle_abs;    /* 虚拟腿摆角绝对值 */
} leg_solver_cache_t;

/* 腿部状态 */
typedef struct {
    leg_config_t config;
    leg_input_t  input;
    leg_output_t output;
} leg_state_t;

void Leg_Init(leg_state_t *leg);
uint8_t Leg_Solve(leg_state_t *leg);
uint8_t Leg_Force_Map_Forward(const leg_state_t *leg, float force,
                              float torque, float output[2]);
uint8_t Leg_Force_Map_Inverse(const leg_state_t *leg, const float input[2],
                              float *force, float *torque);

#endif
