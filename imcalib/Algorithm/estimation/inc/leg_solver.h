/*
 * 由前后髋角rad、角速度rad/s和杆长m求五连杆几何、虚拟腿速度及力映射矩阵。
 * Forward将轴向力N/摆矩N·m变成两髋力矩，Inverse反解；几何valid与force_valid分别检查。
 */

#ifndef LEG_SOLVER_H
#define LEG_SOLVER_H

#include <stdint.h>
#include "math_utils.h"

/* 数学常量 */
#define LEG_PI       3.14159265358979f
#define LEG_2PI      6.28318530717959f
#define LEG_HALF_PI  1.57079632679490f

/* 几何参数 */
typedef struct {
    float lu; /* 上杆长度m。 */
    float lg; /* 下杆长度m。 */
    float offset_phi0; /* 几何腿角零位偏置rad。 */
} leg_config_t;

/* 髋关节输入 */
typedef struct {
    float hip_f; /* 解算坐标中的前髋角rad。 */
    float hip_b; /* 解算坐标中的后髋角rad。 */
    float d_hip_f; /* 前髋角速度rad/s。 */
    float d_hip_b; /* 后髋角速度rad/s。 */
} leg_input_t;

/* 求解输出 */
typedef struct {
    float thigh_angle;              /* 大腿角 (电机+偏置, 不经解算) */
    float virtual_leg_length; /* 髋轴至轮轴虚拟腿长m。 */
    float virtual_leg_angle; /* 按当前解算零位定义的虚拟腿角rad。 */
    float virtual_shank_angle;      /* 虚拟小腿角 (小腿相对大腿) */
    float d_virtual_leg_length; /* 虚拟腿伸缩速度m/s。 */
    float d_virtual_leg_angle; /* 虚拟腿摆角速度rad/s。 */
    float d_virtual_shank_angle;    /* 虚拟小腿角速度 */
    float vshank_jac[2];            /* 虚拟小腿雅可比 [0]=b, [1]=a */
    float point_jac[2][2];          /* P点雅可比 */
    float leg_jac[2][2];            /* 腿雅可比 */
    float force_map[2][2];          /* 力矩映射 */
    float force_det; /* 两髋力矩映射矩阵行列式，用于检查奇异性。 */
    uint8_t force_valid; /* 映射可用 */
    uint8_t valid;       /* 数值有效 */
} leg_output_t;

/* 求解中间量 */
typedef struct {
    float qf;           /* 前髋角 */
    float qb;           /* 后髋角 */
    float vf;           /* 前髋速度 */
    float vb;           /* 后髋速度 */
    float lu; /* 上杆长度m。 */
    float lg; /* 下杆长度m。 */
    float phi_a;        /* 前下杆AP角 */
    float phi_b;        /* 后下杆BP角 */
    float virtual_leg_angle_abs; /* 几何计算得到的绝对腿角rad，不是fabs后的角度。 */
} leg_solver_cache_t;

/* 腿部状态 */
typedef struct {
    leg_config_t config;
    leg_input_t  input;
    leg_output_t output;
} leg_state_t;

void Leg_Init(leg_state_t *leg);
uint8_t Leg_Solve(leg_state_t *leg);
uint8_t Leg_Height_Estimate(const leg_state_t *left, const leg_state_t *right, float *height); /* 髋轴到轮轴的两腿投影均值m，不是绝对离地高度。 */
uint8_t Leg_Force_Map_Forward(const leg_state_t *leg, float force,
                              float torque, float output[2]);
uint8_t Leg_Force_Map_Inverse(const leg_state_t *leg, const float input[2],
                              float *force, float *torque);

#endif
