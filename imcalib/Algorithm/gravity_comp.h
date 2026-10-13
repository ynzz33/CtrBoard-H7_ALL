/*
 * 文件职责：整条腿自重模型查询与摆矩补偿（gravity_comp.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：机器自重表、实测腿长和世界腿角。
 * 输出/去向：插值质心半径/偏角，计算重力摆矩并经五连杆映射到电机。
 * 关键边界：只负责重力，不叠加弹簧或最终限幅；超出表域夹到边界并记录clamped。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef GRAVITY_COMP_H
#define GRAVITY_COMP_H

#include "leg_solver.h"
#include "machine_config.h"

typedef struct {
    float world_angle;
    float offset;
    float radius;
    float torque;
    uint8_t clamped;
    uint8_t valid;
} gravity_leg_result_t;

typedef struct {
    gravity_leg_result_t leg[2];
    float delta_dm[4];
    uint8_t valid;
} gravity_comp_result_t;

uint8_t Gravity_Comp_Compute(const machine_gravity_cfg_t *cfg,
    float length, float world_angle, gravity_leg_result_t *result);
void Gravity_Comp_Prepare(const leg_state_t *left, const leg_state_t *right, const float world_angle[2], const gravity_comp_result_t *reference, gravity_comp_result_t *result);

#endif
