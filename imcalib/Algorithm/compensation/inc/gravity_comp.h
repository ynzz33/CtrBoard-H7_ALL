/*
 * 按腿长插值腿部质心半径和偏角，用相对重力的腿角求自重摆矩，再映射为两髋力矩。
 * 超出表域取边界并记录clamped；缓存复用只减少查表，不叠加弹簧或发送命令。
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
void Gravity_Comp_Prepare(const leg_state_t *left, const leg_state_t *right, const float world_angle[2], const gravity_comp_result_t *cached_result, gravity_comp_result_t *result);

#endif
