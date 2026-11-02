/*
 * 按腿长插值腿部质心半径和偏角，用相对重力的腿角求自重摆矩，再映射为两髋力矩。
 * 超出表域取边界并记录clamped；缓存复用只减少查表，不叠加弹簧或发送命令。
 */

#include "gravity_comp.h"

#include <math.h>
#include <string.h>

/* 按length(m)夹到表域并在线性相邻节点间插值，输出质心偏角rad和半径m。 */
static uint8_t Gravity_Comp_Lookup(const machine_gravity_cfg_t *cfg,
    float length, float *offset, float *radius, uint8_t *clamped)
{
    const machine_gravity_node_t *nodes;
    float fraction;

    nodes = cfg->nodes;
    *clamped = (uint8_t)(length < nodes[0].length || length > nodes[cfg->count - 1u].length);
    length = clampf(length, nodes[0].length, nodes[cfg->count - 1u].length);
    for (uint8_t i = 0u; i < cfg->count - 1u; i++)
    {
        if (length <= nodes[i + 1u].length)
        {
            fraction = (length - nodes[i].length) / (nodes[i + 1u].length - nodes[i].length);
            *offset = nodes[i].offset + fraction * (nodes[i + 1u].offset - nodes[i].offset);
            *radius = nodes[i].radius + fraction * (nodes[i + 1u].radius - nodes[i].radius);
            return (uint8_t)(isfinite(*offset) && isfinite(*radius));
        }
    }
    return 0u;
}

/* 查腿长对应质心，再用mass*g*radius*sin(world_angle-offset)求自重摆矩N·m。 */
uint8_t Gravity_Comp_Compute(const machine_gravity_cfg_t *cfg,
    float length, float world_angle, gravity_leg_result_t *result)
{
    gravity_leg_result_t calculated;

    memset(result, 0, sizeof(*result));
    memset(&calculated, 0, sizeof(calculated));
    if (!isfinite(world_angle) || !Gravity_Comp_Lookup(cfg, length, &calculated.offset, &calculated.radius, &calculated.clamped))
    {
        return 0u;
    }
    calculated.world_angle = remainderf(world_angle, LEG_2PI);
    calculated.torque = cfg->mass * cfg->gravity * calculated.radius * sinf(calculated.world_angle - calculated.offset);
    if (!isfinite(calculated.torque))
    {
        return 0u;
    }
    calculated.valid = 1u;
    *result = calculated;
    return 1u;
}

/* 分别计算两腿自重摆矩并映射到四髋；可复用cached_result的质心查表结果。
 * 相同角度直接复用力矩；全部成功才置result.valid。 */
void Gravity_Comp_Prepare(const leg_state_t *left, const leg_state_t *right, const float world_angle[2], const gravity_comp_result_t *cached_result, gravity_comp_result_t *result)
{
    const leg_state_t *leg[2] = {left, right};
    float angle;

    memset(result, 0, sizeof(*result));
    if (machine->compensation.gravity == NULL)
    {
        return;
    }
    for (uint8_t side = 0u; side < 2u; side++)
    {
        if (cached_result != NULL && cached_result->valid)
        {
            result->leg[side] = cached_result->leg[side];
            angle = remainderf(world_angle[side], LEG_2PI);
            if (memcmp(&angle, &cached_result->leg[side].world_angle, sizeof(angle)) == 0)
            {
                memcpy(&result->delta_dm[side * 2u], &cached_result->delta_dm[side * 2u], 2u * sizeof(float));
                continue;
            }
            result->leg[side].world_angle = angle;
            result->leg[side].torque = machine->compensation.gravity->mass * machine->compensation.gravity->gravity * result->leg[side].radius * sinf(angle - result->leg[side].offset);
            if (!isfinite(result->leg[side].torque))
            {
                return;
            }
        }
        else if (!Gravity_Comp_Compute(machine->compensation.gravity, leg[side]->output.virtual_leg_length, world_angle[side], &result->leg[side]))
        {
            return;
        }
        if (!Leg_Force_Map_Forward(leg[side], 0.0f, result->leg[side].torque, &result->delta_dm[side * 2u]))
        {
            return;
        }
    }
    result->valid = 1u;
}
