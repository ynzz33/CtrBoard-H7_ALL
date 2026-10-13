/*
 * 文件职责：整条腿自重模型查询与摆矩补偿（gravity_comp.c）。
 * 输入/来源：机器自重表、实测腿长和世界腿角。
 * 输出/去向：准备质心半径/偏角、重力摆矩和四髋增量，供消费端直接复用。
 * 关键边界：只负责重力，不叠加弹簧或最终限幅；超出表域夹到边界并记录clamped。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "gravity_comp.h"

#include <math.h>
#include <string.h>

/* 有效节点插值 */
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

/* 完整自重摆矩 */
/*
 * 由腿长查询/插值质心半径和偏角，按世界腿角计算自重摆矩。
 * 超表域取边界并标记clamped；结果是摆矩，不是额外支撑轴向力。
 */
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

/* 本拍查表与映射 */
/* 两种角度引用共用同腿查表结果；相同角度直接复用摆矩和映射值。 */
void Gravity_Comp_Prepare(const leg_state_t *left, const leg_state_t *right, const float world_angle[2], const gravity_comp_result_t *reference, gravity_comp_result_t *result)
{
    const leg_state_t *leg[2] = {left, right};
    float angle;

    memset(result, 0, sizeof(*result));
    if (machine->gravity == NULL)
    {
        return;
    }
    for (uint8_t side = 0u; side < 2u; side++)
    {
        if (reference != NULL && reference->valid)
        {
            result->leg[side] = reference->leg[side];
            angle = remainderf(world_angle[side], LEG_2PI);
            if (memcmp(&angle, &reference->leg[side].world_angle, sizeof(angle)) == 0)
            {
                memcpy(&result->delta_dm[side * 2u], &reference->delta_dm[side * 2u], 2u * sizeof(float));
                continue;
            }
            result->leg[side].world_angle = angle;
            result->leg[side].torque = machine->gravity->mass * machine->gravity->gravity * result->leg[side].radius * sinf(angle - result->leg[side].offset);
            if (!isfinite(result->leg[side].torque))
            {
                return;
            }
        }
        else if (!Gravity_Comp_Compute(machine->gravity, leg[side]->output.virtual_leg_length, world_angle[side], &result->leg[side]))
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
