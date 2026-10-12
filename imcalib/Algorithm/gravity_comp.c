/*
 * 文件职责：整条腿自重模型查询与摆矩补偿（gravity_comp.c）。
 * 输入/来源：机器自重表、实测腿长和世界腿角。
 * 输出/去向：插值质心半径/偏角，计算重力摆矩并经五连杆映射到电机。
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
    uint8_t i;

    if (cfg == NULL || cfg->nodes == NULL || cfg->count < 2u
        || !isfinite(length))
    {
        return 0u;
    }
    nodes = cfg->nodes;
    for (i = 0u; i < cfg->count; i++)
    {
        if (!isfinite(nodes[i].length) || !isfinite(nodes[i].offset)
            || !isfinite(nodes[i].radius) || nodes[i].radius < 0.0f
            || (i > 0u && nodes[i].length <= nodes[i - 1u].length))
        {
            return 0u;
        }
    }
    *clamped = (uint8_t)(length < nodes[0].length
        || length > nodes[cfg->count - 1u].length);
    length = clampf(length, nodes[0].length, nodes[cfg->count - 1u].length);
    for (i = 0u; i < cfg->count - 1u; i++)
    {
        if (length <= nodes[i + 1u].length)
        {
            fraction = (length - nodes[i].length)
                / (nodes[i + 1u].length - nodes[i].length);
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

    if (result == NULL)
    {
        return 0u;
    }
    memset(result, 0, sizeof(*result));
    memset(&calculated, 0, sizeof(calculated));
    if (cfg == NULL || !isfinite(cfg->mass) || cfg->mass <= 0.0f
        || !isfinite(cfg->gravity) || cfg->gravity <= 0.0f
        || !isfinite(world_angle)
        || !Gravity_Comp_Lookup(cfg, length, &calculated.offset,
            &calculated.radius, &calculated.clamped))
    {
        return 0u;
    }
    calculated.world_angle = remainderf(world_angle, LEG_2PI);
    calculated.torque = cfg->mass * cfg->gravity * calculated.radius
        * sinf(calculated.world_angle - calculated.offset);
    if (!isfinite(calculated.torque))
    {
        return 0u;
    }
    calculated.valid = 1u;
    *result = calculated;
    return 1u;
}

/* 双腿全部成功才发布 */
uint8_t Gravity_Comp_Apply(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float raw_dm[4],
    gravity_comp_result_t *result)
{
    const leg_state_t *leg[2];
    gravity_comp_result_t calculated;
    float sum[4];
    uint8_t i;
    uint8_t side;

    if (result != NULL)
    {
        memset(result, 0, sizeof(*result));
    }
    if (raw_dm == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < 4u; i++)
    {
        sum[i] = base_dm != NULL ? base_dm[i] : 0.0f;
    }
    memset(raw_dm, 0, sizeof(sum));
    if (base_dm == NULL || result == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < 4u; i++)
    {
        if (!isfinite(sum[i]))
        {
            return 0u;
        }
    }
    memset(&calculated, 0, sizeof(calculated));
    if (machine->gravity != NULL)
    {
        leg[0] = left;
        leg[1] = right;
        if (world_angle == NULL)
        {
            return 0u;
        }
        for (side = 0u; side < 2u; side++)
        {
            if (leg[side] == NULL || !leg[side]->output.valid || !leg[side]->output.force_valid
                || !Gravity_Comp_Compute(machine->gravity,
                    leg[side]->output.virtual_leg_length, world_angle[side], &calculated.leg[side])
                || !Leg_Force_Map_Forward(leg[side], 0.0f, calculated.leg[side].torque,
                    &calculated.delta_dm[side * 2u]))
            {
                return 0u;
            }
        }
        for (i = 0u; i < 4u; i++)
        {
            sum[i] += calculated.delta_dm[i];
            if (!isfinite(sum[i]))
            {
                return 0u;
            }
        }
        calculated.valid = 1u;
    }
    memcpy(raw_dm, sum, sizeof(sum));
    *result = calculated;
    return 1u;
}
