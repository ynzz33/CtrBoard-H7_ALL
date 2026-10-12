/*
 * 文件职责：弹簧和完整腿自重补偿的组合入口（leg_compensation.c）。
 * 输入/来源：已映射的四髋基础力矩、双腿几何、世界腿角和弹簧接入选择。
 * 输出/去向：先加弹簧抵消，再加重力摆矩，最后逐电机限幅并发布补偿诊断。
 * 关键边界：spring_enable仅决定命令是否抵消实体弹簧，不改变检测使用的实体弹簧模型；补偿只加一次。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "leg_compensation.h"
#include "gas_spring.h"

#include <string.h>

/* 补偿后限幅 */
uint8_t Leg_Compensation_Apply(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float limit,
    float output_dm[4], compensation_debug_t *debug)
{
    return Leg_Compensation_Apply_Mode(left, right, world_angle, base_dm, limit,
        1u, output_dm, debug);
}

/* 按路径补偿 */
/*
 * 按spring_enable决定是否抵消实体弹簧，再始终调用完整自重补偿，最后逐电机限幅。
 * debug保存各项增量和限幅前原始值；饱和后映射回轴向力可能不再严格等于原请求。
 */
uint8_t Leg_Compensation_Apply_Mode(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float limit,
    uint8_t spring_enable, float output_dm[4], compensation_debug_t *debug)
{
    float raw[4];
    uint8_t i;

    memset(debug, 0, sizeof(*debug));
    if (base_dm == NULL)
    {
        return 0u;
    }
    if (spring_enable)
    {
        if (!Gas_Spring_Apply(left, right, base_dm, raw))
        {
            return 0u;
        }
        for (i = 0u; i < 4u; i++)
        {
            debug->spring_dm[i] = raw[i] - base_dm[i];
        }
    }
    else
    {
        memcpy(raw, base_dm, sizeof(raw));
    }
    if (!Gravity_Comp_Apply(left, right, world_angle, raw, raw, &debug->gravity))
    {
        memset(debug, 0, sizeof(*debug));
        return 0u;
    }
    memcpy(debug->raw_dm, raw, sizeof(raw));
    for (i = 0u; i < 4u; i++)
    {
        output_dm[i] = clampf(raw[i], -limit, limit);
        debug->saturated |= (uint8_t)(output_dm[i] != raw[i]);
    }
    return 1u;
}
