/*
 * 文件职责：弹簧和完整腿自重补偿的组合入口（leg_compensation.c）。
 * 输入/来源：准备阶段读取双腿和IMU；消费阶段读取缓存、基础力矩与接入选择。
 * 输出/去向：准备两种角度引用的模型值；保持弹簧、重力、限幅的叠加顺序。
 * 关键边界：spring_enable仅决定命令是否抵消实体弹簧，不改变检测使用的实体弹簧模型；补偿只加一次。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "leg_compensation.h"
#include "gas_spring.h"

#include <string.h>
#include <math.h>

/* 准备本拍物理量 */
void Leg_Compensation_Prepare(leg_compensation_values_t *values, const leg_state_t *left, const leg_state_t *right, const imu_state_t *imu, uint8_t enabled)
{
    const leg_state_t *leg[2] = {left, right};
    float angle[2];

    memset(values, 0, sizeof(*values));
    if (enabled & LEG_COMP_SPRING)
    {
        Gas_Spring_Prepare(left, right, &values->spring);
    }
    if (enabled & LEG_COMP_GRAVITY)
    {
        for (uint8_t side = 0u; side < 2u; side++)
        {
            angle[side] = leg[side]->output.virtual_leg_angle - imu->euler_rad[ATTITUDE_ROLL];
        }
        Gravity_Comp_Prepare(left, right, angle, NULL, &values->gravity);
    }
    if (enabled & LEG_COMP_GRAVITY_WORLD)
    {
        for (uint8_t side = 0u; side < 2u; side++)
        {
            angle[side] = remainderf(leg[side]->output.virtual_leg_angle - imu->pitch_world, LEG_2PI);
        }
        Gravity_Comp_Prepare(left, right, angle, &values->gravity, &values->gravity_world);
    }
}

/* 补偿后限幅 */
uint8_t Leg_Compensation_Apply(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, float output_dm[4], compensation_debug_t *debug)
{
    return Leg_Compensation_Apply_Mode(spring, gravity, base_dm, limit, 1u, output_dm, debug);
}

/* 按路径补偿 */
/*
 * 按spring_enable叠加本拍弹簧抵消值，再叠加对应重力缓存，最后逐电机限幅。
 * debug保存各项增量和限幅前原始值；饱和后映射回轴向力可能不再严格等于原请求。
 */
uint8_t Leg_Compensation_Apply_Mode(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, uint8_t spring_enable, float output_dm[4], compensation_debug_t *debug)
{
    float raw[4];
    uint8_t spring_active = (uint8_t)(spring_enable && GAS_SPRING_COMP_ENABLE && machine->spring != NULL);

    memset(debug, 0, sizeof(*debug));
    if ((spring_active && !spring->valid) || (machine->gravity != NULL && !gravity->valid))
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < 4u; i++)
    {
        raw[i] = base_dm[i];
        if (spring_active)
        {
            raw[i] += spring->delta_dm[i];
            debug->spring_dm[i] = raw[i] - base_dm[i];
        }
        if (machine->gravity != NULL)
        {
            raw[i] += gravity->delta_dm[i];
        }
        if (!isfinite(raw[i]))
        {
            memset(debug, 0, sizeof(*debug));
            return 0u;
        }
    }
    debug->gravity = *gravity;
    memcpy(debug->raw_dm, raw, sizeof(raw));
    for (uint8_t i = 0u; i < 4u; i++)
    {
        output_dm[i] = clampf(raw[i], -limit, limit);
        debug->saturated |= (uint8_t)(output_dm[i] != raw[i]);
    }
    return 1u;
}
