/*
 * 按enabled位标志计算气弹簧和本拍选定姿态引用的一份重力缓存；此时尚未叠加电机输出。
 * Apply_Mode按输出开关将选中的缓存加到base_dm，再对四髋逐个限矩。
 */

#include "leg_compensation.h"
#include "gas_spring.h"

#include <string.h>
#include <math.h>

/* 清缓存后计算实体弹簧，再选择一种角度输入并统一计算一份重力输出。
 * 缓存供AIR估力和命令叠加分别使用；计算spring不等于启用气弹簧抵消输出。 */
void Leg_Compensation_Prepare(leg_compensation_values_t *values, const leg_state_t *left, const leg_state_t *right, const imu_state_t *imu, uint8_t enabled)
{
    const leg_state_t *leg[2] = {left, right};
    float angle[2];

    memset(values, 0, sizeof(*values));
    /* 求实体弹簧伸腿力及抵消髋力矩，AIR取前者，输出补偿按开关取后者。 */
    if (enabled & LEG_COMP_SPRING)
    {
        Gas_Spring_Prepare(left, right, &values->spring);
    }
    /* 输出路径已选择一种姿态引用；两种标志不允许同时提出。 */
    if ((enabled & (LEG_COMP_GRAVITY | LEG_COMP_GRAVITY_WORLD)) ==
        (LEG_COMP_GRAVITY | LEG_COMP_GRAVITY_WORLD))
    {
        return; /* 非法请求不生成有效重力结果，消费端会拒绝出力。 */
    }
    if (enabled & LEG_COMP_GRAVITY_WORLD)
    {
        for (uint8_t side = 0u; side < 2u; side++)
        {
            angle[side] = remainderf(leg[side]->output.virtual_leg_angle - imu->pitch_world, LEG_2PI);
        }
    }
    else if (enabled & LEG_COMP_GRAVITY)
    {
        for (uint8_t side = 0u; side < 2u; side++)
        {
            angle[side] = leg[side]->output.virtual_leg_angle - imu->euler_rad[ATTITUDE_ROLL];
        }
    }
    else
    {
        return; /* 本拍只需弹簧模型或没有补偿请求。 */
    }
    Gravity_Comp_Prepare(left, right, angle, NULL, &values->gravity);
}

/* 按默认允许弹簧的路径调用Apply_Mode；是否真正叠加仍受总开关和模型配置限制。 */
uint8_t Leg_Compensation_Apply(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, float output_dm[4], compensation_debug_t *debug)
{
    return Leg_Compensation_Apply_Mode(spring, gravity, base_dm, limit, 1u, output_dm, debug);
}

/* 从base_dm四髋基础力矩N·m开始，按spring_active叠加弹簧，再叠加选定重力并逐髋限矩。
 * 缺所需有效模型则返回0；debug记录增量、限幅前命令及是否饱和。 */
uint8_t Leg_Compensation_Apply_Mode(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, uint8_t spring_enable, float output_dm[4], compensation_debug_t *debug)
{
    float raw[4];
    /* 三项同时满足才向电机命令加入弹簧抵消；模型已计算并不代表这里开启。 */
    uint8_t spring_active = (uint8_t)(spring_enable && GAS_SPRING_COMP_ENABLE && machine->compensation.spring != NULL);

    memset(debug, 0, sizeof(*debug));
    if ((spring_active && !spring->valid) || (machine->compensation.gravity != NULL && !gravity->valid))
    {
        return 0u;
    }
    /* 顺序：基础髋力矩 + 可选弹簧抵消 + 选定重力补偿，单位均为N·m。 */
    for (uint8_t i = 0u; i < 4u; i++)
    {
        raw[i] = base_dm[i];
        if (spring_active)
        {
            raw[i] += spring->delta_dm[i];
            debug->spring_dm[i] = raw[i] - base_dm[i];
        }
        if (machine->compensation.gravity != NULL)
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
    memcpy(debug->unclamped_dm, raw, sizeof(raw));
    for (uint8_t i = 0u; i < 4u; i++)
    {
        output_dm[i] = clampf(raw[i], -limit, limit);
        debug->saturated |= (uint8_t)(output_dm[i] != raw[i]);
    }
    return 1u;
}
