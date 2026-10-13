/*
 * 文件职责：气弹簧伸腿力模型与电机侧抵消（gas_spring.c）。
 * 输入/来源：实测虚拟腿长、机器表启用项和腿部力映射。
 * 输出/去向：估计实体弹簧伸腿力；将抵消力转换为四髋补偿力矩。
 * 关键边界：Gas_Spring_Force是检测用物理模型值；Gas_Spring_Apply是命令补偿，两者含义不同。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "gas_spring.h"
#include "machine_config.h"
#include "../../Drivers/CMSIS/DSP/Include/arm_math.h"
#include <math.h>
#include <string.h>

/* Leg3 同机模型，输出伸腿力 N */

static float Spring_Force(const machine_spring_cfg_t *model, float L0)
{
    float sqrt_tmp1;
    float theta;

    L0 = fminf(fmaxf(L0, model->len_min), model->len_max);
    theta = acosf((model->bar_sum_sq - L0 * L0) / model->bar_prod2);
    arm_sqrt_f32(fmaxf(model->anchor_sum_sq - model->anchor_prod2 * arm_cos_f32(theta - model->phase), 0.0F), &sqrt_tmp1);
    return model->force_n * (model->numerator_scale * arm_sin_f32(theta - model->phase) * L0 / fmaxf(model->denominator_scale * arm_sin_f32(theta) * sqrt_tmp1, 1.0E-6F));
}

/* 实体弹簧力 */
float Gas_Spring_Force(float L0)
{
    return machine->spring != NULL ? Spring_Force(machine->spring, L0) : 0.0f;
}

/* 本拍实体力与抵消量 */
void Gas_Spring_Prepare(const leg_state_t *left, const leg_state_t *right, gas_spring_result_t *result)
{
    const leg_state_t *leg[2] = {left, right};

    memset(result, 0, sizeof(*result));
    if (machine->spring != NULL)
    {
        for (uint8_t side = 0u; side < 2u; side++)
        {
            result->force[side] = Gas_Spring_Force(leg[side]->output.virtual_leg_length);
            if (!isfinite(result->force[side]))
            {
                return;
            }
#if GAS_SPRING_COMP_ENABLE
            if (!Leg_Force_Map_Forward(leg[side], -result->force[side], 0.0f, &result->delta_dm[side * 2u]) || !isfinite(result->delta_dm[side * 2u]) || !isfinite(result->delta_dm[side * 2u + 1u]))
            {
                return;
            }
#endif
        }
    }
    result->valid = 1u;
}

/* 叠加本拍抵消量 */
uint8_t Gas_Spring_Apply(const gas_spring_result_t *spring, const float base_dm[4], float raw_dm[4])
{
    float result[4];
    uint8_t active = (uint8_t)(GAS_SPRING_COMP_ENABLE && machine->spring != NULL);

    memcpy(result, base_dm, sizeof(result));
    memset(raw_dm, 0, sizeof(result));
#if GAS_SPRING_COMP_ENABLE
    if (active && !spring->valid)
    {
        return 0u;
    }
#else
    (void)spring;
#endif
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (active)
        {
            result[i] += spring->delta_dm[i];
        }
        if (!isfinite(result[i]))
        {
            return 0u;
        }
    }
    memcpy(raw_dm, result, sizeof(result));
    return 1u;
}
