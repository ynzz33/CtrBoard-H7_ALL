/*
 * 按实测虚拟腿长求实体气弹簧轴向力，并映射成四髋抵消力矩。
 * 实体力供AIR估力；delta_dm供命令补偿，两者用途不同，计算模型不代表启用抵消。
 */

#ifndef GAS_SPRING_H
#define GAS_SPRING_H

#include <stdint.h>
#include "leg_solver.h"

#ifndef GAS_SPRING_COMP_ENABLE
#define GAS_SPRING_COMP_ENABLE 1
#endif

float Gas_Spring_Force(float L0);

typedef struct {
    float force[2]; /* 实体伸腿力 */
    float delta_dm[4]; /* 电机抵消量 */
    uint8_t valid;
} gas_spring_result_t;

void Gas_Spring_Prepare(const leg_state_t *left, const leg_state_t *right, gas_spring_result_t *result);
uint8_t Gas_Spring_Apply(const gas_spring_result_t *spring, const float base_dm[4], float unclamped_dm[4]);

#endif
