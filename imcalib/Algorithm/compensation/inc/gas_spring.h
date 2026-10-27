/*
 * 文件职责：气弹簧伸腿力模型与电机侧抵消（gas_spring.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：准备阶段读取实测腿长、机器表与力映射；输出阶段读取本拍缓存。
 * 输出/去向：声明实体弹簧力、电机抵消量与准备/叠加接口。
 * 关键边界：Gas_Spring_Apply只叠加准备值；实体力与输出抵消量含义不同。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
