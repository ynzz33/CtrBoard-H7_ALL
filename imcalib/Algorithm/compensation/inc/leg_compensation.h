/*
 * 文件职责：弹簧和完整腿自重补偿的组合入口（leg_compensation.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：准备阶段读取双腿和IMU；消费阶段读取缓存、基础力矩与接入选择。
 * 输出/去向：声明本拍准备值及按弹簧、重力、限幅顺序使用的接口。
 * 关键边界：spring_enable仅决定命令是否抵消实体弹簧，不改变检测使用的实体弹簧模型；补偿只加一次。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef LEG_COMPENSATION_H
#define LEG_COMPENSATION_H

#include "gravity_comp.h"
#include "gas_spring.h"
#include "imu_state.h"

enum {
    LEG_COMP_SPRING = 1u,
    LEG_COMP_GRAVITY = 2u,
    LEG_COMP_GRAVITY_WORLD = 4u,
};

/* 执行拍预计算值 */
typedef struct {
    gas_spring_result_t spring;
    gravity_comp_result_t gravity; /* 原姿态引用 */
    gravity_comp_result_t gravity_world; /* 补角引用 */
} leg_compensation_values_t;

typedef struct {
    gravity_comp_result_t gravity;
    float spring_dm[4];
    float unclamped_dm[4];
    uint8_t saturated;
    uint8_t bench;
} compensation_debug_t;

void Leg_Compensation_Prepare(leg_compensation_values_t *values, const leg_state_t *left, const leg_state_t *right, const imu_state_t *imu, uint8_t enabled);
uint8_t Leg_Compensation_Apply(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, float output_dm[4], compensation_debug_t *debug);
uint8_t Leg_Compensation_Apply_Mode(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, uint8_t spring_enable, float output_dm[4], compensation_debug_t *debug);

#endif
