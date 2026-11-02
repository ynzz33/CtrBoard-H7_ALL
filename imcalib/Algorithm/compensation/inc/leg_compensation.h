/*
 * 按enabled位标志计算气弹簧和本拍选定姿态引用的一份重力缓存；此时尚未叠加电机输出。
 * Apply_Mode按输出开关将选中的缓存加到base_dm，再对四髋逐个限矩。
 */

#ifndef LEG_COMPENSATION_H
#define LEG_COMPENSATION_H

#include "gravity_comp.h"
#include "gas_spring.h"
#include "imu_state.h"

enum {
    LEG_COMP_SPRING = 1u,
    LEG_COMP_GRAVITY = 2u, /* 正常输出欧拉俯仰；与WORLD互斥。 */
    LEG_COMP_GRAVITY_WORLD = 4u, /* 自起/测试输出pitch_world；与普通模式互斥。 */
};

/* 输出补偿缓存；重力模式互斥，AIR不读取此结构。 */
typedef struct {
    gas_spring_result_t spring; /* 实体弹簧力与用于抵消的四髋力矩缓存。 */
    gravity_comp_result_t gravity; /* 本拍选定的输出重力补偿：正常欧拉俯仰，自起/测试pitch_world。 */
} leg_compensation_values_t;

typedef struct {
    gravity_comp_result_t gravity; /* 实际选中并应用到输出的重力补偿诊断。 */
    float spring_dm[4];
    float unclamped_dm[4];
    uint8_t saturated;
    uint8_t bench;
} compensation_debug_t;

void Leg_Compensation_Prepare(leg_compensation_values_t *values, const leg_state_t *left, const leg_state_t *right, const imu_state_t *imu, uint8_t enabled);
uint8_t Leg_Compensation_Apply(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, float output_dm[4], compensation_debug_t *debug);
uint8_t Leg_Compensation_Apply_Mode(const gas_spring_result_t *spring, const gravity_comp_result_t *gravity, const float base_dm[4], float limit, uint8_t spring_enable, float output_dm[4], compensation_debug_t *debug);

#endif
