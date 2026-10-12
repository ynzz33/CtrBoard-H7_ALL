/*
 * 文件职责：弹簧和完整腿自重补偿的组合入口（leg_compensation.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：已映射的四髋基础力矩、双腿几何、世界腿角和弹簧接入选择。
 * 输出/去向：先加弹簧抵消，再加重力摆矩，最后逐电机限幅并发布补偿诊断。
 * 关键边界：spring_enable仅决定命令是否抵消实体弹簧，不改变检测使用的实体弹簧模型；补偿只加一次。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef LEG_COMPENSATION_H
#define LEG_COMPENSATION_H

#include "gravity_comp.h"

typedef struct {
    gravity_comp_result_t gravity;
    float spring_dm[4];
    float raw_dm[4];
    uint8_t saturated;
    uint8_t bench;
} compensation_debug_t;

uint8_t Leg_Compensation_Apply(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float limit,
    float output_dm[4], compensation_debug_t *debug);
uint8_t Leg_Compensation_Apply_Mode(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float limit,
    uint8_t spring_enable, float output_dm[4], compensation_debug_t *debug);

#endif
