/*
 * 统一候选力矩：dm按四髋索引、dji按左/右轮保存N·m，valid仅说明计算可用。
 * 最终由Control_Output_Apply检查许可并发送；清结构会得到无效零候选。
 */

#ifndef TORQUE_OUTPUT_H
#define TORQUE_OUTPUT_H

#include <string.h>
#include <stdint.h>
#include "motor_types.h"

/* 所有控制策略共用的候选输出，发送接口见task/control/control_output.c。 */
typedef struct {
    float   dm[DM_MOTOR_NUM];
    float   dji[DJI_MOTOR_NUM];
    uint8_t valid;
} torque_output_t;

static inline void Torque_Output_Clear(torque_output_t *t)
{
    memset(t, 0, sizeof(*t));
}

#endif
