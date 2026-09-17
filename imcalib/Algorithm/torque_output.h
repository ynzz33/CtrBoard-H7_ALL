#ifndef TORQUE_OUTPUT_H
#define TORQUE_OUTPUT_H

#include "dm.h"
#include "dji.h"

/* 力矩输出: DM 与 DJI 分离, 各自用本驱动索引 */
typedef struct {
    float dm[DM_MOTOR_NUM];
    float dji[DJI_MOTOR_NUM];
} torque_output_t;

#endif
