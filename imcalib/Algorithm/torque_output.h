/*
 * 文件职责：策略无关的统一力矩结果（torque_output.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：LQR/RL/自起/补偿台架各自求解结果。
 * 输出/去向：保存四髋/双轮力矩及valid，交唯一执行出口检查和发送。
 * 关键边界：valid只说明求解结果可用；驱动发送结果和实体执行成功是另外的状态。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef TORQUE_OUTPUT_H
#define TORQUE_OUTPUT_H

#include <string.h>
#include <stdint.h>
#include "motor_types.h"

/* 力矩命令: 各控制器 (LQR / 手动腿测 / RL) 的统一产物, 只由 task_actuation.c 的 output_dispatch() 下发
 *   dm[]   四髋 (前左/后左/前右/后右, DM 驱动索引), N·m
 *   dji[]  两轮 (左/右, DJI 驱动索引), N·m
 *   valid  0 = 本拍无有效命令 (未投入 / 解算失败 / 未使能), 分发层发零力矩 */
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
