/*
 * 文件职责：公共电机反馈类型与电机枚举（motor_types.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：DM/DJI驱动输出的物理角、速、矩、在线和接收时间。
 * 输出/去向：为任务快照/算法提供统一数据布局。
 * 关键边界：电机枚举决定数组顺序；物理量已在驱动边界处理标定，算法不能再重复反向。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>

typedef enum {
    DM_MOTOR_LEG_F_LFT = 0,
    DM_MOTOR_LEG_B_LFT,
    DM_MOTOR_LEG_F_RGT,
    DM_MOTOR_LEG_B_RGT,
    DM_MOTOR_NUM,
} dm_motor_idx_t;

typedef enum {
    DJI_M2006 = 0,
    DJI_M3508 = 1,
} dji_motor_type_t;

#define DJI_MOTOR_WHEEL_LFT    0u
#define DJI_MOTOR_WHEEL_RGT    1u
#define DJI_MOTOR_NUM          2u

typedef struct {
    uint16_t             feedback_id;
    uint16_t             control_id;
} motor_cfg_t;

#endif
