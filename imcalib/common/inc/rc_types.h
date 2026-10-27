/*
 * 文件职责：DR16原始数据与统一遥控指令类型（rc_types.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：协议层原始通道/在线/时间戳与归一化指令。
 * 输出/去向：声明dr16_t、rc_command_t及拨杆原始取值。
 * 关键边界：原始拨杆编码不等于策略编号；速度/偏航角速度/腿长目标分别为m/s、rad/s、m。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef RC_TYPES_H
#define RC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define DR16_CH_LIMIT    660
#define DR16_SW_UP       1
#define DR16_SW_MID      3
#define DR16_SW_DOWN     2

/* 控制策略 */
typedef enum {
    CTRL_STRATEGY_RL = 0,       /* RL 推理 */
    CTRL_STRATEGY_LQR,          /* LQR 平衡 */
    CTRL_STRATEGY_DISABLE,      /* 左下 / 离线 */
    CTRL_STRATEGY_COMPENSATION = 4, /* 补偿台架 */
} ctrl_strategy_t;

typedef struct {
    ctrl_strategy_t strategy;
    uint8_t rc_enable;
    uint8_t engage;
} rc_control_mode_t;

typedef struct {
    int16_t  ch0;
    int16_t  ch1;
    int16_t  ch2;
    int16_t  ch3;
    int16_t  wheel;
    uint8_t  s1;
    uint8_t  s2;
    int16_t  mx;
    int16_t  my;
    int16_t  mz;
    uint8_t  ml;
    uint8_t  mr;
    uint16_t key;
    bool     online;
    uint32_t last_rx_tick;
    uint64_t received_ns;
} dr16_t;

typedef struct {
    float velocity_target;   /* 前进速度目标m/s */
    float yaw_rate_target;   /* 偏航角速度目标rad/s，不是偏航角 */
    float leg_length_target; /* 最终腿长目标m */
    uint8_t s1;         /* 左拨杆 */
    uint8_t s2;         /* 右拨杆 */
    uint8_t online;
    rc_control_mode_t mode; /* 模式请求 */
} rc_command_t;

#endif
