/*
 * 统一遥控目标及策略/使能类型，供通信任务发布、执行和推理任务复制。
 * 目标单位及每个拨杆的转换规则见rc_command；选择策略不等于实际投入。
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
