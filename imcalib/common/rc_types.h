/*
 * 文件职责：DR16原始数据与统一遥控指令类型（rc_types.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：协议层原始通道/在线/时间戳与归一化指令。
 * 输出/去向：声明dr16_t、rc_command_t及拨杆原始取值。
 * 关键边界：原始拨杆编码不等于策略编号；vel/yaw指令单位与len调节输入需要分清。
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
    float vel;          /* 前进 m/s */
    float yaw;          /* 转向 rad/s */
    float len;          /* 拨轮 腿长 */
    uint8_t s1;         /* 左拨杆 */
    uint8_t s2;         /* 右拨杆 */
    uint8_t online;
} rc_command_t;

#endif
