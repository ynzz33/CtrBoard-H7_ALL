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
