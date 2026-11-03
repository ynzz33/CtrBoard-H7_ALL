/*
 * 保存映射后的IMU姿态、角速度、比力和竖直运动加速度及有效标志。
 * euler_deg/euler_rad是同一欧拉姿态的两种单位；pitch_world另含倒置分支，不能混用。
 */

#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <stdint.h>
#include "machine_config.h"

/* 欧拉角索引 */
enum {
    ATTITUDE_PITCH = 0,
    ATTITUDE_ROLL  = 1,
    ATTITUDE_YAW   = 2,
};

/* IMU 状态 */
typedef struct {
#if MACHINE_LATENCY_CAPTURE_ENABLE
    uint64_t received_ns;
#endif
    float quat[4];          /* 四元数 */
    float euler_deg[3];     /* 欧拉角 */
    float euler_rad[3];     /* 弧度 */
    float gyro_rad_s[3];    /* 角速度 */
    float acc_g[3];         /* 加速度 G */
    float accel_vertical;  /* 竖直m/s² */
    uint8_t online;         /* 数值有效 */
    uint32_t last_timestamp_ms;
    float pitch_world;     /* 世界俯仰 */
    uint8_t pitch_world_valid; /* 计算有效 */
} imu_state_t;

#endif
