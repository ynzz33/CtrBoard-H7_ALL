#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <stdint.h>
#include "pid.h"

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float gyro_rad_s[3];
    uint32_t timestamp_ms;
    float dt_s;
} imu_input_t;

typedef struct {
    float quat[4];
    float euler_deg[3];
    float euler_rad[3];
} imu_reference_t;

typedef struct {
    float quat[4];
    float euler_deg[3];
    float euler_rad[3];
    float rotation_t[3][3];
    float accel_normed[3];
    float accel_trust;
    float mahony_error[3];
    float mahony_output[3];
} imu_output_t;

typedef struct {
    imu_input_t input;
    imu_reference_t reference;
    imu_output_t output;
    pid_t mahony_pid[3];
    uint32_t last_timestamp_ms;
    uint8_t online;
    uint8_t timestamp_valid;
    uint8_t initialized;
} imu_state_t;

#endif
