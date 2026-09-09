#ifndef ATTITUDE_ALGORITHM_H
#define ATTITUDE_ALGORITHM_H

#include <stdbool.h>
#include "imu_state.h"

#define ATTITUDE_MAHONY_KP         10.0f
#define ATTITUDE_MAHONY_KI         0.1f
#define ATTITUDE_MAHONY_I_MAX      5.0f
#define ATTITUDE_MIN_DT_S          0.0001f
#define ATTITUDE_MAX_DT_S          0.0500f
#define ATTITUDE_MIN_ACC_NORM_G    0.001f
#define ATTITUDE_ACC_TRUST_FULL    0.10f
#define ATTITUDE_ACC_TRUST_ZERO    0.50f

enum {
    ATTITUDE_PITCH = 0,
    ATTITUDE_ROLL  = 1,
    ATTITUDE_YAW   = 2,
};

void Attitude_Init(imu_state_t *state);
void IMU_State_Convert_Unit(imu_state_t *state);
bool Attitude_Update(imu_state_t *state);

#endif
