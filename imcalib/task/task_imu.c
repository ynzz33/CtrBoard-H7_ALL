#include "robot_control.h"
#include "hi229.h"
#include "Attitude_Algorithm.h"

void imu_task_init(void)
{
    Attitude_Init(&imu_state);
}

void imu_task_body(void)
{
    hi229_data_t sample;
    uint8_t first_sample;
    uint8_t new_sample;

    HI229_Process();
    if (!HI229_Online())
    {
        imu_state.online = 0u;
        imu_state.timestamp_valid = 0u;
        return;
    }
    sample = HI229_Snapshot();
    first_sample = (uint8_t)!imu_state.timestamp_valid;
    new_sample = (uint8_t)(first_sample || sample.ts != imu_state.last_timestamp_ms);
    if (!new_sample)
    {
        return;
    }
    if (first_sample)
    {
        Attitude_Init(&imu_state);
        imu_state.timestamp_valid = 1u;
    }
    imu_state.input.gyro_dps[0] = HI229_GYR_SIGN_X * sample.gyr[0];
    imu_state.input.gyro_dps[1] = HI229_GYR_SIGN_Y * sample.gyr[1];
    imu_state.input.gyro_dps[2] = HI229_GYR_SIGN_Z * sample.gyr[2];
    imu_state.input.accel_g[0] = HI229_ACC_SIGN_X * sample.acc[0];
    imu_state.input.accel_g[1] = HI229_ACC_SIGN_Y * sample.acc[1];
    imu_state.input.accel_g[2] = HI229_ACC_SIGN_Z * sample.acc[2];
    imu_state.input.timestamp_ms = sample.ts;
    imu_state.reference.quat[0] = sample.quat[0];
    imu_state.reference.quat[1] = HI229_QUAT_SIGN_X * sample.quat[1];
    imu_state.reference.quat[2] = HI229_QUAT_SIGN_Y * sample.quat[2];
    imu_state.reference.quat[3] = HI229_QUAT_SIGN_Z * sample.quat[3];
    imu_state.reference.euler_deg[0] = HI229_EUL_SIGN_ROLL * sample.eul[0];
    imu_state.reference.euler_deg[1] = HI229_EUL_SIGN_PITCH * sample.eul[1];
    imu_state.reference.euler_deg[2] = HI229_EUL_SIGN_YAW * sample.eul[2];
    IMU_State_Convert_Unit(&imu_state);

    if (!first_sample && sample.ts > imu_state.last_timestamp_ms)
    {
        imu_state.input.dt_s = (float)(sample.ts - imu_state.last_timestamp_ms) * 0.001f;
    }
    else
    {
        imu_state.input.dt_s = 0.005f;
    }
    imu_state.online = Attitude_Update_From_HI229(&imu_state) ? 1u : 0u;
    imu_state.last_timestamp_ms = sample.ts;
}
