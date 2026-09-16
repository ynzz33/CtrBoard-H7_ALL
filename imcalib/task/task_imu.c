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

    HI229_Process();
    if (!HI229_Online())
    {
        imu_state.online = 0u;
        return;
    }
    sample = HI229_Snapshot();

    /* 去重 */
    if (imu_state.online && sample.ts == imu_state.last_timestamp_ms)
        return;

    /* 首帧初始化 */
    if (!imu_state.online)
        Attitude_Init(&imu_state);

    /* 符号 + 单位 */
    imu_state.quat[0] = sample.quat[0];
    imu_state.quat[1] = HI229_QUAT_SIGN_X * sample.quat[1];
    imu_state.quat[2] = HI229_QUAT_SIGN_Y * sample.quat[2];
    imu_state.quat[3] = HI229_QUAT_SIGN_Z * sample.quat[3];

    imu_state.euler_deg[0] = HI229_EUL_SIGN_ROLL  * sample.eul[0];
    imu_state.euler_deg[1] = HI229_EUL_SIGN_PITCH * sample.eul[1];
    imu_state.euler_deg[2] = HI229_EUL_SIGN_YAW   * sample.eul[2];

    imu_state.gyro_rad_s[0] = HI229_GYR_SIGN_X * sample.gyr[0] * 0.01745329251994f;
    imu_state.gyro_rad_s[1] = HI229_GYR_SIGN_Y * sample.gyr[1] * 0.01745329251994f;
    imu_state.gyro_rad_s[2] = HI229_GYR_SIGN_Z * sample.gyr[2] * 0.01745329251994f;

    /* 四元数归一化 + deg→rad */
    imu_state.online = Attitude_Update(&imu_state) ? 1u : 0u;
    imu_state.last_timestamp_ms = sample.ts;
}
