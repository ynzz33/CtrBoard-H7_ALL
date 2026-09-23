#include "robot_control.h"
#include "hi229.h"
#include "Attitude_Algorithm.h"
#include "machine_config.h"

void imu_task_init(void)
{
    Attitude_Init(&imu_state);
}

/* 读取 + 按机器表取轴/乘符号 + 单位换算 */
void imu_task_body(void)
{
    hi229_data_t sample;
    const imu_cfg_t *cfg;
    uint8_t i;

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

    /* 欧拉角、四元数分别按源通道重映射，再独立应用极性 */
    cfg = &machine->imu;
    imu_state.quat[0] = sample.quat[0];
    for (i = 0u; i < 3u; i++)
    {
        imu_state.quat[i + 1u]   = (float)cfg->quat_sign[i]
                                 * sample.quat[cfg->quat_src[i] + 1u];
        imu_state.euler_deg[i]   = (float)cfg->eul_sign[i] * sample.eul[cfg->eul_src[i]];
        imu_state.gyro_rad_s[i]  = (float)cfg->gyr_sign[i] * sample.gyr[i] * 0.01745329251994f;
        imu_state.acc_g[i]       = (float)cfg->acc_sign[i] * sample.acc[i];
    }

    /* 四元数归一化 + deg→rad */
    imu_state.online = Attitude_Update(&imu_state) ? 1u : 0u;
    imu_state.last_timestamp_ms = sample.ts;
}
