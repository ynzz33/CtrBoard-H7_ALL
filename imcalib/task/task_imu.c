/*
 * 从HI229取新样本，按机器配置映射轴和符号，换算单位并校验姿态。
 * 计算供大角度恢复使用的pitch_world后发布；断流发布online=0、pitch_world_valid=0。
 */

#include "robot_control.h"
#include "robot_state.h"
#include "hi229.h"
#include "Attitude_Algorithm.h"
#include "machine_config.h"
#include "pitch_world.h"

#define IMU_PITCH_WORLD_IDX  ATTITUDE_ROLL /* 清姿态历史；后续首个有效HI229样本重新建立姿态状态。 */
void imu_task_init(void)
{
    Attitude_Init(&imu_state);
}

/* 解析并去重HI229样本，按机器表取轴/符号，将角速度换为rad/s，计算pitch_world后发布。
 * 离线时清online与pitch_world_valid；相同模块时间戳不重复发布。 */
void imu_task_body(void)
{
    hi229_data_t sample;
    imu_state_t next = imu_state;
    const imu_cfg_t *cfg;

    HI229_Process();
    if (!HI229_Online())
    {
        next.online = 0u;
        next.pitch_world = 0.0f;
        next.pitch_world_valid = 0u;
        Robot_Feedback_Publish_Imu(&next);
        return;
    }
    sample = HI229_Snapshot();

    /* 去重 */
    if (next.online && sample.ts == next.last_timestamp_ms)
        return;

    /* 首帧初始化 */
    if (!next.online)
        Attitude_Init(&next);

    /* 姿态和惯性量各按来源通道与极性映射 */
    cfg = &machine->imu;
    next.quat[0] = sample.quat[0];
    for (uint8_t i = 0u; i < 3u; i++)
    {
        next.quat[i + 1u]        = (float)cfg->quat_sign[i] * sample.quat[cfg->quat_src[i] + 1u];
        next.euler_deg[i]        = (float)cfg->eul_sign[i] * sample.eul[cfg->eul_src[i]];
        next.gyro_rad_s[i]       = (float)cfg->gyr_sign[i] * sample.gyr[cfg->gyr_src[i]] * 0.01745329251994f;
        next.acc_g[i]            = (float)cfg->acc_sign[i] * sample.acc[cfg->acc_src[i]];
    }

    /* 完整反馈校验 */
    next.online = Attitude_Update(&next) ? 1u : 0u;
    next.pitch_world = 0.0f;
    next.pitch_world_valid = 0u;
    if (next.online)
    {
        next.pitch_world_valid = Pitch_World_Calc(next.euler_rad[IMU_PITCH_WORLD_IDX], next.quat, &next.pitch_world) ? 1u : 0u;
    }
#if MACHINE_LATENCY_CAPTURE_ENABLE
    next.received_ns = sample.received_ns;
#endif
    next.last_timestamp_ms = sample.ts;
    Robot_Feedback_Publish_Imu(&next);
}
