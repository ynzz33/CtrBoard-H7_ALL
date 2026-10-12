/*
 * 文件职责：公共IMU状态数据结构（imu_state.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：HI229解析后经取轴、单位换算和归一化的测量。
 * 输出/去向：向控制快照提供四元数、欧拉角、角速度、比力及pitch_world有效性。
 * 关键边界：欧拉角同时保存deg/rad；pitch_world是倒置判别补角，不能混同原LQR pitch。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <stdint.h>

/* 欧拉角索引 */
enum {
    ATTITUDE_PITCH = 0,
    ATTITUDE_ROLL  = 1,
    ATTITUDE_YAW   = 2,
};

/* IMU 状态 */
typedef struct {
    float quat[4];          /* 四元数 */
    float euler_deg[3];     /* 欧拉角 */
    float euler_rad[3];     /* 弧度 */
    float gyro_rad_s[3];    /* 角速度 */
    float acc_g[3];         /* 加速度 G */
    uint8_t online;         /* 在线 */
    uint32_t last_timestamp_ms;
    float pitch_world;     /* 世界俯仰 */
    uint8_t pitch_world_valid; /* 计算有效 */
} imu_state_t;

#endif
