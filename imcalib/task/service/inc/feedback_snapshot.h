/*
 * IMU任务发布姿态，通信任务成批发布电机/双腿/投影高度并发布遥控命令。
 * 读取时在临界区复制这些最新结构，防止半更新；设备实际采样时刻仍然不同。
 */

#ifndef FEEDBACK_SNAPSHOT_H
#define FEEDBACK_SNAPSHOT_H

#include "motor_types.h"
#include "rc_types.h"
#include "imu_state.h"
#include "leg_solver.h"

typedef struct {
    float pos_rad[DM_MOTOR_NUM];        /* 解码角 */
    float pos_zero_rad[DM_MOTOR_NUM];   /* 加零点 */
    float pos_continuous_rad[DM_MOTOR_NUM]; /* RL连续标定角，rad */
    float vel_rad_s[DM_MOTOR_NUM];
    float trq_nm[DM_MOTOR_NUM];
    uint32_t last_rx_tick[DM_MOTOR_NUM]; /* 各髋最新有效帧接收时刻ms，用于在线期限。 */
    uint64_t parsed_rx_ns[DM_MOTOR_NUM]; /* 各髋有效反馈帧携带的接收时刻ns。 */
    uint8_t online[DM_MOTOR_NUM];
} dm_motor_state_t;

typedef struct {
    float angle_rad[DJI_MOTOR_NUM];
    float angle_total_rad[DJI_MOTOR_NUM];
    float vel_rad_s[DJI_MOTOR_NUM];
    int16_t current_raw[DJI_MOTOR_NUM];
    uint32_t last_rx_tick[DJI_MOTOR_NUM];
    uint8_t online[DJI_MOTOR_NUM];
} dji_motor_state_t;

typedef struct {
    dm_motor_state_t dm;
    dji_motor_state_t dji;
    uint32_t timestamp_ms; /* 通信任务整理电机状态的HAL时刻ms。 */
    uint8_t updated;
    uint8_t valid; /* 在线且无故障 */
} motor_state_t;

/* 镜像使用发布完成时刻判断软件数据是否过期；不是传感器采样或CAN接收时刻。 */
typedef struct {
    uint64_t published_ns; /* 软件完整发布本组数据的上电相对时刻ns。 */
    uint8_t seen; /* 已发布过数据，不等于该数据当前有效。 */
} feedback_meta_t;

typedef struct robot_feedback {
    imu_state_t imu;
    motor_state_t motor;
    leg_state_t left;
    leg_state_t right;
    rc_command_t rc;
    float height; /* 与motor/left/right同批发布的公共几何估计。 */
    uint8_t height_valid;
    feedback_meta_t imu_meta;
    feedback_meta_t motor_meta;
    uint64_t captured_ns; /* 本次软件快照时刻，与来源采样时刻分开。 */
} robot_feedback_t;

uint8_t Robot_Feedback_Read(robot_feedback_t *out);
void Robot_Feedback_Publish_Imu(const imu_state_t *next);
void Robot_Feedback_Publish_Motor(const motor_state_t *next,
    const leg_state_t *left, const leg_state_t *right);
void Robot_Feedback_Publish_Rc(const rc_command_t *next);

#endif
