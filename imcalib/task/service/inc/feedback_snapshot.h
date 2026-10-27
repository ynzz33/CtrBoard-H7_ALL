/*
 * 文件职责：跨任务反馈发布/读取（feedback_snapshot.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：imu/comm任务更新后的IMU、电机/腿和遥控结果。
 * 输出/去向：临界区发布，执行/策略任务一次读取同批结构。
 * 关键边界：不同设备仍是异步采样；DM原始接收时刻仍随电机反馈发布。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
    float vel_rad_s[DM_MOTOR_NUM];
    float trq_nm[DM_MOTOR_NUM];
    uint32_t last_rx_tick[DM_MOTOR_NUM];
    uint64_t parsed_rx_ns[DM_MOTOR_NUM];
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
    uint32_t timestamp_ms;
    uint8_t updated;
    uint8_t valid; /* 在线且无故障 */
} motor_state_t;

/* 镜像使用发布完成时刻判断软件数据是否过期；不是传感器采样或CAN接收时刻。 */
typedef struct {
    uint64_t published_ns;
    uint8_t seen;
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
