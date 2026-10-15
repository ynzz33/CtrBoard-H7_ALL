/*
 * 文件职责：跨任务反馈发布/读取与元数据（feedback_snapshot.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：imu/comm任务更新后的IMU、电机/腿和遥控结果。
 * 输出/去向：临界区发布，执行/策略任务一次读取结构与接收/发布时间戳。
 * 关键边界：发布时刻不等于传感器采样或CAN接收时刻；sequence表示软件发布，旧样本不能伪装成新反馈。
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
    float pos_continuous_rad[DM_MOTOR_NUM]; /* RL连续标定角，rad；随整份电机快照发布 */
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

/* TODO: 死代码 —— 以下三个类型所属元数据链无消费者，见 feedback_snapshot.c 内标记 */
/* 软件发布元数据：不能代替原传感器/电机样本接收时间。 */
typedef struct {
    uint32_t sequence; /* 发布次数序号，可以回绕 */
    uint64_t published_ns; /* 完成此次软件发布的单调时刻ns */
    uint8_t seen; /* 是否曾发布过该类反馈 */
} feedback_meta_t;

/* 原始样本接收元数据：与解析出的物理反馈配套保存。 */
typedef struct {
    uint64_t received_ns; /* 原数据到达时刻ns，不是读取快照时刻 */
    uint8_t seen; /* 是否有可追溯的样本接收记录 */
} feedback_sample_t;

/* 发布元数据集 */
typedef struct {
    feedback_meta_t imu;
    feedback_meta_t motor;
    feedback_meta_t rc;
    feedback_sample_t imu_sample;
    feedback_sample_t rc_sample;
} feedback_metadata_t;

typedef struct robot_feedback {
    imu_state_t imu;
    motor_state_t motor;
    leg_state_t left;
    leg_state_t right;
    rc_command_t rc;
    /* TODO: 死代码 —— 以下 6 个字段全工程无消费者，见 feedback_snapshot.c 内标记 */
    feedback_meta_t imu_meta;
    feedback_meta_t motor_meta;
    feedback_meta_t rc_meta;
    feedback_sample_t imu_sample;
    feedback_sample_t rc_sample;
    uint64_t captured_ns;
} robot_feedback_t;

uint8_t Robot_Feedback_Read(robot_feedback_t *out);
void Robot_Feedback_Publish_Imu(const imu_state_t *next, feedback_sample_t sample);
void Robot_Feedback_Publish_Motor(const motor_state_t *next,
    const leg_state_t *left, const leg_state_t *right);
void Robot_Feedback_Publish_Rc(const rc_command_t *next,
    feedback_sample_t sample);

#endif
