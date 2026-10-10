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
} motor_state_t;

typedef struct {
    uint32_t sequence;
    uint64_t published_ns;
    uint8_t seen;
} feedback_meta_t;

typedef struct {
    uint64_t received_ns;
    uint8_t seen;
} feedback_sample_t;

typedef struct robot_feedback {
    imu_state_t imu;
    motor_state_t motor;
    leg_state_t left;
    leg_state_t right;
    rc_command_t rc;
    feedback_meta_t imu_meta;
    feedback_meta_t motor_meta;
    feedback_meta_t rc_meta;
    feedback_sample_t imu_sample;
    feedback_sample_t motor_sample[DM_MOTOR_NUM + DJI_MOTOR_NUM];
    feedback_sample_t rc_sample;
    uint64_t captured_ns;
} robot_feedback_t;

uint8_t Robot_Feedback_Read(robot_feedback_t *out);
void Robot_Feedback_Publish_Imu(const imu_state_t *next, feedback_sample_t sample);
void Robot_Feedback_Publish_Motor(const motor_state_t *next,
    const leg_state_t *left, const leg_state_t *right,
    const feedback_sample_t sample[DM_MOTOR_NUM + DJI_MOTOR_NUM]);
void Robot_Feedback_Publish_Rc(const rc_command_t *next,
    uint8_t rc_enable, uint8_t mode, feedback_sample_t sample);

#endif
