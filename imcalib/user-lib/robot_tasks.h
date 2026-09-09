#ifndef __ROBOT_TASKS_H
#define __ROBOT_TASKS_H

#include "main.h"
#include "cmsis_os.h"
#include "imu_state.h"
#include "dm.h"
#include "dji.h"
#include "leg_solver.h"

typedef struct {
    float pos_rad[DM_MOTOR_NUM];
    float vel_rad_s[DM_MOTOR_NUM];
    float trq_nm[DM_MOTOR_NUM];
    uint32_t last_rx_tick[DM_MOTOR_NUM];
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
    float   a[6];      /* 6 维动作 */
    uint8_t updated;
} action_state_t;

typedef struct {
    float   vx;
    float   yaw_rate;
    float   height;
    uint8_t mode;
} command_state_t;

/* 故障位 */
#define FAULT_NONE    0u
#define FAULT_IMU     0x01u
#define FAULT_RC      0x02u
#define FAULT_MOTOR   0x04u
#define FAULT_CAN     0x08u
#define FAULT_ACTION  0x10u

extern imu_state_t       imu_state;
extern motor_state_t     motor_state;
extern leg_state_t       leg_l;
extern leg_state_t       leg_r;
extern action_state_t    action_state;
extern command_state_t   command_state;
extern volatile uint32_t ctrl_fault;

extern osSemaphoreId ctrl_tick_sem_handle;

void Robot_Control_Init(void);
void imu_task_body(void);
void ctrl_task_body(void);
void output_task_body(void);
void monitor_task_body(void);

#endif
