#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>

typedef enum {
    DM_MOTOR_LEG_F_LFT = 0,
    DM_MOTOR_LEG_B_LFT,
    DM_MOTOR_LEG_F_RGT,
    DM_MOTOR_LEG_B_RGT,
    DM_MOTOR_NUM,
} dm_motor_idx_t;

typedef enum {
    DJI_M2006 = 0,
    DJI_M3508 = 1,
} dji_motor_type_t;

#define DJI_MOTOR_WHEEL_LFT    0u
#define DJI_MOTOR_WHEEL_RGT    1u
#define DJI_MOTOR_NUM          2u

typedef struct {
    uint16_t             feedback_id;
    uint16_t             control_id;
} motor_cfg_t;

#endif
