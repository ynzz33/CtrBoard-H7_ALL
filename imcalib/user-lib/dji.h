#ifndef __DJI_H
#define __DJI_H

#include "main.h"
#include "can_bus.h"
#include <stdbool.h>

/* 电机型 */
typedef enum {
    DJI_M2006 = 0,
    DJI_M3508 = 1,
} dji_motor_type_t;

#define DJI_CURRENT_MAX_M2006  10000
#define DJI_CURRENT_MAX_M3508  16384
#define DJI_ANGLE_CPR          8192L
#define DJI_RAD_PER_COUNT      (0.0007669903939f)
#define DJI_RPM_TO_RAD_S       (0.1047197551f)
#define DJI_OFFLINE_MS         10u

/* 电机序 */
#define DJI_MOTOR_WHEEL_LFT    0u
#define DJI_MOTOR_WHEEL_RGT    1u
#define DJI_MOTOR_NUM          2u

/* 固定参 */
typedef struct {
    FDCAN_HandleTypeDef *handle;
    dji_motor_type_t     type;
    uint8_t              motor_id;
    uint16_t             feedback_id;
    uint16_t             control_id;
} dji_motor_config_t;

/* 反馈值 */
typedef struct {
    volatile uint8_t raw_data[8];
    uint16_t         angle_raw;
    int16_t          vel_raw;
    int16_t          current_raw;
    int8_t           temp_raw;
    int32_t          angle_total;
    float            angle_rad;
    float            angle_total_rad;
    float            vel_rad_s;
    volatile uint8_t raw_pending;
    uint8_t          angle_pending;
    volatile uint32_t last_rx_tick;
} dji_motor_feedback_t;

#define DJI_FB_ANGLE(data) \
    ((uint16_t)(((uint16_t)(data)[0] << 8) | (uint16_t)(data)[1]))
#define DJI_FB_VEL(data) \
    ((int16_t)(((uint16_t)(data)[2] << 8) | (uint16_t)(data)[3]))
#define DJI_FB_CURRENT(data) \
    ((int16_t)(((uint16_t)(data)[4] << 8) | (uint16_t)(data)[5]))
#define DJI_FB_TEMP(data) ((int8_t)(data)[6])

extern const dji_motor_config_t dji_motor_config[DJI_MOTOR_NUM];
extern dji_motor_feedback_t dji_motor_feedback[DJI_MOTOR_NUM];

void Dji_Init(void);
void Dji_Parse(void);
void Dji_Circle_Calculate(void);
float Dji_Encoder_To_Rad(int32_t encoder_count);
float Dji_Rpm_To_Rad_S(int16_t rpm);
HAL_StatusTypeDef Dji_Send_Current(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const int16_t current_raw[4]);
HAL_StatusTypeDef Dji_All_Stop(void);
bool Dji_Is_Online(uint8_t index);

#endif
