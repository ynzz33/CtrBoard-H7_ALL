#ifndef __DM_H
#define __DM_H

#include "main.h"
#include "can_bus.h"
#include <stdbool.h>

/* 电机型 */
typedef enum {
    DM_MOTOR_J4310 = 0,
    DM_MOTOR_TYPE_NUM,
} dm_motor_type_t;

/* 电机序 */
#define DM_MOTOR_LEG_F_LFT  0u
#define DM_MOTOR_LEG_B_LFT  1u
#define DM_MOTOR_LEG_F_RGT  2u
#define DM_MOTOR_LEG_B_RGT  3u
#define DM_MOTOR_NUM        4u

/* 命令码 */
#define DM_CMD_CLEAR_ERROR  0xFBu
#define DM_CMD_ENABLE       0xFCu
#define DM_CMD_DISABLE      0xFDu
#define DM_CMD_SET_ZERO     0xFEu

/* 编码值 */
#define DM_ANGLE_CPR        65536L
#define DM_MIT_FIELD_MAX    0x0FFFu

/* MIT量程 */
#define DM_MIT_POS_MIN      (-3.14159f)
#define DM_MIT_POS_MAX      ( 3.14159f)
#define DM_MIT_VEL_MIN      (-30.0f)
#define DM_MIT_VEL_MAX      ( 30.0f)
#define DM_MIT_TRQ_MIN      (-10.0f)
#define DM_MIT_TRQ_MAX      ( 10.0f)

/* 超时值 */
#define DM_OFFLINE_MS       10u

/* 固定参 */
typedef struct {
    FDCAN_HandleTypeDef *handle;
    dm_motor_type_t      type;
    uint16_t             feedback_id;
    uint16_t             control_id;
} dm_motor_config_t;

/* 反馈值 */
typedef struct {
    volatile uint8_t  raw_data[8];
    uint16_t          angle_raw;
    uint16_t          vel_raw;
    uint16_t          trq_raw;
    uint8_t           err_raw;
    uint8_t           motor_id;
    uint8_t           temp_mos;
    uint8_t           temp_rotor;
    int32_t           angle_total;
    float             pos_rad;
    float             vel_rad_s;
    float             trq_nm;
    volatile uint8_t  raw_pending;
    volatile uint32_t last_rx_tick;
} dm_motor_feedback_t;

extern const dm_motor_config_t dm_motor_config[DM_MOTOR_NUM];
extern dm_motor_feedback_t dm_motor_feedback[DM_MOTOR_NUM];

void Dm_Init(void);
void Dm_Parse(void);
bool Dm_Is_Online(uint8_t index);
float Dm_Uint_To_Float(uint16_t value, float min, float max, uint8_t bits);
HAL_StatusTypeDef Dm_Mit_Control(uint8_t index, uint16_t angle_raw,
                                 uint16_t vel_raw, uint16_t kp_raw,
                                 uint16_t kd_raw, uint16_t trq_raw);
HAL_StatusTypeDef Dm_Send_Command(uint8_t index, uint8_t command);

#endif
