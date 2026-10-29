/*
 * 文件职责：达妙髋电机MIT协议与状态/使能管理（dm.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：CAN反馈、机器ID/极性/零点/MIT量程与四髋力矩请求。
 * 输出/去向：解析物理角/速/矩和接收时间，维护在线/故障/使能，量化MIT命令并发送。
 * 关键边界：所有实体极性/零点在驱动边界统一处理；控制层传物理量，不再重复乘符号。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef __DM_H
#define __DM_H

#include "main.h"
#include "can_bus.h"
#include "machine_config.h"
#include "motor_types.h"
#include <stdbool.h>

/* 电机序 */


/* 命令码 */
typedef enum {
    DM_CMD_CLEAR_ERROR = 0xFBu,
    DM_CMD_ENABLE      = 0xFCu,
    DM_CMD_DISABLE     = 0xFDu,
    DM_CMD_SET_ZERO    = 0xFEu,
} dm_cmd_t;

/* 编码值 */
#define DM_ANGLE_CPR        65536L
#define DM_MIT_FIELD_MAX    0x0FFFu

/* MIT量程: 见 machine_config.h (dm.c 内引用) */

/* 超时值 */
#define DM_OFFLINE_MS       MACHINE_DM_OFFLINE_MS

/* 固定参 (与 dji 同结构) */
typedef motor_cfg_t dm_motor_config_t;

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
    float             pos_rad;          /* 解码角 */
    float             pos_zero_rad;     /* 加零点 */
    int32_t           angle_total;      /* 解码方向下的累计编码值 */
    float             pos_continuous_rad; /* 零点后连续角，rad；RL使用 */
    float             vel_rad_s;
    float             trq_nm;
    volatile uint8_t  raw_pending;
    volatile uint8_t  rx_seen;
    volatile uint32_t last_rx_tick;
    volatile uint64_t last_rx_ns;
    uint64_t parsed_rx_ns;
    uint8_t parsed_seen;
} dm_motor_feedback_t;

extern const dm_motor_config_t dm_motor_config[DM_MOTOR_NUM];
extern dm_motor_feedback_t dm_motor_feedback[DM_MOTOR_NUM];

void Dm_Init(void);
void Dm_Parse(void);
bool Dm_Is_Online(uint8_t index);
bool Dm_Is_Enabled(uint8_t index);
bool Dm_Has_Fault(uint8_t index);
void Dm_Enable_Watchdog(void);
void Dm_Disable_Watchdog(void);
float Dm_Uint_To_Float(uint16_t value, float min, float max, uint8_t bits);
uint16_t Dm_Float_To_Uint(float value, float min, float max, uint8_t bits);
HAL_StatusTypeDef Dm_Mit_Control(uint8_t index, uint16_t angle_raw,
                                 uint16_t vel_raw, uint16_t kp_raw,
                                 uint16_t kd_raw, uint16_t trq_raw);
HAL_StatusTypeDef Dm_Send_Command(uint8_t index, uint8_t command);
HAL_StatusTypeDef Dm_All_Enable(void);
HAL_StatusTypeDef Dm_All_Disable(void);
HAL_StatusTypeDef Dm_Send_Zero(void);
HAL_StatusTypeDef Dm_Send_Torque(const float torque[DM_MOTOR_NUM]);

#endif
