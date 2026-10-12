/*
 * 文件职责：DJI轮电机反馈与力矩到电流发送（dji.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：CAN编码器/RPM/电流反馈、机器型号/减速比/极性与轮矩命令。
 * 输出/去向：解析连续角和轮速；换算力矩对应raw电流，按反馈ID槽位发送。
 * 关键边界：反馈极性与发送极性分别适用；零力矩是零电流指令，不代表轮子立即停止。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef __DJI_H
#define __DJI_H

#include "main.h"
#include "can_bus.h"
#include "motor_types.h"
#include <stdbool.h>

/* 电机型 */


#define DJI_ANGLE_CPR          8192L
#define DJI_OFFLINE_MS         10u

/* 电机固有参数 (按型号, 与机器无关) */
#define DJI_CURRENT_MAX_M2006  10000
#define DJI_CURRENT_MAX_M3508  16384
/* 满电流时输出轴堵转力矩 (在"标准减速比"下) */
#define DJI_NM_FULL_M2006      (0.18f * 10.0f)
#define DJI_NM_FULL_M3508      (0.30f * 20.0f)
/* 标准减速比 (实机总比见机器表 dji_gear_ratio, 按比例缩放) */
#define DJI_RATIO_STD_M2006    36.0f
#define DJI_RATIO_STD_M3508    19.2f

/* 型号与减速比: 见 machine_config.c (按机器选择) */

/* 电机序 */

/* 固定参 (与 dm 同结构) */
typedef motor_cfg_t dji_motor_config_t;

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
    volatile uint8_t rx_seen;
    uint8_t          angle_pending;
    volatile uint32_t last_rx_tick;
    volatile uint64_t received_ns;
    uint64_t parsed_rx_ns;
    uint8_t parsed_seen;
} dji_motor_feedback_t;

extern const dji_motor_config_t dji_motor_config[DJI_MOTOR_NUM];
extern dji_motor_feedback_t dji_motor_feedback[DJI_MOTOR_NUM];
extern volatile int16_t wheel_current[4]; /* 最后一次左右轮电流指令 raw, 供发送与 VOFA 观测 */

void Dji_Init(void);
void Dji_Parse(void);
void Dji_Circle_Calculate(void);
float Dji_Encoder_To_Rad(int32_t encoder_count);
float Dji_Rpm_To_Rad_S(int16_t rpm);
int16_t Dji_Torque_To_Current(uint8_t index, float torque_nm);
HAL_StatusTypeDef Dji_Send_Wheel_Torque(float left_torque_nm,
                                        float right_torque_nm);
HAL_StatusTypeDef Dji_Send_Current(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const volatile int16_t current_raw[4]);
HAL_StatusTypeDef Dji_All_Stop(void);
bool Dji_Is_Online(uint8_t index);

#endif
