#ifndef __MACHINE_CONFIG_H
#define __MACHINE_CONFIG_H

#include <stdint.h>

/* 机器编号 */
#define MACHINE_ID_CHUANLIANTUI   0u
#define MACHINE_ID_LOCAL          1u
#define MACHINE_NUM               2u

/* 电机数量 (须与 dm.c / dji.c 的枚举一致) */
#define MACHINE_LEG_NUM           4u
#define MACHINE_WHEEL_NUM         2u

/* 上电默认机器: 换机器改这一行 */
#define MACHINE_DEFAULT           MACHINE_ID_CHUANLIANTUI

/* 一路电机的极性 */
typedef struct {
    int8_t fb;      /* 反馈极性 */
    int8_t out;     /* 输出极性 */
} motor_sign_t;

/* 一台机器的全部参数 */
typedef struct {
    const char *name;
    /* 轮: 型号(0=M2006, 1=M3508) + 总传动比 + 满限幅力矩(Nm) */
    uint8_t     dji_type;
    float       dji_gear_ratio;
    float       dji_trq_clamp;
    /* 腿: MIT 三个满量程 + 满限幅力矩(Nm) */
    float       dm_pos_max;
    float       dm_vel_max;
    float       dm_trq_max;
    float       dm_trq_clamp;
    /* 极性: 腿 4 台 (前左/后左/前右/后右), 轮 2 个 (左/右) */
    motor_sign_t dm_sign[MACHINE_LEG_NUM];
    motor_sign_t dji_sign[MACHINE_WHEEL_NUM];
    /* 总线号: 1=FDCAN1 2=FDCAN2 3=FDCAN3 */
    uint8_t     dm_bus[MACHINE_LEG_NUM];
    uint8_t     dji_bus;
    /* 电机零点: 4 台腿 (前左/后左/前右/后右), 在 dm.c 解码时叠加 */
    float       dm_zero[MACHINE_LEG_NUM];
    /* 腿几何: 杆长 + 腿摆角零位 (左/右) */
    float       leg_lu;
    float       leg_lg;
    float       leg_off_phi0[2];
} machine_cfg_t;

extern const machine_cfg_t machine_table[MACHINE_NUM];
extern const machine_cfg_t *machine;     /* 当前生效的机器 */

void    Machine_Select(uint8_t id);
uint8_t Machine_Id(void);

#endif
