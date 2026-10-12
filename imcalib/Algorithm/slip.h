/*
 * 文件职责：带加速度状态的速度融合与异常轮速降权（slip.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：轮速运动学观测、IMU前向加速度、dt和接触许可。
 * 输出/去向：维护速度/加速度及协方差；AIR停用轮速修正但保留预测/IMU修正。
 * 关键边界：不发布独立打滑状态；降权不是主动轮速制动，接触标志由执行层传入。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef SLIP_H
#define SLIP_H
#include <stdint.h>

/* 速度补偿参数，方差按对应状态单位的平方计。 */
typedef struct {
    float p0_velocity;
    float p0_acceleration;
    float q_velocity;       /* 每1ms方差 */
    float q_acceleration;   /* 每1ms方差 */
    float r_velocity;
    float r_acceleration;
    float gate;             /* 新息σ倍数 */
    float gate_min;         /* 最低门限m/s */
} slip_param_t;

typedef struct {
    float velocity;         /* 速度m/s */
    float acceleration;     /* 加速度m/s² */
    float covariance[2][2]; /* 状态协方差 */
    float innovation;       /* 轮速新息 */
    float limit;            /* 新息门限 */
    float noise_scale;      /* 轮速降权倍数 */
    uint8_t initialized;
    uint8_t valid;
} slip_state_t;

extern slip_param_t slip_param;
void Slip_Init(slip_state_t *st, float v0, float a0);
void Slip_Reset(slip_state_t *st);
uint8_t Slip_Update(slip_state_t *st, float wheel_v, float imu_acc, float dt);
uint8_t Slip_Update_Contact(slip_state_t *st, float wheel_v, float imu_acc,
                            float dt, uint8_t wheel_contact);
#endif
