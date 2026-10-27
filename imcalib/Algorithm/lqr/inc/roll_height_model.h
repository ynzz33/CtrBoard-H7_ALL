/* MATLAB固定维度契约；输入为补偿前虚拟轴向F请求，不是FN估计或电机力矩。 */
#ifndef ROLL_HEIGHT_MODEL_H
#define ROLL_HEIGHT_MODEL_H

#include <stdint.h>

#define RH_NX 4u
#define RH_NU 2u
#define RH_NE (RH_NX + RH_NU)
#define RH_MODEL_IFACE 5u

typedef struct {
    uint32_t iface;
    uint8_t machine_id; /* 生成数据与机型绑定，不是运行模式 */
    float ts;
    float K[RH_NU][RH_NX];
    float A[RH_NX][RH_NX];
    float B[RH_NX][RH_NU];
    float f[RH_NX];
    float L[RH_NE][RH_NX]; /* 先预测再校正；扩张态为四个物理状态+左右恒力偏置 */
} roll_height_model_t;

extern const roll_height_model_t roll_height_model;

#endif
