/*
 * 按编译机型选择增益表，将左右腿长夹到表域后求40个K系数并检查有限性。
 * 夹紧仅作用于查表输入，不改写实际反馈腿长；系数排列由增益接口定义。
 */

#ifndef LQR_GAIN_TABLE_H
#define LQR_GAIN_TABLE_H

#include <stdint.h>
#include "machine_config.h"

#define LQR_STATE_HIP_FRONT_V2 2u
#define LQR_INPUT_WHEELS_HIPS_V1 1u

typedef void (*lqr_gain_eval_t)(float lL, float lR, float K[40]);
typedef struct {
    const char *table_id;
    const char *model;
    uint8_t machine_id;
    uint8_t state_schema;
    uint8_t input_schema;
    uint8_t checked;
    float dt;
    float len_min;
    float len_max;
    float wheel_r;
    float leg_lu;
    float leg_lg;
    float x_eq[10];
    float u_eq[4];
    lqr_gain_eval_t eval;
} lqr_gain_desc_t;

extern const lqr_gain_desc_t lqr_gain_small;
extern const lqr_gain_desc_t lqr_gain_big;
const lqr_gain_desc_t *LQR_Gain_Info(void);
uint8_t LQR_Gain_Eval(float lL, float lR, float K[40]);
void LQR_K_WBR(float lL, float lR, float K[40]);

#endif
