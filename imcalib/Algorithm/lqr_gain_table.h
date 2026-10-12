/*
 * 文件职责：两机LQR增益表选择、兼容校验与求值（lqr_gain_table.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：编译选机、machine配置、两腿长度和可选小机历史表。
 * 输出/去向：校验周期/几何/状态约定/工作域，再求40个状态优先排列的K系数。
 * 关键边界：不能把某机K用于另一机；只夹增益求值域，不把实测腿长改成边界值。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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

enum {
    LQR_GAIN_OK = 0,
    LQR_GAIN_BAD_TABLE,
    LQR_GAIN_BAD_MACHINE,
    LQR_GAIN_BAD_SCHEMA,
    LQR_GAIN_BAD_PERIOD,
    LQR_GAIN_BAD_GEOMETRY,
    LQR_GAIN_BAD_DOMAIN,
};

extern const lqr_gain_desc_t lqr_gain_small;
extern const lqr_gain_desc_t lqr_gain_big;
const lqr_gain_desc_t *LQR_Gain_Info(void);
uint8_t LQR_Gain_Check(const lqr_gain_desc_t *gain, const machine_cfg_t *cfg,
                       uint8_t machine_id, float dt);
uint8_t LQR_Gain_Compatible(void);
uint8_t LQR_Ready(void);
uint8_t LQR_Gain_Eval(float lL, float lR, float K[40], uint8_t legacy);
void LQR_K_WBR(float lL, float lR, float K[40]);

#endif
