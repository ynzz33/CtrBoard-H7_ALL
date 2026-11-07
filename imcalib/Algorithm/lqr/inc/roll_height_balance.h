/*
 * 用平均腿长/长度速度、横滚角/角速度四个状态生成NORMAL左右轴向力。
 * 可选偏置观察器只使用上一拍确认提交且未饱和的力请求；本模块不发送电机命令。
 */
#ifndef ROLL_HEIGHT_BALANCE_H
#define ROLL_HEIGHT_BALANCE_H
#include "roll_height_model.h"
#include "roll_height_mpc.h"
#include "lqr_balance.h"
#include "leg_solver.h"
#include "simple-function.h"

typedef struct {
    float base_force[RH_NU]; /* 原支撑前馈+显式向心项-估计偏置，N */
    float height_trim; /* 静态共模前馈试调N/腿，不是观测偏置 */
    float common_shift; /* 为满足两腿下限而增加的平均力N，未触限为0 */
} roll_height_feedback_t;

typedef struct {
    float state[RH_NX]; /* 未经控制低通的物理状态估计 */
    float bias[RH_NU]; /* 左右等效附加支持力；控制时减去该估计 */
    uint8_t permit; /* 上一拍已提交且未限矩，当前双腿估计有支撑 */
    uint8_t seeded;
} roll_height_observer_t;

typedef struct roll_height_balance {
    roll_height_mpc_t mpc; /* 本控制实例的静态求解工作区和诊断。 */
    roll_height_feedback_t lqr;
    roll_height_observer_t observer;
    float x[RH_NX];
    float target[RH_NX]; /* 本拍四态目标；高度项由腿长斜坡投影得到 */
    float force[RH_NU];
    lpf_t lpf_omg_roll; /* roll角速度低通 */
    lpf_t lpf_height_rate; /* 高度变化速度低通 */
    float height_rate_raw;
    uint8_t permit;
    uint8_t sampled;
    uint8_t valid; /* 本拍四态输出是否可用；复位或计算异常为0 */
} roll_height_balance_t;

uint8_t Roll_Height_Available(void);
void Roll_Height_Reset(roll_height_balance_t *state);
void Roll_Height_Begin_Tick(roll_height_balance_t *state, uint8_t permit, uint8_t observer_permit,
    uint8_t output_applied); /* MPC保持许可与观察器支持力许可分开。 */
uint8_t Roll_Height_Update(roll_height_balance_t *state, const lqr_state_t *lqr,
                          const leg_state_t *left, const leg_state_t *right,
                          const float length_ramp_target[RH_NU], const float length_rate_target[RH_NU], float turn_ff, const leg_compensation_values_t *values, const lqr_runtime_config_t *config);
#endif
