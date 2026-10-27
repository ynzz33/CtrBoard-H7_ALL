/* 正常LQR路径：1kHz四状态反馈+慢变力偏置观测，无额外腿力限幅。 */
#ifndef ROLL_HEIGHT_BALANCE_H
#define ROLL_HEIGHT_BALANCE_H
#include "roll_height_model.h"
#include "lqr_balance.h"
#include "leg_solver.h"
#include "simple-function.h"

typedef enum { RH_LQR_OK = 0, RH_LQR_BAD_RESULT } roll_height_result_t;
typedef struct {
    float equilibrium[RH_NU]; /* 原支撑前馈+显式向心项-估计偏置，N */
    float feedback[RH_NU][RH_NX]; /* -K*(x-ref)，分项诊断 */
    roll_height_result_t result;
} roll_height_feedback_t;

typedef struct {
    float state[RH_NX]; /* 未经控制低通的物理状态估计 */
    float bias[RH_NU]; /* 左右等效附加支持力；控制时减去该估计 */
    uint8_t permit; /* 上一拍已提交且未限矩，当前双腿估计有支撑 */
    uint8_t seeded;
    uint8_t updated;
    uint8_t limited; /* 偏置估计触及边界，不代表电机限矩 */
} roll_height_observer_t;

typedef struct {
    roll_height_feedback_t lqr;
    roll_height_observer_t observer;
    float x[RH_NX];
    float reference[RH_NX];
    float force[RH_NU];
    lowpass1d_t velocity_filter[2];
    float height_rate_raw;
    float height_goal;
    uint32_t update_count;
    uint32_t elapsed_us;
    uint32_t max_elapsed_us;
    uint8_t permit;
    uint8_t sampled;
    uint8_t valid;
} roll_height_balance_t;

uint8_t Roll_Height_Available(void);
void Roll_Height_Reset(roll_height_balance_t *state);
void Roll_Height_Begin_Tick(roll_height_balance_t *state, uint8_t permit, uint8_t observer_permit);
uint8_t Roll_Height_Update(roll_height_balance_t *state, const lqr_state_t *lqr,
                          const leg_state_t *left, const leg_state_t *right,
                          const float length_target[RH_NU], float turn_ff);
#endif
