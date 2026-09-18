#ifndef LQR_BALANCE_H
#define LQR_BALANCE_H

#include <stdint.h>

#include "dr16.h"
#include "imu_state.h"
#include "leg_solver.h"
#include "lowpass.h"

/* 状态序 — 与 MATLAB 模型一致 */
enum {
    LQR_X_S = 0,    /* 前进位移 m */
    LQR_X_DS,       /* 前进速度 m/s */
    LQR_X_PHI,      /* 偏航角 rad (不参与控制) */
    LQR_X_DPHI,     /* 偏航角速度 rad/s */
    LQR_X_THL,      /* 左腿摆角-世界系 rad */
    LQR_X_DTHL,     /* 左腿摆角速度 rad/s */
    LQR_X_THR,      /* 右腿摆角-世界系 rad */
    LQR_X_DTHR,     /* 右腿摆角速度 rad/s */
    LQR_X_THB,      /* 机体俯仰角 rad */
    LQR_X_DTHB,     /* 机体俯仰角速度 rad/s */
    LQR_X_NUM,
};

/* 输出序 */
enum {
    LQR_U_WL = 0,   /* 左轮扭矩 N·m */
    LQR_U_WR,       /* 右轮扭矩 N·m */
    LQR_U_BL,       /* 左髋虚拟扭矩 N·m */
    LQR_U_BR,       /* 右髋虚拟扭矩 N·m */
    LQR_U_NUM,
};

/* 腿长工作区间改由机器配置表提供: machine->leg_len_min / leg_len_max */

/* 遥控量程 */
#define LQR_RC_DEADBAND     20
#define LQR_RC_VEL_MAX      1.2f    /* m/s */
#define LQR_RC_YAW_MAX      3.0f    /* rad/s */
#define LQR_RC_LEN_RATE     0.3f    /* m/s */

/* 输出限幅 (N·m), 首次上电保守值 */
#define LQR_WHEEL_TRQ_MAX   1.5f
#define LQR_HIP_TRQ_MAX     2.0f

typedef struct {
    float x[LQR_X_NUM];             /* 状态 */
    float target[LQR_X_NUM];        /* 目标 */
    float u[LQR_U_NUM];             /* LQR 输出 */
    float K[LQR_U_NUM][LQR_X_NUM];  /* 增益 */
    float len[2];                   /* 实测腿长 */
    float len_eval[2];              /* 上次增益求值腿长 */
    float leg_len_tgt[2];           /* 腿长目标 */
    float pos;                      /* 位移积分 */
    float roll;                     /* 机体横滚角 */
    lowpass1d_t lpf_vel;            /* 速度低通 */
    lowpass1d_t lpf_omg_pitch;      /* 俯仰角速度低通 */
    lowpass1d_t lpf_omg_yaw;        /* 偏航角速度低通 */
} lqr_state_t;

void    LQR_Init(lqr_state_t *st);
uint8_t LQR_Enable_Latch(lqr_state_t *st, const leg_state_t *leg_l,
                         const leg_state_t *leg_r);
uint8_t LQR_Target_Update(lqr_state_t *st, const dr16_t *rc, float dt);
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,
                         const leg_state_t *leg_l, const leg_state_t *leg_r,
                         const float wheel_vel[2], float dt);
void    LQR_Control_Update(lqr_state_t *st);

#endif
