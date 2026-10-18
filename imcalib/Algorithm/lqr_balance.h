/*
 * 文件职责：LQR状态构建、遥控目标和状态反馈（lqr_balance.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：IMU、双腿几何、轮速及执行层融合速度/空中控制选择。
 * 输出/去向：形成10维状态/目标；按实测腿长求K，输出双轮和左右腿摆矩。
 * 关键边界：腿长PID由leg_balance负责；AIR只使用腿角/角速度四列、轮矩为零；本文件不发报文。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef LQR_BALANCE_H
#define LQR_BALANCE_H

#include <stdint.h>

#include "rc_types.h"
#include "imu_state.h"
#include "leg_solver.h"
#include "simple-function.h"
#include "lqr_gain_table.h"

/* 状态序 — 与 MATLAB 模型一致 */
enum {
    LQR_X_S = 0,    /* 前进位移 m */
    LQR_X_DS,       /* 前进速度 m/s */
    LQR_X_PHI,      /* 偏航角 rad */
    LQR_X_DPHI,     /* 偏航角速度 rad/s */
    LQR_X_THL,      /* 左腿前摆角 rad */
    LQR_X_DTHL,     /* 左腿摆角速度 rad/s */
    LQR_X_THR,      /* 右腿前摆角 rad */
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

typedef struct {
    float   vel_leg_comp_sign; /* 速度补偿 */
    uint8_t yaw_hold;          /* 偏航角环 */
    uint8_t yaw_rate_hold;     /* 偏航角速度环 */
    uint8_t pos_hold;          /* 位移环 (0 = 位移列不进控制) */
    float   acc_fwd_sign;      /* 前向加速度符号 */
    float   pitch_comp_sign;   /* 轮速补偿俯仰项符号 */
    float   pos_arm_vel;       /* 首次积分速度门槛m/s；0不检查速度 */
    uint8_t wheel_enable;      /* 轮通道 */
    uint8_t hip_enable;        /* 髋通道 */
    uint8_t len_pid_enable;    /* 腿长PID */
    uint8_t air_release_enable; /* 空中释放 */
    float   trq_max_wheel;     /* 轮限幅 */
    float   trq_max_hip;       /* 髋限幅 */
} lqr_runtime_config_t;

typedef lqr_runtime_config_t lqr_debug_t;

/* LQR运行状态：最终长度目标、PID实际给定、实测长度是三个不同概念。 */
typedef struct {
    float x[LQR_X_NUM]; /* 10维状态，顺序由LQR_X枚举固定 */
    float target[LQR_X_NUM]; /* 与x同序的目标，长度目标不在这个数组内 */
    float u[LQR_U_NUM]; /* 双轮及双腿摆矩输出N·m，尚未组合腿长力 */
    float u_col[LQR_X_NUM]; /* 左轮每个状态列的力矩贡献N·m */
    float u_hip_col[LQR_X_NUM]; /* 左虚拟髋每个状态列的力矩贡献N·m */
    float K[LQR_U_NUM][LQR_X_NUM]; /* 增益矩阵，行输出/列状态 */
    float len[2]; /* 左右实测虚拟腿长m，用于增益调度 */
    float len_eval[2]; /* 上次求K使用的腿长m，只作重复计算判据 */
    float whl[2];                   /* 轮对地角速度 (调试) */
    uint8_t gain_valid; /* 本次增益/控制输出是否有效 */
    uint8_t valid;                  /* 状态估计有效 */
    float ds_raw;                   /* 运动学速度 (未滤波) */
    float ds_kf;                    /* 融合速度 */
    float a_fwd;                    /* 前向加速度 m/s² */
    float leg_len_tgt[2]; /* 左右最终长度目标m，原斜坡的终点 */
    float pos;                      /* 位移积分 */
    uint8_t pos_armed;              /* 位置基准已锁 */
    float yaw_tgt;                  /* 偏航角目标 */
    float vel_tgt;                  /* 速度目标 */
    float roll;                     /* 机体横滚角 */
    uint8_t air_control; /* 执行层选择的AIR四列反馈/轮零模式 */
    uint8_t air_release; /* 执行层选择的轴向释放，暂停长度/roll和弹簧抵消 */
    float air_hip_max; /* 空中虚拟腿摆矩限幅N·m */
    lowpass1d_t lpf_omg_pitch;      /* 俯仰角速度低通 */
    lowpass1d_t lpf_omg_yaw;        /* 偏航角速度低通 */
} lqr_state_t;

extern lqr_debug_t lqr_debug;

void    LQR_Init(lqr_state_t *st);
void    LQR_Velocity_Apply(lqr_state_t *st, float velocity, float dt);
uint8_t LQR_Enable_Latch(lqr_state_t *st, const rc_command_t *cmd);
uint8_t LQR_Target_Update(lqr_state_t *st, const rc_command_t *cmd);
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,
                         const leg_state_t *leg_l, const leg_state_t *leg_r,
                         const float wheel_vel[2]);
void    LQR_Control_Update(lqr_state_t *st);

#endif
