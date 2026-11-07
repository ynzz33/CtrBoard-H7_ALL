/*
 * 生成腿长轨迹，NORMAL用四态腿长/横滚反馈求轴向力，其他路径按条件使用PD或释放。
 * 组合轴向力、十态LQR摆矩和轮矩，映射四髋后加入选定补偿并限矩，填候选输出。
 */

#ifndef LEG_BALANCE_H
#define LEG_BALANCE_H

#include <stdint.h>

#include "leg_solver.h"
#include "lqr_balance.h"
#include "pid.h"
#include "simple-function.h"
#include "torque_output.h"
#include "leg_compensation.h"
#include "roll_height_balance.h"

/* NORMAL固定四态；PD容器仅供非NORMAL，恢复动作另持有独立实例。 */
typedef struct {
    pid_t leg_len[2]; /* 非NORMAL长度PD；p为N/m，d为N·s/m，输出为N */
    ramp_t length_ramp[2]; /* NORMAL成对参考位置，非NORMAL仍使用原斜坡 */
    float length_ref_rate[2]; /* NORMAL长度参考速度m/s，与参考位置成对生成 */
    uint8_t length_ref_ready;
    pid_t roll; /* 非NORMAL横滚PD；p为N/rad，d为N·s/rad */
    float turn_ff; /* NORMAL向心补偿N，正值左减右加；最终限矩前请求 */
    float F[2]; /* 含向心项，弹簧/自重补偿及逐电机限幅前的左右轴向请求N */
    float Tp[2]; /* 补偿前左右虚拟腿摆矩N·m */
    torque_output_t cmd; /* 已映射/补偿/限幅的统一力矩结果 */
    compensation_debug_t compensation; /* 弹簧、自重和饱和诊断快照 */
    uint8_t air_release; /* 本模块上一拍释放状态，用于识别进出释放 */
    roll_height_balance_t roll_height; /* NORMAL唯一轴向控制，不再后备PD */
} leg_balance_t;

void Leg_Balance_Reference_Preview(leg_balance_t *lb, const lqr_state_t *st); /* 仅写MPC预览缓冲。 */
void    Leg_Balance_Init(leg_balance_t *lb);
void    Leg_Balance_Reset(leg_balance_t *lb);
/* 只重组当拍已有F与已选十态输入，不推进任何控制/参考/模型历史。 */
uint8_t Leg_Balance_Apply_Inputs(leg_balance_t *lb,const leg_state_t *left,const leg_state_t *right,
    const float F[2],const float input[4],const leg_compensation_values_t *values,
    const lqr_runtime_config_t *config,torque_output_t *torque);
uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            const leg_compensation_values_t *values,
                            const lqr_runtime_config_t *config,
                            uint8_t normal_mode,
                            float dt, torque_output_t *torque);

#endif
