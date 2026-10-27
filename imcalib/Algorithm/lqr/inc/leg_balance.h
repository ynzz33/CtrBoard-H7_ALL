/*
 * 文件职责：NORMAL固定四态LQR、非NORMAL原PD及腿部输出组合（leg_balance.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：LQR状态/最终腿长目标、双腿实测几何、显式运行配置和dt。
 * 输出/去向：生成左右轴向力/摆矩，经公共补偿与限幅后填torque_output_t。
 * 关键边界：AIR释放时轴向主动出力为零；非释放时用原轨迹斜坡；只求解，不发送CAN。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
    ramp_t length_ramp[2]; /* 原给定轨迹历史，首次实测播种，随后沿上拍输出 */
    pid_t roll; /* 非NORMAL横滚PD；p为N/rad，d为N·s/rad */
    float turn_ff; /* NORMAL向心补偿N，正值左减右加；最终限矩前请求 */
    float F[2]; /* 含向心项，弹簧/自重补偿及逐电机限幅前的左右轴向请求N */
    float Tp[2]; /* 补偿前左右虚拟腿摆矩N·m */
    torque_output_t cmd; /* 已映射/补偿/限幅的统一力矩结果 */
    compensation_debug_t compensation; /* 弹簧、自重和饱和诊断快照 */
    uint8_t air_release; /* 本模块上一拍释放状态，用于识别进出释放 */
    roll_height_balance_t roll_height; /* NORMAL唯一轴向控制，不再后备PD */
} leg_balance_t;

void    Leg_Balance_Init(leg_balance_t *lb);
void    Leg_Balance_Reset(leg_balance_t *lb);
uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            const leg_compensation_values_t *values,
                            const lqr_runtime_config_t *config,
                            uint8_t normal_mode,
                            float dt, torque_output_t *torque);

#endif
