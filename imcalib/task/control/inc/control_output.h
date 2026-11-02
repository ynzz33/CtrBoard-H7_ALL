/*
 * 检查候选力矩是否有限、许可和故障是否允许出力，再提交DM髋力矩及DJI轮力矩。
 * 不满足条件时发送零指令；输出数组记录提交的命令值，HAL_OK仅代表驱动提交成功。
 */
/*
 * 执行拍最终输出：只复查许可、更新诊断并发送唯一电机报文。
 * 不更新检测、恢复、PID、积分器或控制器内部状态。
 */
#ifndef CONTROL_OUTPUT_H
#define CONTROL_OUTPUT_H

#include "robot_control.h"
#include "torque_output.h"

void Control_Output_Apply(const control_frame_t *frame, ctrl_strategy_t strategy,
                          const torque_output_t *torque,
                          const compensation_debug_t *debug);

#endif
