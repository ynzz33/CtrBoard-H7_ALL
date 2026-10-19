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
void Control_Output_Publish_Air_Debug(void);

#endif
