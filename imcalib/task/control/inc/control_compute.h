/*
 * 把执行拍输入转换为候选电机力矩：补偿测试、LQR平衡/自起/AIR或RL动作执行。
 * 管理控制器投入历史和RL分频；torque有效性与dispatch分别表示候选可用和本拍需要发送。
 */
/*
 * 执行拍控制运算：台阶接管先于普通状态更新，只推进被选中的控制路径并生成候选力矩。
 * 不读取新反馈、不重新做检测、不发送电机报文。
 */
#ifndef CONTROL_COMPUTE_H
#define CONTROL_COMPUTE_H

#include "robot_control.h"
#include "torque_output.h"

void Control_Compute_Run(control_frame_t *frame, ctrl_strategy_t strategy,
                         uint8_t entered_fallen, torque_output_t *torque,
                         uint8_t *dispatch);
uint8_t Control_Compute_LQR_Engaged(void);
uint8_t Control_Compute_RL_Engaged(void);

#endif
