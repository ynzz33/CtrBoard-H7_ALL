/*
 * 执行拍策略准备：更新本拍恢复、模型缓存和公共检测，只确认控制路径。
 * 不推进LQR/RL控制器，不生成力矩，也不发送电机报文。
 */
#ifndef CONTROL_STRATEGY_H
#define CONTROL_STRATEGY_H

#include "robot_control.h"

ctrl_strategy_t Control_Strategy_Prepare(control_frame_t *frame,
                                         uint8_t lqr_running,
                                         uint8_t *entered_fallen);

#endif
