/*
 * 执行层AIR后处理：消费已发布判断，沿用原目标回置与控制选择。
 * 不推进检测、不求解力矩、不发送CAN；在原两个响应位置调用。
 */
#ifndef AIR_PROCESS_H
#define AIR_PROCESS_H

#include "robot_control.h"

void Air_Process_Update(control_frame_t *frame);

#endif
