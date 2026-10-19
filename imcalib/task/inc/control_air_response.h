/*
 * 执行层AIR后处理：消费已发布判断，沿用原目标回置与控制选择。
 * 不推进检测、不求解力矩、不发送CAN；在原两个响应位置调用。
 */
#ifndef CONTROL_AIR_RESPONSE_H
#define CONTROL_AIR_RESPONSE_H

#include "robot_control.h"

void Control_Air_Response_Update(control_frame_t *frame);

#endif
