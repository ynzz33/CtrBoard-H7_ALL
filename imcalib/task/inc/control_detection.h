/*
 * 执行层检测调度：消费同拍快照，推进公共离地观测和三态判断。
 * 不生成控制目标/力矩，不复位自起动作；返回进入FALLEN的事件供执行层消费。
 */
#ifndef CONTROL_DETECTION_H
#define CONTROL_DETECTION_H

#include "robot_control.h"

uint8_t Control_Detection_Update(control_frame_t *frame);

#endif
