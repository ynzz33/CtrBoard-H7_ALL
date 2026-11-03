/* 更新RL状态和公共选路，再按本拍输出路径准备补偿。
 * 正常使用欧拉俯仰，台阶/自起/补偿测试使用pitch_world；输出只保存一份重力结果。
 * AIR独立估力，不依赖该输出缓存。 */
#ifndef CONTROL_STRATEGY_H
#define CONTROL_STRATEGY_H

#include "robot_control.h"

/* 自起在同拍交接到平衡时，按新状态重新选择输出补偿。 */
void Control_Model_Prepare(control_frame_t *frame);

ctrl_strategy_t Control_Strategy_Prepare(control_frame_t *frame,
                                         uint8_t *entered_fallen);

#endif
