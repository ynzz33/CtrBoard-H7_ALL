/*
 * 将DR16通道和拨杆转换为策略选择、使能/投入，以及速度、偏航速率和腿长目标。
 * 摇杆死区、比例及腿长变化率取机器配置；该层生成目标，不发送电机命令。
 */

#ifndef RC_COMMAND_H
#define RC_COMMAND_H

#include "rc_types.h"

/* 摇杆死区 (raw) */

/* 遥控指令: 速度 + 转向 + 腿长 + 拨杆 */


void Rc_Command_Update(rc_command_t *cmd, const dr16_t *rc, float dt);

#endif
