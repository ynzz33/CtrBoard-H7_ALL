#ifndef RC_COMMAND_H
#define RC_COMMAND_H

#include "rc_types.h"

/* 摇杆死区 (raw) */
#define RC_DEADBAND_VEL     10u
#define RC_DEADBAND_YAW     20u
#define RC_DEADBAND_LEN     20u

/* 遥控指令: 速度 + 转向 + 腿长 + 拨杆 */


void Rc_Command_Update(rc_command_t *cmd, const dr16_t *rc);

#endif
