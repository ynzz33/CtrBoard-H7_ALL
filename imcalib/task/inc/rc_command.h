/*
 * 文件职责：遥控死区和统一归一化指令（rc_command.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：DR16原始通道、拨轮、拨杆和在线标志。
 * 输出/去向：commTask每拍生成唯一rc_command，供执行和策略读取。
 * 关键边界：这里只处理通道量；拨杆模式语义在task_actuation；长度通道表示调节输入，不是实测腿长。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

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
