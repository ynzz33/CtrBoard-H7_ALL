/*
 * 文件职责：遥控死区和统一归一化指令（rc_command.c）。
 * 输入/来源：DR16原始通道、拨轮、拨杆和在线标志。
 * 输出/去向：commTask每拍生成唯一rc_command，供执行和策略读取。
 * 关键边界：这里只处理通道量；拨杆模式语义在task_actuation；长度通道表示调节输入，不是实测腿长。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "rc_command.h"
#include "dr16.h"
#include "machine_config.h"

/* 死区 + 限幅 → [-1,1] */
static float Rc_Axis(int16_t raw, uint16_t deadband)
{
    int16_t value;

    value = DR16_Deadline(raw, deadband);
    if (value > DR16_CH_LIMIT)
    {
        value = DR16_CH_LIMIT;
    }
    else if (value < -DR16_CH_LIMIT)
    {
        value = -DR16_CH_LIMIT;
    }
    return (float)value / (float)DR16_CH_LIMIT;
}

/* DR16 快照 → 指令; 离线全零 */
void Rc_Command_Update(rc_command_t *cmd, const dr16_t *rc)
{
    if (!rc->online)
    {
        cmd->vel = 0.0f;
        cmd->yaw = 0.0f;
        cmd->len = 0.0f;
        cmd->s1 = 0u;
        cmd->s2 = 0u;
        cmd->online = 0u;
        return;
    }
    cmd->vel = Rc_Axis(rc->ch1, RC_DEADBAND_VEL) * machine->lqr.vel_max;
    cmd->yaw = -Rc_Axis(rc->ch2, RC_DEADBAND_YAW) * machine->lqr.yaw_max;
    cmd->len = Rc_Axis(rc->wheel, RC_DEADBAND_LEN);
    cmd->s1 = rc->s1;
    cmd->s2 = rc->s2;
    cmd->online = 1u;
}
