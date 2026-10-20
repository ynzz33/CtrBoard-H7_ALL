/*
 * 文件职责：遥控死区和统一归一化指令（rc_command.c）。
 * 输入/来源：DR16原始通道、拨轮、拨杆和在线标志。
 * 输出/去向：commTask每拍生成唯一rc_command，供执行和策略读取。
 * 关键边界：模式请求每拍解码一次，执行层检查出力许可；len是公共腿长目标。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "rc_command.h"
#include "dr16.h"
#include "machine_config.h"
#include "lqr_gain_table.h"
#include "gas_spring.h"
#include <math.h>
#include <stddef.h>

/* 已验证通道归一化 */
static float Rc_Axis(int16_t raw, uint16_t deadband)
{
    int16_t value;

    value = DR16_Deadline(raw, deadband);
    return (float)value / (float)DR16_CH_LIMIT;
}

/* 公共指令换算与唯一模式解码；离线清除模式请求，拨轮沿用原累加公式。 */
void Rc_Command_Update(rc_command_t *cmd, const dr16_t *rc, float dt)
{
    float len_min;
    float len_max;

    cmd->mode.strategy = CTRL_STRATEGY_DISABLE;
    cmd->mode.rc_enable = 0u;
    cmd->mode.engage = 0u;
    if (!rc->online)
    {
        cmd->vel = 0.0f;
        cmd->yaw = 0.0f;
        cmd->len = machine->rc.len_init;
        cmd->s1 = 0u;
        cmd->s2 = 0u;
        cmd->online = 0u;
        return;
    }
    cmd->vel = Rc_Axis(rc->ch1, RC_DEADBAND_VEL) * machine->rc.vel_max;
    cmd->yaw = -Rc_Axis(rc->ch2, RC_DEADBAND_YAW) * machine->rc.yaw_max;
    len_min = fmaxf(machine->leg_len_min, LQR_Gain_Info()->len_min);
    len_max = fminf(machine->leg_len_max, LQR_Gain_Info()->len_max);
    cmd->len += Rc_Axis(rc->wheel, RC_DEADBAND_LEN) * machine->rc.len_rate * dt*0.3;
    cmd->len = fminf(fmaxf(cmd->len, len_min), len_max);
    cmd->s1 = rc->s1;
    cmd->s2 = rc->s2;
    cmd->online = 1u;

    /* 拨杆语义 */
    switch (cmd->s1)
    {
    case DR16_SW_MID:
        if (cmd->s2 == DR16_SW_UP)
        {
            cmd->mode.rc_enable = (uint8_t)(GAS_SPRING_COMP_ENABLE && machine->spring != NULL && machine->gravity != NULL);
            cmd->mode.strategy = cmd->mode.rc_enable ? CTRL_STRATEGY_COMPENSATION : CTRL_STRATEGY_DISABLE;
            cmd->mode.engage = 1u;
        }
        else
        {
            cmd->mode.strategy = CTRL_STRATEGY_LQR;
            cmd->mode.rc_enable = machine->lqr_configured;
            cmd->mode.engage = (uint8_t)(cmd->s2 == DR16_SW_MID);
        }
        break;

    case DR16_SW_UP: cmd->mode.strategy = CTRL_STRATEGY_RL;
        cmd->mode.rc_enable = machine->rl.configured ? 1u : 0u;
        cmd->mode.engage = (uint8_t)(cmd->s2 == DR16_SW_MID);
        break;

    case DR16_SW_DOWN:
    default:
        break;
    }
}
