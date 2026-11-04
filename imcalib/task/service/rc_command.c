/*
 * 将DR16通道和拨杆转换为策略选择、使能/投入，以及速度、偏航速率和腿长目标。
 * 摇杆死区、比例及腿长变化率取机器配置；该层生成目标，不发送电机命令。
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
        cmd->velocity_target = 0.0f;
        cmd->yaw_rate_target = 0.0f;
        cmd->leg_length_target = machine->rc.len_init;
        cmd->s1 = 0u;
        cmd->s2 = 0u;
        cmd->online = 0u;
        return;
    }
    cmd->velocity_target = Rc_Axis(rc->ch1, machine->rc.deadband_vel) * machine->rc.vel_max;
    cmd->yaw_rate_target = -Rc_Axis(rc->ch2, machine->rc.deadband_yaw) * machine->rc.yaw_max;
    cmd->s1 = rc->s1;
    cmd->s2 = rc->s2;
#if MACHINE_LATENCY_CAPTURE_ENABLE
    cmd->received_ns = rc->received_ns;
#endif
    cmd->online = 1u;

    /* 拨杆语义 */
    switch (cmd->s1)
    {
    case DR16_SW_MID:
        if (cmd->s2 == DR16_SW_UP)
        {
            cmd->mode.rc_enable = (uint8_t)(GAS_SPRING_COMP_ENABLE && machine->compensation.spring != NULL && machine->compensation.gravity != NULL);
            cmd->mode.strategy = cmd->mode.rc_enable ? CTRL_STRATEGY_COMPENSATION : CTRL_STRATEGY_DISABLE;
            cmd->mode.engage = 1u;
        }
        else
        {
            cmd->mode.strategy = CTRL_STRATEGY_LQR;
            cmd->mode.rc_enable = 1u; /* LQR表由编译机型绑定，反馈/出力许可仍在执行层检查。 */
            cmd->mode.engage = (uint8_t)(cmd->s2 == DR16_SW_MID);
        }
        break;

    case DR16_SW_UP: cmd->mode.strategy = CTRL_STRATEGY_RL;
        cmd->mode.rc_enable = 1u;
        cmd->mode.engage = (uint8_t)(cmd->s2 == DR16_SW_MID);
        break;

    case DR16_SW_DOWN:
    default:
        break;
    }
    len_min = MACHINE_BIG_LEG_LEN_MIN;
    /* 公共拨轮保持本机腿长范围；RL入口仅加训练机高偏置。 */
    len_max = MACHINE_BIG_LEG_LEN_MAX;
    cmd->leg_length_target += Rc_Axis(rc->wheel, machine->rc.deadband_len) * machine->rc.len_rate * dt*0.3;
    cmd->leg_length_target = fminf(fmaxf(cmd->leg_length_target, len_min), len_max);
}
