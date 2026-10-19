/*
 * 文件职责：唯一最终输出口。
 * 复查候选力矩、当前许可和RL恢复状态后发送DM/DJI；不参与控制计算。
 */
#include "control_output.h"
#include "robot_state.h"
#include "dm.h"
#include "dji.h"
#include "air_detection.h"
#include <math.h>
#include <string.h>

void Control_Output_Apply(const control_frame_t *frame, ctrl_strategy_t strategy,
                          const torque_output_t *torque,
                          const compensation_debug_t *debug)
{
    torque_output_t applied;
    compensation_debug_t applied_debug;
    uint8_t zero_output;

    applied = *torque;
    applied_debug = *debug;
    applied_debug.bench = (uint8_t)(strategy == CTRL_STRATEGY_COMPENSATION);
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!isfinite(applied.dm[i]))
        {
            applied.valid = 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!isfinite(applied.dji[i]))
        {
            applied.valid = 0u;
        }
    }
    zero_output = (uint8_t)(!applied.valid || !frame->drive || ctrl_fault != FAULT_NONE || !torque_output_enabled);
    if (strategy == CTRL_STRATEGY_RL && rl_control.recovery.phase != RL_RECOVERY_NORMAL && rl_control.recovery.phase != RL_RECOVERY_RECOVER)
    {
        zero_output = 1u;
    }
    if (zero_output)
    {
        memset(&applied, 0, sizeof(applied));
        memset(&applied_debug, 0, sizeof(applied_debug));
        applied_debug.bench = (uint8_t)(strategy == CTRL_STRATEGY_COMPENSATION);
    }
    taskENTER_CRITICAL();
    compensation_debug = applied_debug;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        rl_output_dm_cmd_nm[i] = applied.dm[i];
    }
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_LFT] = applied.dji[DJI_MOTOR_WHEEL_LFT];
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_RGT] = applied.dji[DJI_MOTOR_WHEEL_RGT];
    taskEXIT_CRITICAL();

    if (zero_output)
    {
        output_debug_dm_sent = (uint8_t)(Dm_Send_Zero() == HAL_OK);
        output_debug_dji_sent = (uint8_t)(Dji_All_Stop() == HAL_OK);
        return;
    }
    output_debug_dm_sent = (uint8_t)(Dm_Send_Torque(applied.dm) == HAL_OK);
    output_debug_dji_sent = (uint8_t)(Dji_Send_Wheel_Torque(applied.dji[DJI_MOTOR_WHEEL_LFT], applied.dji[DJI_MOTOR_WHEEL_RGT]) == HAL_OK);
}

void Control_Output_Publish_Air_Debug(void)
{
    taskENTER_CRITICAL();
    air_debug = air_detection;
    taskEXIT_CRITICAL();
}
