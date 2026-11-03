/*
 * 检查候选力矩是否有限、许可和故障是否允许出力，再提交DM髋力矩及DJI轮力矩。
 * 不满足条件时发送零指令；输出数组记录提交的命令值，HAL_OK仅代表驱动提交成功。
 */
#include "control_output.h"
#include "robot_state.h"
#include "latency_state.h"
#include "output_capture.h"
#include "mono_ns.h"
#include "dm.h"
#include "dji.h"
#include "air_detection.h"
#include <math.h>
#include <string.h>

/* 复查候选有限性、基础许可、即时故障及RL阶段，再发送正常力矩或零指令。
 * output_*记录提交命令；output_debug_*_sent仅记录HAL提交结果，不是电机实际执行回执。 */
void Control_Output_Apply(const control_frame_t *frame, ctrl_strategy_t strategy,
                          const torque_output_t *torque,
                          const compensation_debug_t *debug)
{
#if MACHINE_LATENCY_CAPTURE_ENABLE
    output_capture_record_t record={0};
    record.kind=OUTPUT_CAPTURE_CONTROL;record.begin_ns=Mono_Ns_Get();
#endif
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
    zero_output = (uint8_t)(!applied.valid || !frame->drive_permit || ctrl_fault != FAULT_NONE || !torque_output_enabled);
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
        output_dm_torque_nm[i] = applied.dm[i];
    }
    output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT] = applied.dji[DJI_MOTOR_WHEEL_LFT];
    output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT] = applied.dji[DJI_MOTOR_WHEEL_RGT];
    taskEXIT_CRITICAL();

    if (zero_output)
    {
        output_debug_dm_sent = (uint8_t)(Dm_Send_Zero() == HAL_OK);
        output_debug_dji_sent = (uint8_t)(Dji_All_Stop() == HAL_OK);
#if MACHINE_LATENCY_CAPTURE_ENABLE
    record.end_ns=Mono_Ns_Get();record.id=trace_dispatch_sequence;record.fault=ctrl_fault;record.strategy=(uint8_t)strategy;record.machine=MACHINE_DEFAULT;record.wheel_online=frame->trace_wheel_online;
    record.flags=(uint16_t)((zero_output?2u:0u)|(output_debug_dm_sent?4u:0u)|(output_debug_dji_sent?8u:0u)|(torque_output_enabled?16u:0u)|(frame->drive_permit?32u:0u));
    for(uint8_t i=0u;i<4u;i++) { record.torque_nm[i]=applied.dm[i]; }
    for(uint8_t i=0u;i<2u;i++) { record.torque_nm[i+4u]=applied.dji[i];record.wheel_speed[i]=frame->wheel_vel[i];record.wheel_age_us[i]=Latency_Age_Us(record.begin_ns,frame->trace_rx_ns[i+5u]); }
    Output_Capture_Push(&record);
#endif
        return;
    }
    output_debug_dm_sent = (uint8_t)(Dm_Send_Torque(applied.dm) == HAL_OK);
    output_debug_dji_sent = (uint8_t)(Dji_Send_Wheel_Torque(applied.dji[DJI_MOTOR_WHEEL_LFT], applied.dji[DJI_MOTOR_WHEEL_RGT]) == HAL_OK);
#if MACHINE_LATENCY_CAPTURE_ENABLE
    record.end_ns=Mono_Ns_Get();record.id=trace_dispatch_sequence;record.fault=ctrl_fault;record.strategy=(uint8_t)strategy;record.machine=MACHINE_DEFAULT;record.wheel_online=frame->trace_wheel_online;
    record.flags=(uint16_t)((zero_output?2u:0u)|(output_debug_dm_sent?4u:0u)|(output_debug_dji_sent?8u:0u)|(torque_output_enabled?16u:0u)|(frame->drive_permit?32u:0u));
    for(uint8_t i=0u;i<4u;i++) { record.torque_nm[i]=applied.dm[i]; }
    for(uint8_t i=0u;i<2u;i++) { record.torque_nm[i+4u]=applied.dji[i];record.wheel_speed[i]=frame->wheel_vel[i];record.wheel_age_us[i]=Latency_Age_Us(record.begin_ns,frame->trace_rx_ns[i+5u]); }
    Output_Capture_Push(&record);
#endif
}
