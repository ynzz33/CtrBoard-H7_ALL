/*
 * 在短临界区复制控制诊断到浮点通道数组，退出临界区后交给Vofa_Send。
 * 当前为停车位移诊断（46号=5013）；旧5008布局仅供手动切换，无运行时选帧。
 */

#include "vofa_debug.h"
#include "air_detection.h"
#include "robot_state.h"
#include "dm.h"
#include "dji.h"
#include "control_condition.h"
#include "machine_config.h"
#include "Vofa_send.h"
#include "dr16.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

void Vofa_Debug_Send(void)
{
    static float dbg[VOFA_MAX_CH];
    uint8_t online_mask;
    uint16_t state_bits;

    memset(dbg, 0, sizeof(dbg));
    /* ch0 在线掩码 */
    online_mask  = imu_state.online ? 0x01u : 0x00u;
    online_mask |= dr16.online ? 0x02u : 0x00u;
    online_mask |= motor_state.dm.online[0] ? 0x04u : 0x00u;
    online_mask |= motor_state.dm.online[1] ? 0x08u : 0x00u;
    online_mask |= motor_state.dm.online[2] ? 0x10u : 0x00u;
    online_mask |= motor_state.dm.online[3] ? 0x20u : 0x00u;
    online_mask |= motor_state.dji.online[0] ? 0x40u : 0x00u;
    online_mask |= motor_state.dji.online[1] ? 0x80u : 0x00u;
    dbg[0] = (float)online_mask;

    /* ch1 状态位: 使能/跌倒/左腿有效/右腿有效/四髋使能/已投入 */
    state_bits  = robot_state.motor_enabled ? 0x01u : 0x00u;
    state_bits |= robot_state.fallen ? 0x02u : 0x00u;
    state_bits |= leg_l.output.valid ? 0x04u : 0x00u;
    state_bits |= leg_r.output.valid ? 0x08u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_LFT) ? 0x10u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_LFT) ? 0x20u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_RGT) ? 0x40u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_RGT) ? 0x80u : 0x00u;
    state_bits |= output_task_lqr_engaged() ? 0x100u : 0x00u;
    state_bits |= output_task_rl_engaged() ? 0x200u : 0x00u;
    dbg[1] = (float)state_bits;
    dbg[2] = (float)ctrl_fault;

    taskENTER_CRITICAL();
    /* 固定48路：停车门控、状态来源、七组轮矩、电流与最终输出。 */
    dbg[3]=(float)control_condition.condition;
    dbg[4]=(float)((lqr_state.valid ? 4u : 0u) | (lqr_state.gain_valid ? 8u : 0u) | (output_task_lqr_engaged() ? 16u : 0u) | (torque_output_enabled ? 32u : 0u) | (rc_command.mode.engage ? 64u : 0u) | (rc_command.online ? 128u : 0u) | (lqr_state.air_control || lqr_state.air_release ? 256u : 0u) | (output_debug_dm_sent ? 512u : 0u) | (output_debug_dji_sent ? 1024u : 0u) | (leg_balance.compensation.saturated ? 2048u : 0u) | (lqr_state.pos_armed ? 16384u : 0u));
    dbg[5]=(float)lqr_state.pos_trace;
    dbg[6]=rc_command.velocity_target;dbg[7]=rc_command.yaw_rate_target;
    dbg[8]=lqr_state.x[LQR_X_S];dbg[9]=lqr_state.target[LQR_X_S];
    dbg[10]=imu_state.acc_g[2]; /* 机体Z轴加速度，g，含重力 */dbg[11]=lqr_state.ds_kf;
    dbg[12]=lqr_state.ds_raw; /* 运动学估速，m/s */
    /* 13为LQR本拍生效的速度目标，6为原遥控请求。 */
    dbg[13]=lqr_state.target[LQR_X_DS];
    dbg[14]=lqr_state.target[LQR_X_THB]+LQR_Gain_Info()->x_eq[LQR_X_THB]-lqr_state.x[LQR_X_THB];
    dbg[15]=lqr_state.x[LQR_X_DTHB];
    dbg[16]=lqr_state.target[LQR_X_THL]+LQR_Gain_Info()->x_eq[LQR_X_THL]-lqr_state.x[LQR_X_THL];
    dbg[17]=lqr_state.x[LQR_X_DTHL];
    dbg[18]=lqr_state.target[LQR_X_THR]+LQR_Gain_Info()->x_eq[LQR_X_THR]-lqr_state.x[LQR_X_THR];
    dbg[19]=lqr_state.x[LQR_X_DTHR];
    dbg[20]=motor_state.dji.vel_rad_s[0];dbg[21]=motor_state.dji.vel_rad_s[1];
    dbg[22]=lqr_state.a_fwd; /* 水平前向加速度，m/s^2 */dbg[23]=lqr_state.wheel_unclamped[1];
    /* 24~30左轮、31~37右轮：位置/速度/偏航/腿角/腿角速/pitch/pitch角速。 */
    for (uint8_t j=0u;j<LQR_WHEEL_TERM_NUM;j++)
    {
        dbg[24u+j]=lqr_state.wheel_term[0][j];
        dbg[31u+j]=lqr_state.wheel_term[1][j];
    }
    /* 命令按反馈ID取CAN槽，再用out符号还原控制坐标；反馈已乘fb，不能再翻一次。 */
    dbg[38]=(float)wheel_current[dji_motor_config[DJI_MOTOR_WHEEL_LFT].feedback_id-0x201u]*(float)machine->wheel.dji_sign[DJI_MOTOR_WHEEL_LFT].out;
    dbg[39]=(float)wheel_current[dji_motor_config[DJI_MOTOR_WHEEL_RGT].feedback_id-0x201u]*(float)machine->wheel.dji_sign[DJI_MOTOR_WHEEL_RGT].out;
    dbg[40]=(float)motor_state.dji.current_raw[DJI_MOTOR_WHEEL_LFT];
    dbg[41]=(float)motor_state.dji.current_raw[DJI_MOTOR_WHEEL_RGT];
    dbg[42]=output_wheel_torque_nm[0];dbg[43]=output_wheel_torque_nm[1];
    dbg[44]=lqr_state.len[0];dbg[45]=lqr_state.len[1];
    dbg[46]=5013.0f;dbg[47]=(float)HAL_GetTick();
    taskEXIT_CRITICAL();
    Vofa_Send(dbg, VOFA_MAX_CH);
}
