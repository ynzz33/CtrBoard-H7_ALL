/*
 * 在短临界区复制控制诊断到浮点通道数组，退出临界区后交给Vofa_Send。
 * 有效通道赋值以当前编译分支为准；注释掉的旧布局保留作人工切换参考。
 */

#include "vofa_debug.h"
#include "robot_state.h"
#include "dm.h"
#include "air_detection.h"
#include "control_detection.h"
#include "control_condition.h"
#include "motion_guard.h"
#include "fallen_process.h"
#include "fallen_detection.h"
#include "math_utils.h"
#include "machine_config.h"
#include "stair_detection.h"
#include "stair_process.h"
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
    // float turn_scale;
    // float contribution;
    // uint8_t row;
    // uint8_t col;
    // const lqr_gain_desc_t *gain;
    // float term;
    // uint8_t group;

    memset(dbg, 0, sizeof(dbg));
    /* ch0 在线掩码 */
    online_mask  = imu_state.online ? 0x01u : 0x00u;
    online_mask |= DR16_Online() ? 0x02u : 0x00u;
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

    /* 旧腿部通道，保留为手动改布局时的参考。 */
    // dbg[3] = (float)leg_l.input.hip_f;
    // dbg[4] = (float)leg_l.input.hip_b;
    // dbg[5] = (float)leg_r.input.hip_f;
    // dbg[6] = (float)leg_r.input.hip_b;
    // dbg[7] = (float)leg_l.output.virtual_leg_length;
    // dbg[8] = (float)leg_l.output.virtual_leg_angle;
    // dbg[9] = (float)leg_l.output.virtual_shank_angle;
    // dbg[10] = (float)leg_r.output.virtual_leg_length;
    // dbg[11] = (float)leg_r.output.virtual_leg_angle;
    // dbg[12] = (float)leg_r.output.virtual_shank_angle;
    // dbg[13] = (float)leg_l.output.d_virtual_leg_length;
    // dbg[14] = (float)leg_l.output.d_virtual_leg_angle;
    // dbg[15] = (float)leg_l.output.d_virtual_shank_angle;
    // dbg[16] = (float)leg_r.output.d_virtual_leg_length;
    // dbg[17] = (float)leg_r.output.d_virtual_leg_angle;
    // dbg[18] = (float)leg_r.output.d_virtual_shank_angle;

//     /* 离地—落地交接诊断布局：只读执行拍已发布状态，不增加控制采样或发布接口。 */
//     taskENTER_CRITICAL();
//     dbg[3] = air_detection.mean_force;
//     dbg[4] = (float)air_detection.flight;
//     dbg[5] = (float)air_detection.valid;
//     dbg[6] = (float)air_detection.initialized;
//     dbg[7] = (float)air_detection.fault;
//     dbg[8] = air_detection.elapsed;
//     dbg[9] = air_detection.initial_unload_elapsed;
//     dbg[10] = air_detection.landing[0].compression;
//     dbg[11] = air_detection.landing[1].compression;
//     /* 落地腿长闭环：实际速度、显式P/D、总PD与roll项用于区分给定和出力。 */
//     dbg[12] = leg_l.output.d_virtual_leg_length;
//     dbg[13] = leg_r.output.d_virtual_leg_length;
//     dbg[14] = leg_balance.leg_len[0].pout;
//     dbg[15] = leg_balance.leg_len[1].pout;
//     dbg[16] = leg_balance.leg_len[0].dout;
//     dbg[17] = leg_balance.leg_len[1].dout;
//     dbg[18] = leg_balance.leg_len[0].position_mode_output;
//     dbg[19] = leg_balance.leg_len[1].position_mode_output;
//     dbg[20] = leg_balance.roll.pout;
//     dbg[21] = leg_balance.roll.position_mode_output;
//     dbg[22] = (float)control_condition.condition;
//     dbg[23] = (float)control_condition.valid;
//     dbg[24] = (float)control_condition.started;
//     dbg[25] = (float)lqr_state.air_control;
//     dbg[26] = (float)lqr_state.air_release;
//     dbg[27] = lqr_state.x[LQR_X_THB];
//     dbg[28] = lqr_state.x[LQR_X_DTHB];
//     dbg[29] = lqr_state.ds_kf;
//     dbg[30] = lqr_state.pos;
//     dbg[31] = output_dm_torque_nm[DM_MOTOR_LEG_F_LFT];
//     dbg[32] = output_dm_torque_nm[DM_MOTOR_LEG_B_LFT];
//     dbg[33] = output_dm_torque_nm[DM_MOTOR_LEG_F_RGT];
//     dbg[34] = output_dm_torque_nm[DM_MOTOR_LEG_B_RGT];
//     dbg[35] = leg_l.output.virtual_leg_length;
//     dbg[36] = leg_r.output.virtual_leg_length;
//     dbg[37] = leg_balance.length_ramp[0].out;
//     dbg[38] = leg_balance.length_ramp[1].out;
//     dbg[39] = leg_balance.F[0];
//     dbg[40] = leg_balance.F[1];
//     dbg[41] = leg_balance.Tp[0];
//     dbg[42] = leg_balance.Tp[1];
//     dbg[43] = lqr_state.leg_length_target[0];
//     dbg[44] = lqr_state.leg_length_target[1];
//     dbg[45] = (float)(air_detection.flight && air_param.compression_enabled && (air_detection.landing[0].elapsed > 0.0f || air_detection.landing[1].elapsed > 0.0f));
//     dbg[46] = (float)leg_balance.air_release;
//     dbg[47] = (float)robot_state.fallen;
//     taskEXIT_CRITICAL();
//     // dbg[23] = lqr_state.a_fwd;
//
//
//     /* LQR排查布局：固定48路；40～43用于yaw实际/目标，其余保留四态诊断。 */
//     taskENTER_CRITICAL();
//     dbg[3] = (float)control_condition.condition;
//     dbg[4] = (float)control_condition.valid;
//     dbg[5] = (float)Roll_Height_Available();
//     dbg[6] = (float)leg_balance.roll_height.valid;
//     dbg[7] = (float)leg_balance.roll_height.update_count;
//     dbg[8] = (float)((air_detection.valid ? 0x01u : 0u) | (air_detection.initialized ? 0x02u : 0u) | (air_detection.fault ? 0x04u : 0u) | (air_detection.flight ? 0x08u : 0u) | (leg_balance.roll_height.observer.seeded ? 0x10u : 0u) | (leg_balance.roll_height.observer.updated ? 0x20u : 0u) | (leg_balance.roll_height.observer.limited ? 0x40u : 0u));
//     /* ch9保留为0（数组入口已清零）；四态结果可用性统一看ch6的valid。 */
//     dbg[10] = leg_balance.roll_height.x[2] - leg_balance.roll_height.target[2];
//     dbg[11] = air_detection.leg[0].force; /* 左FN估计N；由反馈力矩、弹簧和轮重动态项生成 */
//     dbg[12] = air_detection.leg[1].force; /* 右FN估计N；须结合ch8的valid位读取 */
//     dbg[13] = lqr_state.roll;
//     dbg[14] = leg_balance.roll_height.x[0];
//     dbg[15] = leg_balance.roll_height.x[1];
//     dbg[16] = leg_balance.roll_height.x[2];
//     dbg[17] = leg_balance.roll_height.target[2];
//     dbg[18] = leg_balance.roll_height.x[3];
//     dbg[19] = machine->lqr.filter.lpf_alpha[3];
//     dbg[20] = leg_l.output.virtual_leg_length;
//     dbg[21] = leg_r.output.virtual_leg_length;
//     dbg[22] = lqr_state.leg_length_target[0];
//     dbg[23] = lqr_state.leg_length_target[1];
//     dbg[24] = leg_balance.roll_height.observer.bias[0]; /* 左等效附加力估计；控制中减去 */
//     dbg[25] = leg_balance.roll_height.observer.bias[1]; /* 右等效附加力估计；控制中减去 */
//     dbg[26] = leg_balance.F[0];
//     dbg[27] = leg_balance.F[1];
//     dbg[28] = 0.5f * (leg_balance.roll_height.force[0] - leg_balance.roll_height.lqr.base_force[0] + leg_balance.roll_height.force[1] - leg_balance.roll_height.lqr.base_force[1]);
//     dbg[29] = 0.5f * (leg_balance.F[0] + leg_balance.F[1]);
//     dbg[30] = 0.5f * (leg_balance.roll_height.force[0] - leg_balance.roll_height.lqr.base_force[0] - leg_balance.roll_height.force[1] + leg_balance.roll_height.lqr.base_force[1]);
//     dbg[31] = 0.5f * (leg_balance.F[0] - leg_balance.F[1]);
//     dbg[32] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][0] - leg_balance.roll_height.lqr.feedback[1][0]);
//     dbg[33] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][1] - leg_balance.roll_height.lqr.feedback[1][1]);
//     dbg[34] = leg_balance.Tp[0];
//     dbg[35] = leg_balance.Tp[1];
//     dbg[36] = (float)leg_balance.compensation.saturated;
//     dbg[37] = 0.5f * ((leg_l.output.virtual_leg_length - leg_balance.length_ramp[0].out) - (leg_r.output.virtual_leg_length - leg_balance.length_ramp[1].out));
//     dbg[38] = (float)lqr_state.air_control;
//     dbg[39] = (float)lqr_state.air_release;
//     /* 原40～43四态诊断，保留供手动恢复布局。 */
//     // dbg[40] = leg_balance.roll_height.height_target;
//     // dbg[41] = leg_balance.roll_height.height_rate_raw;
//     // dbg[42] = 0.5f * (leg_balance.length_ramp[0].out + leg_balance.length_ramp[1].out);
//     // dbg[43] = 0.5f * (leg_balance.roll_height.lqr.base_force[0] + leg_balance.roll_height.lqr.base_force[1]);
//     dbg[40] = lqr_state.x[LQR_X_PHI]; /* yaw实际角度rad */
//     dbg[41] = lqr_state.target[LQR_X_PHI] + LQR_Gain_Info()->x_eq[LQR_X_PHI]; /* yaw目标角度rad */
//     dbg[42] = lqr_state.x[LQR_X_DPHI]; /* yaw实际角速度rad/s，LQR使用的滤后反馈 */
//     dbg[43] = lqr_state.target[LQR_X_DPHI] + LQR_Gain_Info()->x_eq[LQR_X_DPHI]; /* yaw目标角速度rad/s */
//     dbg[44] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][2] + leg_balance.roll_height.lqr.feedback[1][2]);
//     dbg[45] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][3] + leg_balance.roll_height.lqr.feedback[1][3]);
//     dbg[46] = machine->lqr.filter.lpf_alpha[2];
//     dbg[47] = (float)robot_state.fallen;
//     taskEXIT_CRITICAL();

//     /* LQR静止抖动分项：只读已有K/状态，不重算增益、不推进控制器。
//      * 27~46按四输出各五组记录限幅前贡献，单位统一Nm。
//      * 世界腿角+pitch即相对机身腿角；长度与状态取自LQR本身。
//      */
//     taskENTER_CRITICAL();
//     gain = LQR_Gain_Info();
//     for (uint8_t j = 0u; j < LQR_X_NUM; j++)
//     {
//         dbg[3u + j] = lqr_state.x[j];
//     }
//     dbg[13] = lqr_state.target[LQR_X_S] + gain->x_eq[LQR_X_S];
//     dbg[14] = lqr_state.target[LQR_X_DS] + gain->x_eq[LQR_X_DS];
//     dbg[15] = lqr_state.target[LQR_X_PHI] + gain->x_eq[LQR_X_PHI];
//     dbg[16] = lqr_state.target[LQR_X_DPHI] + gain->x_eq[LQR_X_DPHI];
//     dbg[17] = lqr_state.target[LQR_X_THL] + gain->x_eq[LQR_X_THL];
//     dbg[18] = lqr_state.target[LQR_X_THR] + gain->x_eq[LQR_X_THR];
//     dbg[19] = lqr_state.target[LQR_X_THB] + gain->x_eq[LQR_X_THB];
//     dbg[20] = lqr_state.len[0];
//     dbg[21] = lqr_state.len[1];
//     /* bit0状态有效/1增益有效/2 AIR反馈/3轴向释放/4离地/5三态有效/6四髋限幅/7四态有效；bit8~9三态。 */
//     dbg[22] = (float)((lqr_state.valid ? 0x01u : 0u) |
//                       (lqr_state.gain_valid ? 0x02u : 0u) |
//                       (lqr_state.air_control ? 0x04u : 0u) |
//                       (lqr_state.air_release ? 0x08u : 0u) |
//                       (air_detection.flight ? 0x10u : 0u) |
//                       (control_condition.valid ? 0x20u : 0u) |
//                       (leg_balance.compensation.saturated ? 0x40u : 0u) |
//                       (leg_balance.roll_height.valid ? 0x80u : 0u) |
//                       ((uint16_t)control_condition.condition << 8));
//     /* 最终门控后的轮矩请求；不能再用未投入时的LQR缓存代表已下发输出。 */
//     dbg[23] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT];
//     dbg[24] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT];
//     dbg[25] = lqr_state.u[LQR_U_BL]; /* 虚拟髋模型输出，尚未组合轴向力/补偿。 */
//     dbg[26] = lqr_state.u[LQR_U_BR];
//     /* 仅在LQR投入且状态/增益有效时重建；后台读取，不保证与缓存u严格同拍。
//      * 每输出：速度、左腿角速度、右腿角速度、pitch角速度、其余状态+u_eq。
//      * AIR掩码、开关和角度环绕与LQR_Control_Update保持一致。
//      */
//     if (output_task_lqr_engaged() && lqr_state.valid && lqr_state.gain_valid)
//     {
//         for (uint8_t i = 0u; i < LQR_U_NUM; i++)
//         {
//             dbg[31u + i * 5u] = lqr_state.air_control ? 0.0f : gain->u_eq[i];
//             for (uint8_t j = 0u; j < LQR_X_NUM; j++)
//             {
//                 if (lqr_state.air_control && (i == LQR_U_WL || i == LQR_U_WR || j < LQR_X_THL || j > LQR_X_DTHR))
//                 {
//                     term = 0.0f;
//                 }
//                 else if (j == LQR_X_PHI)
//                 {
//                     term = lqr_debug.yaw_hold ? lqr_state.K[i][j] * Angle_Wrap_Pi(lqr_state.target[j] + gain->x_eq[j] - lqr_state.x[j]) : 0.0f;
//                 }
//                 else if ((j == LQR_X_DPHI && !lqr_debug.yaw_rate_hold) || (j == LQR_X_S && !lqr_debug.pos_hold))
//                 {
//                     term = 0.0f;
//                 }
//                 else if (lqr_state.air_control && (j == LQR_X_THL || j == LQR_X_THR))
//                 {
//                     term = lqr_state.K[i][j] * remainderf(lqr_state.target[j] + gain->x_eq[j] - lqr_state.x[j], LEG_2PI);
//                 }
//                 else
//                 {
//                     term = lqr_state.K[i][j] * (lqr_state.target[j] + gain->x_eq[j] - lqr_state.x[j]);
//                 }
//                 switch (j)
//                 {
//                 case LQR_X_DS:   group = 0u; break;
//                 case LQR_X_DTHL: group = 1u; break;
//                 case LQR_X_DTHR: group = 2u; break;
//                 case LQR_X_DTHB: group = 3u; break;
//                 default:         group = 4u; break;
//                 }
//                 dbg[27u + i * 5u + group] += term;
//             }
//         }
//     }
//     dbg[47] = (float)HAL_GetTick(); /* 毫秒时间戳；按它识别掉帧，不将记录强当固定周期。 */
//     taskEXIT_CRITICAL();
//
//
//     /* LQR自起失败诊断：固定赋值，角度rad、速度rad/s、力N、力矩Nm。 */
//     taskENTER_CRITICAL();
//     dbg[3] = (float)rc_command.mode.strategy;
//     dbg[4] = (float)rc_command.mode.engage;
//     dbg[5] = (float)fallen_process_control.lifecycle.phase;
//     dbg[6] = (float)fallen_process_control.lifecycle.fault;
//     dbg[7] = fallen_process_control.lifecycle.elapsed;
//     dbg[8] = fallen_process_control.lifecycle.stable_elapsed;
//     dbg[9] = (float)control_condition.condition;
//     dbg[10] = (float)control_condition.valid;
//     dbg[11] = robot_state.height;
//     dbg[12] = imu_state.pitch_world;
//     dbg[13] = leg_l.output.virtual_leg_length;
//     dbg[14] = leg_r.output.virtual_leg_length;
//     dbg[15] = fallen_process_control.actuator.length_target[0];
//     dbg[16] = fallen_process_control.actuator.length_target[1];
//     dbg[17] = fallen_process_control.actuator.length_ramp[0].out;
//     dbg[18] = fallen_process_control.actuator.length_ramp[1].out;
//     dbg[19] = leg_l.output.virtual_leg_angle;
//     dbg[20] = leg_r.output.virtual_leg_angle;
//     dbg[21] = fallen_process_control.actuator.angle_target[0];
//     dbg[22] = fallen_process_control.actuator.angle_target[1];
//     /* PID误差已包含该阶段实际采用的环绕/展开角处理。 */
//     dbg[23] = fallen_process_control.actuator.angle_pos_pid[0].err[NOW];
//     dbg[24] = fallen_process_control.actuator.angle_pos_pid[1].err[NOW];
//     dbg[25] = leg_l.output.d_virtual_leg_angle;
//     dbg[26] = leg_r.output.d_virtual_leg_angle;
//     dbg[27] = fallen_process_control.actuator.angle_pos_pid[0].position_mode_output;
//     dbg[28] = fallen_process_control.actuator.angle_pos_pid[1].position_mode_output;
//     dbg[29] = fallen_process_control.actuator.angle_speed_pid[0].position_mode_output;
//     dbg[30] = fallen_process_control.actuator.angle_speed_pid[1].position_mode_output;
//     dbg[31] = fallen_process_control.actuator.length_pid[0].position_mode_output;
//     dbg[32] = fallen_process_control.actuator.length_pid[1].position_mode_output;
//     for (uint8_t i = 0u; i < 4u; i++)
//     {
//         dbg[33u + i] = output_dm_torque_nm[i];
//         dbg[37u + i] = motor_state.dm.trq_nm[i];
//     }
//     dbg[41] = (float)fallen_process_control.actuator.compensation.saturated;
//     dbg[42] = (float)output_debug_dm_sent;
//     dbg[43] = (float)output_debug_dji_sent;
//     dbg[44] = (float)fallen_process_control.lifecycle.retry;
//     dbg[45] = fallen_process_param.angle_timeout;
//     dbg[46] = (float)robot_state.height_valid;
//     dbg[47] = (float)HAL_GetTick();
//     taskEXIT_CRITICAL();
//
//     /* model5999固定布局：32维当前帧、估速及恢复；不按模式选帧。 */
//     taskENTER_CRITICAL();
//     for (uint8_t i = 0u; i < RL_OBS_SIZE; i++)
//     {
//         dbg[3u + i] = rl_control.observation.obs[i];
//     }
//     for (uint8_t i = 0u; i < 3u; i++) { dbg[35u + i] = rl_control.policy.velocity[i]; }
//     dbg[38] = (float)rl_control.recovery.phase;
//     dbg[39] = robot_state.height + MACHINE_RL_LEG_TO_HEIGHT;
//     dbg[40] = (float)rl_control.recovery.support_valid;
//     dbg[41] = rl_control.recovery.wheel_force[0];
//     dbg[42] = rl_control.recovery.wheel_force[1];
//     dbg[43] = (float)rl_control.recovery.has_stood;
//     dbg[44] = (float)rl_control.recovery.stable_ms;
//     dbg[45] = (float)rl_control.recovery.tracking_ms;
//     dbg[46] = (float)rl_control.policy.run_us;
//     dbg[47] = (float)HAL_GetTick();
//     taskEXIT_CRITICAL();
// //
//
//     /* model1999：完整obs35与原始action6，固定通道，不按模式选帧。 */
//     /* 3~6命令，7~9角速度，10~12重力，13~18位置，19~24速度，25~30上一动作，31~37模式。
//      * 38~43当前动作：左前/左后/右前/右后/左轮/右轮；44阶段、45obs有效、46推理us、47时间ms。
//      * 后台读取最新共享值，当前动作与obs不保证来自同一次推理，仅作趋势诊断。 */
//     taskENTER_CRITICAL();
//     for (uint8_t i = 0u; i < RL_OBS_SIZE; i++) { dbg[3u + i] = rl_control.observation.obs[i]; }
//     for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++) { dbg[38u + i] = action_state.a[i]; }
//     dbg[44] = (float)rl_control.recovery.phase;
//     dbg[45] = (float)rl_control.observation.valid;
//     dbg[46] = (float)rl_control.policy.run_us;
//     dbg[47] = (float)HAL_GetTick();
//     taskEXIT_CRITICAL();
    /* 上一版十态/四态联合布局，保留供手动恢复；本版改为腿长差排查布局。 */
#if 0
    taskENTER_CRITICAL();
    dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (leg_balance.roll_height.valid ? 4u : 0u) | (control_condition.valid ? 8u : 0u) | (lqr_state.air_control ? 16u : 0u) | (lqr_state.air_release ? 32u : 0u) | (air_detection.flight ? 64u : 0u) | (leg_balance.compensation.saturated ? 128u : 0u) | (leg_balance.roll_height.observer.permit ? 256u : 0u) | (leg_balance.roll_height.observer.updated ? 512u : 0u) | (leg_balance.roll_height.observer.limited ? 1024u : 0u) | (leg_balance.roll_height.observer.seeded ? 2048u : 0u) | ((uint32_t)control_condition.condition << 12) | (output_debug_dm_sent ? 16384u : 0u) | (output_debug_dji_sent ? 32768u : 0u));
    dbg[4] = leg_balance.roll_height.height_target;
    dbg[5] = leg_balance.roll_height.target[2];
    dbg[6] = leg_balance.roll_height.x[2];
    dbg[7] = leg_balance.roll_height.height_rate_raw;
    dbg[8] = leg_balance.roll_height.x[3];
    dbg[9] = leg_balance.roll_height.x[0];
    dbg[10] = leg_balance.roll_height.target[3]; /* 高度速度参考m/s；roll目标固定0。 */
    dbg[11] = lqr_state.roll_rate;
    dbg[12] = leg_balance.roll_height.x[1];
    dbg[13] = lqr_state.leg_length_target[0]; /* 当前两侧共用最终给定。 */
    dbg[14] = leg_balance.length_ramp[0].out;
    dbg[15] = leg_balance.length_ramp[1].out;
    dbg[16] = leg_l.output.virtual_leg_length;
    dbg[17] = leg_r.output.virtual_leg_length;
    dbg[18] = leg_l.output.d_virtual_leg_length;
    dbg[19] = leg_r.output.d_virtual_leg_length;
    dbg[20] = lqr_state.x[LQR_X_THB];
    dbg[21] = lqr_state.x[LQR_X_DTHB];
    dbg[22] = lqr_state.x[LQR_X_THL];
    dbg[23] = lqr_state.x[LQR_X_THR];
    dbg[24] = lqr_state.x[LQR_X_DTHL];
    dbg[25] = lqr_state.x[LQR_X_DTHR];
    dbg[26] = lqr_state.x[LQR_X_DS];
    dbg[27] = lqr_state.target[LQR_X_DS];
    dbg[28] = lqr_state.x[LQR_X_DPHI];
    dbg[29] = lqr_state.target[LQR_X_DPHI];
    dbg[30] = leg_balance.F[0];
    dbg[31] = leg_balance.F[1];
    dbg[32] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][2] + leg_balance.roll_height.lqr.feedback[1][2]);
    dbg[33] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][3] + leg_balance.roll_height.lqr.feedback[1][3]);
    dbg[34] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][0] - leg_balance.roll_height.lqr.feedback[1][0]);
    dbg[35] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][1] - leg_balance.roll_height.lqr.feedback[1][1]);
    dbg[36] = 0.5f * (leg_balance.roll_height.observer.bias[0] + leg_balance.roll_height.observer.bias[1]);
    dbg[37] = 0.5f * (leg_balance.roll_height.observer.bias[0] - leg_balance.roll_height.observer.bias[1]);
    dbg[38] = leg_balance.turn_ff;
    dbg[39] = leg_balance.Tp[0];
    dbg[40] = leg_balance.Tp[1];
    dbg[41] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT];
    dbg[42] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT];
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        dbg[43u + i] = output_dm_torque_nm[i];
    }
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif

//     /* 腿长差排查布局：只读执行拍已发布状态，不推进控制器。
//      * 4~9   左右虚拟腿长、腿长差、腿长均值、左右伸缩速度(m,m/s)
//      * 10~13 四髋反馈关节角 左前/左后/右前/右后(rad，已含驱动标定)；用于看机构是否在剪叉
//      * 14~16 左右轴向力请求N、向心项N；17偏置共模N
//      * 18~20 最终腿长目标、左右斜坡目标m
//      * 21~23 机体俯仰rad、左右世界系腿摆角rad
//      * 24~26 roll rad、roll角速度原始/滤后 rad/s
//      * 27~28 投影高度及斜坡目标m
//      * 29~31 实测前进速度m/s、yaw角速度及目标rad/s
//      * 32~35 左右轮力矩请求Nm、左右轮反馈电流raw(未加载的轮电流会明显偏小)
//      * 36~39 AIR每腿支持力估计N、由反馈反解的每腿电机轴向力N
//      * 40~42 原始左右力半差N、协调抬升量N、左腿力映射行列式
//      * 43~46 四髋最终力矩请求Nm 左前/左后/右前/右后；47毫秒时间
//      * 换版本时手工恢复上一段即可，不按模式选帧。 */
//     taskENTER_CRITICAL();
//     dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (leg_balance.roll_height.valid ? 4u : 0u) | (control_condition.valid ? 8u : 0u) | (lqr_state.air_control ? 16u : 0u) | (lqr_state.air_release ? 32u : 0u) | (air_detection.flight ? 64u : 0u) | (leg_balance.compensation.saturated ? 128u : 0u) | (leg_balance.roll_height.observer.permit ? 256u : 0u) | (leg_balance.roll_height.observer.updated ? 512u : 0u) | (leg_balance.roll_height.observer.limited ? 1024u : 0u) | (leg_balance.roll_height.observer.seeded ? 2048u : 0u) | ((uint32_t)control_condition.condition << 12) | (output_debug_dm_sent ? 16384u : 0u) | (output_debug_dji_sent ? 32768u : 0u) | (leg_balance.roll_height.lqr.common_shift > 0.0f ? 65536u : 0u));
//     dbg[4] = leg_l.output.virtual_leg_length;
//     dbg[5] = leg_r.output.virtual_leg_length;
//     dbg[6] = leg_l.output.virtual_leg_length - leg_r.output.virtual_leg_length;
//     dbg[7] = 0.5f * (leg_l.output.virtual_leg_length + leg_r.output.virtual_leg_length);
//     dbg[8] = leg_l.output.d_virtual_leg_length;
//     dbg[9] = leg_r.output.d_virtual_leg_length;
//     dbg[10] = leg_l.input.hip_f;
//     dbg[11] = leg_l.input.hip_b;
//     dbg[12] = leg_r.input.hip_f;
//     dbg[13] = leg_r.input.hip_b;
//     dbg[14] = leg_balance.F[0];
//     dbg[15] = leg_balance.F[1];
//     dbg[16] = leg_balance.turn_ff;
//     dbg[17] = 0.5f * (leg_balance.roll_height.observer.bias[0] + leg_balance.roll_height.observer.bias[1]);
//     dbg[18] = lqr_state.leg_length_target[0];
//     dbg[19] = leg_balance.length_ramp[0].out;
//     dbg[20] = leg_balance.length_ramp[1].out;
//     dbg[21] = lqr_state.x[LQR_X_THB];
//     dbg[22] = lqr_state.x[LQR_X_THL];
//     dbg[23] = lqr_state.x[LQR_X_THR];
//     dbg[24] = lqr_state.roll;
//     dbg[25] = lqr_state.roll_rate;
//     dbg[26] = leg_balance.roll_height.x[1];
//     dbg[27] = leg_balance.roll_height.x[2];
//     dbg[28] = leg_balance.roll_height.target[2];
//     /* 电机协议反馈力矩Nm，保持与43~46请求相同的物理坐标和顺序。 */
//     dbg[29] = motor_state.dm.trq_nm[DM_MOTOR_LEG_F_LFT];
//     dbg[30] = motor_state.dm.trq_nm[DM_MOTOR_LEG_B_LFT];
//     dbg[31] = motor_state.dm.trq_nm[DM_MOTOR_LEG_F_RGT];
//     /* 四髋顺序：左前、左后、右前、右后；实际叠加的补偿力矩Nm。 */
//     for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
//     {
//         dbg[32u + i] = leg_balance.compensation.spring_dm[i];
//         dbg[36u + i] = leg_balance.compensation.gravity.delta_dm[i];
//     }
//     dbg[40] = leg_balance.roll_height.lqr.height_trim; /* 本拍静态前馈修正N/腿 */
//     dbg[41] = leg_balance.roll_height.lqr.common_shift; /* 下限协调抬升的平均力N */
//     dbg[42] = motor_state.dm.trq_nm[DM_MOTOR_LEG_B_RGT]; /* 右后反馈Nm */
//     for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
//     {
//         dbg[43u + i] = output_dm_torque_nm[i];
//     }
//     dbg[47] = (float)HAL_GetTick();
//     taskEXIT_CRITICAL();

//     /* 腿长闭环与观测输入诊断。 */
//     taskENTER_CRITICAL();
//     dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (leg_balance.roll_height.valid ? 4u : 0u) | (control_condition.valid ? 8u : 0u) | (lqr_state.air_control ? 16u : 0u) | (lqr_state.air_release ? 32u : 0u) | (air_detection.flight ? 64u : 0u) | (leg_balance.compensation.saturated ? 128u : 0u) | (leg_balance.roll_height.observer.permit ? 256u : 0u) | (leg_balance.roll_height.observer.updated ? 512u : 0u) | (leg_balance.roll_height.observer.limited ? 1024u : 0u) | (leg_balance.roll_height.observer.seeded ? 2048u : 0u) | ((uint32_t)control_condition.condition << 12) | (output_debug_dm_sent ? 16384u : 0u) | (output_debug_dji_sent ? 32768u : 0u) | (leg_balance.roll_height.lqr.common_shift > 0.0f ? 65536u : 0u));
//     dbg[4] = leg_l.output.virtual_leg_length;
//     dbg[5] = leg_r.output.virtual_leg_length;
//     dbg[6] = leg_l.output.virtual_leg_length - leg_r.output.virtual_leg_length;
//     dbg[7] = 0.5f * (leg_l.output.virtual_leg_length + leg_r.output.virtual_leg_length);
//     dbg[8] = leg_l.output.d_virtual_leg_length;
//     dbg[9] = leg_r.output.d_virtual_leg_length;
//     dbg[10] = leg_l.input.hip_f;
//     dbg[11] = leg_l.input.hip_b;
//     dbg[12] = leg_r.input.hip_f;
//     dbg[13] = leg_r.input.hip_b;
//     dbg[14] = leg_balance.F[0];
//     dbg[15] = leg_balance.F[1];
//     dbg[16] = leg_balance.turn_ff;
//     dbg[17] = 0.5f * (leg_balance.roll_height.observer.bias[0] + leg_balance.roll_height.observer.bias[1]);
//     dbg[18] = lqr_state.leg_length_target[0];
//     dbg[19] = leg_balance.length_ramp[0].out;
//     dbg[20] = leg_balance.length_ramp[1].out;
//     dbg[21] = lqr_state.x[LQR_X_THB];
//     dbg[22] = lqr_state.x[LQR_X_THL];
//     dbg[23] = lqr_state.x[LQR_X_THR];
//     dbg[24] = lqr_state.roll;
//     dbg[25] = lqr_state.roll_rate;
//     dbg[26] = leg_balance.roll_height.x[1];
//     dbg[27] = leg_balance.roll_height.x[2];
//     dbg[28] = leg_balance.roll_height.target[2];
//     /* 高度速度：原始/滤后/参考，m/s。 */
//     dbg[29] = leg_balance.roll_height.height_rate_raw;
//     dbg[30] = leg_balance.roll_height.x[3];
//     dbg[31] = leg_balance.roll_height.target[3];
//     dbg[32] = leg_balance.roll_height.lqr.observer_force[0];
//     dbg[33] = leg_balance.roll_height.lqr.observer_force[1];
//     dbg[34] = leg_balance.roll_height.lqr.observer_input_used;
//     dbg[35] = 0.5f * (leg_balance.roll_height.lqr.base_force[0] + leg_balance.roll_height.lqr.base_force[1]);
//     dbg[36] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][2] + leg_balance.roll_height.lqr.feedback[1][2]);
//     dbg[37] = 0.5f * (leg_balance.roll_height.lqr.feedback[0][3] + leg_balance.roll_height.lqr.feedback[1][3]);
//     dbg[38] = machine->lqr.roll_height.support_force[0];
//     dbg[39] = machine->lqr.roll_height.support_force[1];
//     dbg[40] = leg_balance.roll_height.lqr.height_trim; /* 本拍静态前馈修正N/腿 */
//     dbg[41] = leg_balance.roll_height.lqr.common_shift; /* 下限协调抬升的平均力N */
//     dbg[42] = (float)MACHINE_HEIGHT_TRIM_ENABLE;
//     for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
//     {
//         dbg[43u + i] = output_dm_torque_nm[i];
//     }
//     dbg[47] = (float)HAL_GetTick();
//     taskEXIT_CRITICAL();

    /* 位移权重专项：保留位移/速度分项，增加停车门槛与腿姿态偏差。 */
#if 0
    float position_error;
    float velocity_error;
    taskENTER_CRITICAL();
    position_error = lqr_state.target[LQR_X_S] + LQR_Gain_Info()->x_eq[LQR_X_S] - lqr_state.x[LQR_X_S];
    velocity_error = lqr_state.target[LQR_X_DS] + LQR_Gain_Info()->x_eq[LQR_X_DS] - lqr_state.x[LQR_X_DS];
    dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (lqr_state.air_control ? 4u : 0u) | (lqr_state.air_release ? 8u : 0u) | (leg_balance.compensation.saturated ? 16u : 0u) | (lqr_state.target[LQR_X_DS] == 0.0f ? 32u : 0u) | (lqr_state.pos_armed ? 64u : 0u) | (lqr_debug.pos_hold ? 128u : 0u) | (lqr_debug.pos_arm_vel <= 0.0f || fabsf(lqr_state.x[LQR_X_DS]) < lqr_debug.pos_arm_vel ? 256u : 0u) | (output_debug_dm_sent ? 512u : 0u) | (output_debug_dji_sent ? 1024u : 0u));
    dbg[4] = lqr_state.x[LQR_X_S];
    dbg[5] = lqr_state.target[LQR_X_S] + LQR_Gain_Info()->x_eq[LQR_X_S];
    dbg[6] = position_error;
    dbg[7] = lqr_state.target[LQR_X_DS];
    dbg[8] = lqr_state.x[LQR_X_DS];
    dbg[9] = velocity_error;
    dbg[10] = (float)lqr_state.pos_armed;
    dbg[11] = (float)lqr_debug.pos_hold;
    dbg[12] = leg_l.output.virtual_leg_angle;
    dbg[13] = leg_r.output.virtual_leg_angle;
    dbg[14] = lqr_state.x[LQR_X_THL];
    dbg[15] = lqr_state.x[LQR_X_THR];
    dbg[16] = lqr_state.target[LQR_X_THL] + LQR_Gain_Info()->x_eq[LQR_X_THL];
    dbg[17] = lqr_state.target[LQR_X_THR] + LQR_Gain_Info()->x_eq[LQR_X_THR];
    dbg[18] = lqr_state.x[LQR_X_THB];
    dbg[19] = lqr_state.x[LQR_X_DTHB];
    for (uint8_t i = 0u; i < LQR_U_NUM; i++)
    {
        /* 输出顺序：左轮、右轮、左腿虚拟摆矩、右腿虚拟摆矩；非最终髋电机矩。 */
        dbg[20u + i] = lqr_state.valid && lqr_state.gain_valid && !lqr_state.air_control && lqr_debug.pos_hold ? lqr_state.K[i][LQR_X_S] * position_error : 0.0f;
        dbg[24u + i] = lqr_state.valid && lqr_state.gain_valid && !lqr_state.air_control ? lqr_state.K[i][LQR_X_DS] * velocity_error : 0.0f;
        dbg[28u + i] = lqr_state.u[i];
        dbg[40u + i] = lqr_state.K[i][LQR_X_S];
    }
    dbg[32] = remainderf(lqr_state.x[LQR_X_THL] - machine->lqr.reference.leg_trim[0], LEG_2PI);
    dbg[33] = remainderf(lqr_state.x[LQR_X_THR] - machine->lqr.reference.leg_trim[1], LEG_2PI);
    dbg[34] = lqr_debug.pos_arm_vel;
    dbg[35] = rc_command.velocity_target;
    dbg[36] = leg_l.output.virtual_leg_length;
    dbg[37] = leg_r.output.virtual_leg_length;
    dbg[38] = lqr_state.leg_length_target[0];
    dbg[39] = lqr_state.roll;
    dbg[44] = lqr_state.K[LQR_U_BL][LQR_X_DS];
    dbg[45] = lqr_state.K[LQR_U_BR][LQR_X_DS];
    dbg[46] = (float)((fabsf(lqr_state.u[LQR_U_WL]) >= lqr_debug.trq_max_wheel ? 1u : 0u) | (fabsf(lqr_state.u[LQR_U_WR]) >= lqr_debug.trq_max_wheel ? 2u : 0u) | (fabsf(lqr_state.u[LQR_U_BL]) >= lqr_debug.trq_max_hip ? 4u : 0u) | (fabsf(lqr_state.u[LQR_U_BR]) >= lqr_debug.trq_max_hip ? 8u : 0u));
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
    /* 普通趴地自起路径采集v3：只读已有状态，不生成控制目标。
     * 16/17沿用路径的角差定义；先fmod有界化，再按原±pi端点环绕。
     * 18/19为PID上次实际使用的测量，RETRACT暂停角环时会保留旧值。
     * 22/23/45观察支持力、AIR条件计时和原始flight，区分交接与后续离地。
     * 后台同帧不是同一物理采样；仅在对应阶段解释PID和路径字段。 */
#if 0
    taskENTER_CRITICAL();
    dbg[3] = (float)(fallen_process_control.lifecycle.phase);
    dbg[4] = (float)(fallen_process_control.lifecycle.fault);
    dbg[5] = (float)(fallen_process_control.lifecycle.elapsed);
    dbg[6] = (float)(fallen_process_control.lifecycle.stable_elapsed);
    dbg[7] = (float)(fallen_process_control.lifecycle.retry);
    dbg[8] = (float)(control_postprocess.phase);
    dbg[9] = (float)(control_condition.condition);
    dbg[10] = (float)(control_condition.valid);
    dbg[11] = (float)(fallen_detection.inverted);
    dbg[12] = (float)(imu_state.pitch_world);
    dbg[13] = (float)(imu_state.gyro_rad_s[1]); /* 原始俯仰角速度，仅观察 */
    dbg[14] = (float)(leg_l.output.virtual_leg_angle);
    dbg[15] = (float)(leg_r.output.virtual_leg_angle);
    dbg[16] = (float)(Angle_Wrap_Pi(fmodf(leg_l.output.virtual_leg_angle - imu_state.pitch_world, LEG_2PI)));
    dbg[17] = (float)(Angle_Wrap_Pi(fmodf(leg_r.output.virtual_leg_angle - imu_state.pitch_world, LEG_2PI)));
    dbg[18] = (float)(fallen_process_control.actuator.angle_pos_pid[0].measured[NOW]);
    dbg[19] = (float)(fallen_process_control.actuator.angle_pos_pid[1].measured[NOW]);
    dbg[20] = (float)(fallen_process_control.actuator.angle_target[0]);
    dbg[21] = (float)(fallen_process_control.actuator.angle_target[1]);
    dbg[22] = (float)(air_detection.mean_force); /* 平均支持力N */
    dbg[23] = (float)(air_detection.elapsed); /* AIR当前条件计时s */
    dbg[24] = (float)(leg_l.output.d_virtual_leg_angle);
    dbg[25] = (float)(leg_r.output.d_virtual_leg_angle);
    dbg[26] = (float)(leg_l.output.virtual_leg_length);
    dbg[27] = (float)(leg_r.output.virtual_leg_length);
    dbg[28] = (float)(fallen_process_control.actuator.length_target[0]);
    dbg[29] = (float)(fallen_process_control.actuator.length_target[1]);
    dbg[30] = (float)(fallen_process_control.actuator.length_ramp[0].out);
    dbg[31] = (float)(fallen_process_control.actuator.length_ramp[1].out);
    dbg[32] = (float)(fallen_process_control.actuator.length_pid[0].position_mode_output);
    dbg[33] = (float)(fallen_process_control.actuator.length_pid[1].position_mode_output);
    dbg[34] = (float)(fallen_process_control.actuator.angle_speed_pid[0].position_mode_output);
    dbg[35] = (float)(fallen_process_control.actuator.angle_speed_pid[1].position_mode_output);
    dbg[36] = (float)(fallen_process_control.actuator.support);
    dbg[37] = (float)(output_dm_torque_nm[DM_MOTOR_LEG_F_LFT]);
    dbg[38] = (float)(output_dm_torque_nm[DM_MOTOR_LEG_B_LFT]);
    dbg[39] = (float)(output_dm_torque_nm[DM_MOTOR_LEG_F_RGT]);
    dbg[40] = (float)(output_dm_torque_nm[DM_MOTOR_LEG_B_RGT]);
    dbg[41] = (float)(motor_state.dm.trq_nm[DM_MOTOR_LEG_F_LFT]);
    dbg[42] = (float)(motor_state.dm.trq_nm[DM_MOTOR_LEG_B_LFT]);
    dbg[43] = (float)(motor_state.dm.trq_nm[DM_MOTOR_LEG_F_RGT]);
    dbg[44] = (float)(motor_state.dm.trq_nm[DM_MOTOR_LEG_B_RGT]);
    dbg[45] = (float)(air_detection.flight); /* 原始AIR结果，不是三态选路 */
    /* bit0路径就绪/bit1新普通路径，bit2~5左右路径。 */
    dbg[46] = (float)((fallen_process_control.normal.rear_path_ready ? 1u : 0u) |
                      (fallen_process_control.normal.planned ? 2u : 0u) |
                      ((uint8_t)fallen_process_control.normal.path[0] << 2) |
                      ((uint8_t)fallen_process_control.normal.path[1] << 4));
    dbg[47] = (float)(HAL_GetTick());
    taskEXIT_CRITICAL();
#endif
    /* 估力分项诊断v1：仅观察现有结果，0～2保持，完整通道/单位见file-guide。
     * 低/中/高腿静立使用；theta沿用Air_Estimate原定义，不更改轴向或标定。
     * 各来源是后台当前值，不保证与AIR内部严格同拍；动态瞬态不可逐行强行对账。 */
#if 0
    taskENTER_CRITICAL();
    dbg[3] = (float)control_condition.condition;
    dbg[4] = (float)control_condition.valid;
    dbg[5] = (float)air_detection.valid;
    dbg[6] = (float)((air_detection.initialized ? 1u : 0u) |
                    (air_detection.flight ? 2u : 0u) |
                    (air_detection.fault ? 4u : 0u) |
                    (leg_l.output.force_valid ? 8u : 0u) |
                    (leg_r.output.force_valid ? 16u : 0u) |
                    (leg_balance.compensation.saturated ? 32u : 0u) |
                    (output_debug_dm_sent ? 64u : 0u) |
                    (leg_balance.roll_height.valid ? 128u : 0u) |
                    (output_debug_dji_sent ? 256u : 0u));
    dbg[7] = leg_balance.roll_height.lqr.height_trim; /* N/腿，实际四态高度补偿 */
    dbg[8] = leg_l.output.virtual_leg_length;
    dbg[9] = leg_r.output.virtual_leg_length;
    dbg[10] = remainderf(leg_l.output.virtual_leg_angle - leg_l.config.offset_phi0 - imu_state.euler_rad[ATTITUDE_ROLL], LEG_2PI);
    dbg[11] = remainderf(leg_r.output.virtual_leg_angle - leg_r.config.offset_phi0 - imu_state.euler_rad[ATTITUDE_ROLL], LEG_2PI);
    dbg[12] = imu_state.accel_vertical;
    dbg[13] = air_detection.leg[0].motor_force;
    dbg[14] = air_detection.leg[1].motor_force;
    dbg[15] = air_detection.leg[0].motor_torque;
    dbg[16] = air_detection.leg[1].motor_torque;
    dbg[17] = air_detection.leg[0].spring_force;
    dbg[18] = air_detection.leg[1].spring_force;
    dbg[19] = air_detection.leg[0].gravity_torque;
    dbg[20] = air_detection.leg[1].gravity_torque;
    dbg[21] = air_detection.leg[0].pressure;
    dbg[22] = air_detection.leg[1].pressure;
    dbg[23] = air_detection.leg[0].height_acceleration;
    dbg[24] = air_detection.leg[1].height_acceleration;
    dbg[25] = air_detection.leg[0].wheel_acceleration;
    dbg[26] = air_detection.leg[1].wheel_acceleration;
    dbg[27] = air_detection.leg[0].force_raw;
    dbg[28] = air_detection.leg[1].force_raw;
    dbg[29] = air_detection.leg[0].force;
    dbg[30] = air_detection.leg[1].force;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        /* 左前/左后/右前/右后；物理坐标Nm，不重复乘驱动极性。 */
        dbg[31u + i] = motor_state.dm.trq_nm[i];
        dbg[35u + i] = output_dm_torque_nm[i];
    }
    for (uint8_t row = 0u; row < 2u; row++)
    {
        for (uint8_t col = 0u; col < 2u; col++)
        {
            dbg[39u + 2u * row + col] = leg_l.output.force_map[row][col];
            dbg[43u + 2u * row + col] = leg_r.output.force_map[row][col];
        }
    }
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif

    /* 倒置上电诊断v1：只读姿态、倒置结果与恢复路径，0～2保持。
     * upright/inverted须结合检测valid解释；planned须结合恢复phase解释。
     * 同帧来自异步共享状态，不等同于控制拍快照；不重新计算姿态或执行许可。 */
#if 0
    taskENTER_CRITICAL();
    dbg[3] = (float)control_condition.condition;
    dbg[4] = (float)control_condition.valid;
    dbg[5] = (float)imu_state.pitch_world_valid;
    dbg[6] = fallen_detection.upright;
    dbg[7] = (float)fallen_detection.inverted;
    dbg[8] = (float)fallen_process_control.lifecycle.phase;
    dbg[9] = (float)fallen_process_control.lifecycle.fault;
    dbg[10] = (float)fallen_process_control.normal.planned;
    dbg[11] = imu_state.euler_rad[ATTITUDE_ROLL]; /* 补角实际使用的欧拉输入rad */
    dbg[12] = imu_state.pitch_world;
    dbg[13] = imu_state.quat[0];
    dbg[14] = imu_state.quat[1];
    dbg[15] = imu_state.quat[2];
    dbg[16] = imu_state.quat[3];
    dbg[17] = imu_state.acc_g[0];
    dbg[18] = imu_state.acc_g[1];
    dbg[19] = imu_state.acc_g[2];
    dbg[20] = (float)fallen_detection.valid;
    dbg[21] = (float)imu_state.online;
    dbg[22] = (float)leg_l.output.valid;
    dbg[23] = (float)leg_r.output.valid;
    dbg[24] = (float)leg_l.output.force_valid;
    dbg[25] = (float)leg_r.output.force_valid;
    dbg[26] = (float)fallen_process_control.lifecycle.recovery_enabled;
    dbg[27] = (float)control_postprocess.phase;
    dbg[28] = fallen_process_control.lifecycle.elapsed;
    dbg[29] = fallen_process_control.lifecycle.stable_elapsed;
    dbg[30] = leg_l.output.virtual_leg_angle;
    dbg[31] = leg_r.output.virtual_leg_angle;
    dbg[32] = fallen_process_control.actuator.angle_target[0];
    dbg[33] = fallen_process_control.actuator.angle_target[1];
    dbg[34] = leg_l.output.virtual_leg_length;
    dbg[35] = leg_r.output.virtual_leg_length;
    dbg[36] = fallen_process_control.actuator.length_target[0];
    dbg[37] = fallen_process_control.actuator.length_target[1];
    dbg[38] = (float)fallen_process_control.normal.path[0];
    dbg[39] = (float)fallen_process_control.normal.path[1];
    dbg[40] = output_dm_torque_nm[DM_MOTOR_LEG_F_LFT];
    dbg[41] = output_dm_torque_nm[DM_MOTOR_LEG_B_LFT];
    dbg[42] = output_dm_torque_nm[DM_MOTOR_LEG_F_RGT];
    dbg[43] = output_dm_torque_nm[DM_MOTOR_LEG_B_RGT];
    dbg[44] = (float)air_detection.flight;
    dbg[45] = (float)air_detection.valid;
    dbg[46] = (float)motor_state.valid;
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
#if 0 /* 上一版主动防翻布局保留 */
    /* 主动防翻v1固定48路；开关关闭同样记录原请求/实际运动，用于对照。 */
    taskENTER_CRITICAL();
    dbg[3] = (float)control_condition.condition;
    dbg[4] = (float)control_condition.valid;
    dbg[5] = (float)machine->lqr.motion_guard.enabled;
    dbg[6] = (float)motion_guard.valid;
    dbg[7] = (float)motion_guard.limited;
    dbg[8] = (float)motion_guard.reason;
    dbg[9] = rc_command.velocity_target;
    dbg[10] = lqr_state.target[LQR_X_DS];
    dbg[11] = lqr_state.ds_kf;
    dbg[12] = rc_command.yaw_rate_target;
    dbg[13] = lqr_state.target[LQR_X_DPHI];
    dbg[14] = remainderf(lqr_state.target[LQR_X_PHI] + LQR_Gain_Info()->x_eq[LQR_X_PHI] - lqr_state.x[LQR_X_PHI], LEG_2PI);
    dbg[15] = motion_guard.yaw_equivalent_raw;
    dbg[16] = motion_guard.yaw_equivalent;
    dbg[17] = motion_guard.reference_scale;
    dbg[18] = motion_guard.yaw_limit_raw;
    dbg[19] = motion_guard.yaw_limit;
    dbg[20] = motion_guard.lateral_budget;
    dbg[21] = motion_guard.length_guard;
    dbg[22] = motion_guard.height_reference;
    dbg[23] = lqr_state.len[0];
    dbg[24] = lqr_state.len[1];
    dbg[25] = lqr_state.leg_length_target[0];
    dbg[26] = lqr_state.roll;
    dbg[27] = lqr_state.roll_rate;
    dbg[28] = lqr_state.x[LQR_X_THB];
    dbg[29] = lqr_state.x[LQR_X_DTHB];
    dbg[30] = lqr_state.x[LQR_X_DPHI];
    dbg[31] = motion_guard.heading_ratio;
    dbg[32] = fabsf(lqr_state.ds_kf * lqr_state.x[LQR_X_DPHI]);
    dbg[33] = leg_balance.turn_ff;
    dbg[34] = lqr_state.u[LQR_U_WL];
    dbg[35] = lqr_state.u[LQR_U_WR];
    dbg[36] = (float)((fabsf(lqr_state.u[LQR_U_WL]) >= lqr_debug.trq_max_wheel ? 1u : 0u) | (fabsf(lqr_state.u[LQR_U_WR]) >= lqr_debug.trq_max_wheel ? 2u : 0u));
    dbg[37] = (float)leg_balance.compensation.saturated;
    dbg[38] = (float)output_debug_dm_sent;
    dbg[39] = (float)output_debug_dji_sent;
    dbg[40] = (float)motion_guard.releasing;
    dbg[41] = lqr_state.valid && lqr_state.gain_valid && lqr_debug.yaw_rate_hold && !lqr_state.air_control ? -lqr_state.K[LQR_U_WL][LQR_X_DPHI] * lqr_state.x[LQR_X_DPHI] : 0.0f;
    dbg[42] = lqr_state.valid && lqr_state.gain_valid && lqr_debug.yaw_rate_hold && !lqr_state.air_control ? -lqr_state.K[LQR_U_WR][LQR_X_DPHI] * lqr_state.x[LQR_X_DPHI] : 0.0f;
    dbg[43] = lqr_state.target[LQR_X_DPHI] * motion_guard.reference_scale;
    dbg[44] = dbg[14] * motion_guard.reference_scale;
    dbg[45] = (float)air_detection.flight;
    dbg[46] = (float)motor_state.valid;
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
#if 0 /* 上一版转弯低频速度来源布局保留 */
    /* 转弯低频速度来源诊断：运动学左右分解，保留速度/腿角/角速度贡献。 */
    taskENTER_CRITICAL();
    turn_scale = motion_guard.reference_scale;
    speed_error = lqr_state.target[LQR_X_DS] + LQR_Gain_Info()->x_eq[LQR_X_DS] - lqr_state.x[LQR_X_DS];
    dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (lqr_state.air_control ? 4u : 0u) | (lqr_state.air_release ? 8u : 0u));
    dbg[4] = rc_command.velocity_target;
    dbg[5] = lqr_state.target[LQR_X_DS];
    dbg[6] = lqr_state.x[LQR_X_DS];
    dbg[7] = rc_command.yaw_rate_target;
    dbg[8] = turn_scale * (lqr_state.target[LQR_X_DPHI] + LQR_Gain_Info()->x_eq[LQR_X_DPHI]);
    dbg[9] = lqr_state.x[LQR_X_DPHI];
    dbg[10] = lqr_state.ds_raw;
    dbg[11] = lqr_state.a_fwd;
    dbg[12] = leg_l.output.virtual_leg_angle;
    dbg[13] = leg_r.output.virtual_leg_angle;
    dbg[14] = lqr_state.x[LQR_X_THL];
    dbg[15] = lqr_state.x[LQR_X_THR];
    dbg[16] = lqr_state.x[LQR_X_DTHL];
    dbg[17] = lqr_state.x[LQR_X_DTHR];
    dbg[18] = lqr_state.x[LQR_X_THB];
    dbg[19] = lqr_state.x[LQR_X_DTHB];
    dbg[20] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    dbg[21] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    dbg[22] = lqr_state.len[0];
    dbg[23] = lqr_state.len[1];
    dbg[24] = lqr_debug.vel_leg_comp_sign;
    dbg[25] = lqr_debug.pitch_comp_sign;
    dbg[26] = lqr_state.whl[0] * machine->wheel.wheel_r;
    dbg[27] = lqr_state.whl[1] * machine->wheel.wheel_r;
    dbg[28] = -leg_l.output.virtual_leg_length * lqr_state.x[LQR_X_DTHL] * cosf(lqr_state.x[LQR_X_THL]);
    dbg[29] = -leg_r.output.virtual_leg_length * lqr_state.x[LQR_X_DTHR] * cosf(lqr_state.x[LQR_X_THR]);
    dbg[30] = -leg_l.output.d_virtual_leg_length * sinf(lqr_state.x[LQR_X_THL]);
    dbg[31] = -leg_r.output.d_virtual_leg_length * sinf(lqr_state.x[LQR_X_THR]);
    dbg[32] = dbg[26] + dbg[28] + dbg[30];
    dbg[33] = dbg[27] + dbg[29] + dbg[31];
    dbg[36] = lqr_state.u[LQR_U_BL];
    dbg[37] = lqr_state.u[LQR_U_BR];
    for (uint8_t side = 0u; side < 2u; side++)
    {
        uint8_t row = (uint8_t)(LQR_U_BL + side);
        if (lqr_state.valid && lqr_state.gain_valid && !lqr_state.air_control)
        {
            dbg[34u + side] = lqr_state.K[row][LQR_X_DS] * speed_error;
            /* 分开两腿角度列和角速度列，保留交叉反馈，不只看本侧。 */
            swing_term = 0.0f;
            for (uint8_t j = LQR_X_THL; j <= LQR_X_THR; j += 2u)
            {
                swing_term += lqr_state.K[row][j] * (lqr_state.target[j] + LQR_Gain_Info()->x_eq[j] - lqr_state.x[j]);
            }
            dbg[38u + side] = swing_term;
            swing_term = 0.0f;
            for (uint8_t j = LQR_X_DTHL; j <= LQR_X_DTHR; j += 2u)
            {
                swing_term += lqr_state.K[row][j] * (lqr_state.target[j] + LQR_Gain_Info()->x_eq[j] - lqr_state.x[j]);
            }
            dbg[40u + side] = swing_term;
        }
    }
    dbg[42] = (float)((fabsf(lqr_state.u[LQR_U_WL]) >= lqr_debug.trq_max_wheel ? 1u : 0u) | (fabsf(lqr_state.u[LQR_U_WR]) >= lqr_debug.trq_max_wheel ? 2u : 0u) | (fabsf(lqr_state.u[LQR_U_BL]) >= lqr_debug.trq_max_hip ? 4u : 0u) | (fabsf(lqr_state.u[LQR_U_BR]) >= lqr_debug.trq_max_hip ? 8u : 0u));
    dbg[43] = lqr_state.ds_raw - lqr_state.ds_kf;
    dbg[44] = leg_balance.turn_ff;
    dbg[45] = (float)control_condition.condition;
    dbg[46] = (float)leg_balance.compensation.saturated;
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
#if 0 /* 上一版布局保留 */
    /* 十态分项诊断v1：8个状态列对四路输出(WL/WR/BL/BR)的贡献 + 输入/状态。
     * 省略恒零的s/phi两列；每行8项之和应约等于对应u（u_eq当前为0）。
     * 4～35按输出分组，每组顺序 ds、dphi、thL、dthL、thR、dthR、pitch、dpitch。 */
    taskENTER_CRITICAL();
    turn_scale = motion_guard.reference_scale;
    speed_error = lqr_state.target[LQR_X_DS] + LQR_Gain_Info()->x_eq[LQR_X_DS] - lqr_state.x[LQR_X_DS];
    dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (lqr_state.air_control ? 4u : 0u) | (lqr_state.air_release ? 8u : 0u));
    if (lqr_state.valid && lqr_state.gain_valid && !lqr_state.air_control)
    {
        for (uint8_t i = 0u; i < LQR_U_NUM; i++)
        {
            dbg[4u + i * 8u] = lqr_state.K[i][LQR_X_DS] * speed_error;
            dbg[5u + i * 8u] = lqr_debug.yaw_rate_hold ? lqr_state.K[i][LQR_X_DPHI] * yaw_rate_error : 0.0f;
            for (uint8_t k = 2u; k < 8u; k++)
            {
                col = (uint8_t)(LQR_X_THL + (k - 2u));
                dbg[4u + i * 8u + k] = lqr_state.K[i][col] * (lqr_state.target[col] + LQR_Gain_Info()->x_eq[col] - lqr_state.x[col]);
            }
        }
    }
    dbg[36] = rc_command.velocity_target;
    dbg[37] = lqr_state.x[LQR_X_DS];
    dbg[38] = rc_command.yaw_rate_target;
    dbg[39] = lqr_state.x[LQR_X_DPHI];
    dbg[40] = lqr_state.x[LQR_X_THL];
    dbg[41] = lqr_state.x[LQR_X_THR];
    dbg[42] = lqr_state.x[LQR_X_THB];
    dbg[43] = lqr_state.roll;
    dbg[44] = lqr_state.len[0];
    dbg[45] = lqr_state.u[LQR_U_BL];
    dbg[46] = lqr_state.u[LQR_U_WL];
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
#if 0 /* 上一版停车积分布局，人工切换时恢复相应局部变量声明。 */
    /* 固定48路：停车积分是否激起转向退出后的低频摆动。 */
    taskENTER_CRITICAL();
    turn_scale = motion_guard.reference_scale;
    dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (lqr_state.air_control ? 4u : 0u) | (lqr_state.air_release ? 8u : 0u));
    dbg[4] = lqr_state.pos;
    dbg[5] = lqr_state.target[LQR_X_S] + LQR_Gain_Info()->x_eq[LQR_X_S];
    dbg[6] = dbg[5] - lqr_state.x[LQR_X_S];
    dbg[7] = lqr_state.target[LQR_X_DS];
    dbg[8] = lqr_state.x[LQR_X_DS];
    dbg[9] = turn_scale * (lqr_state.target[LQR_X_DPHI] + LQR_Gain_Info()->x_eq[LQR_X_DPHI]);
    dbg[10] = lqr_state.x[LQR_X_DPHI];
    dbg[11] = (float)lqr_state.pos_trace;
    dbg[12] = (float)lqr_state.pos_armed;
    dbg[13] = (float)lqr_debug.pos_hold;
    dbg[14] = lqr_debug.pos_leg_tol;
    dbg[15] = lqr_debug.pos_arm_vel;
    dbg[16] = Angle_Wrap_Pi(lqr_state.x[LQR_X_THL] - machine->lqr.reference.leg_trim[0]);
    dbg[17] = Angle_Wrap_Pi(lqr_state.x[LQR_X_THR] - machine->lqr.reference.leg_trim[1]);
    dbg[18] = leg_l.output.virtual_leg_angle;
    dbg[19] = leg_r.output.virtual_leg_angle;
    dbg[20] = lqr_state.x[LQR_X_DTHL];
    dbg[21] = lqr_state.x[LQR_X_DTHR];
    dbg[22] = lqr_state.x[LQR_X_THB];
    dbg[23] = lqr_state.x[LQR_X_DTHB];
    dbg[24] = lqr_state.len[0];
    dbg[25] = lqr_state.len[1];
    dbg[26] = leg_balance.turn_ff;
    dbg[27] = leg_balance.roll_height.lqr.height_trim;
    for (row = 0u; row < LQR_U_NUM; row++)
    {
        dbg[28u + row] = lqr_state.valid && lqr_state.gain_valid && !lqr_state.air_control && lqr_debug.pos_hold ? lqr_state.K[row][LQR_X_S] * dbg[6] : 0.0f;
        dbg[40u + row] = lqr_state.u[row];
    }
    for (row = LQR_U_BL; row <= LQR_U_BR; row++)
    {
        if (lqr_state.valid && lqr_state.gain_valid && !lqr_state.air_control)
        {
            dbg[32u + row - LQR_U_BL] = lqr_state.K[row][LQR_X_DS] * (lqr_state.target[LQR_X_DS] + LQR_Gain_Info()->x_eq[LQR_X_DS] - lqr_state.x[LQR_X_DS]);
            for (col = LQR_X_THL; col <= LQR_X_DTHR; col++)
            {
                contribution = lqr_state.K[row][col] * (lqr_state.target[col] + LQR_Gain_Info()->x_eq[col] - lqr_state.x[col]);
                if (col == LQR_X_THL || col == LQR_X_THR)
                {
                    dbg[34u + row - LQR_U_BL] += contribution;
                }
                else
                {
                    dbg[36u + row - LQR_U_BL] += contribution;
                }
            }
            dbg[38u + row - LQR_U_BL] = lqr_state.K[row][LQR_X_THB] * (lqr_state.target[LQR_X_THB] + LQR_Gain_Info()->x_eq[LQR_X_THB] - lqr_state.x[LQR_X_THB]) + lqr_state.K[row][LQR_X_DTHB] * (lqr_state.target[LQR_X_DTHB] + LQR_Gain_Info()->x_eq[LQR_X_DTHB] - lqr_state.x[LQR_X_DTHB]);
        }
    }
    dbg[44] = (float)((fabsf(lqr_state.u[LQR_U_WL]) >= lqr_debug.trq_max_wheel ? 1u : 0u) | (fabsf(lqr_state.u[LQR_U_WR]) >= lqr_debug.trq_max_wheel ? 2u : 0u) | (fabsf(lqr_state.u[LQR_U_BL]) >= lqr_debug.trq_max_hip ? 4u : 0u) | (fabsf(lqr_state.u[LQR_U_BR]) >= lqr_debug.trq_max_hip ? 8u : 0u));
    dbg[45] = (float)leg_balance.compensation.saturated;
    dbg[46] = (float)control_condition.condition;
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
#if 0 /* 上一版台阶检测布局，人工切换保留。 */
    /* 台阶负向判据48路：22/23为世界角加速度，暂停时为旧值，8的PAUSED位标记。 */
    taskENTER_CRITICAL();
    dbg[1] = (float)(state_bits | (Stair_Process_Owns_Output(&stair_process) ? 0x400u : 0u) | (torque_output_enabled ? 0x800u : 0u));
    dbg[3] = (float)control_condition.condition;
    dbg[4] = (float)control_condition.valid;
    dbg[5] = (float)machine->stair.mode;
    dbg[6] = (float)stair_process.phase;
    dbg[7] = (float)stair_process.exit_reason;
    dbg[8] = (float)(stair_detection.flags | ((uint16_t)stair_process.result << 8u) | (machine->stair.process.confirmed ? 0x1000u : 0u));
    dbg[9] = stair_detection.elapsed;
    dbg[10] = stair_process.elapsed;
    dbg[11] = stair_process.total_elapsed;
    dbg[12] = machine->stair.detection.angle_threshold;
    dbg[13] = machine->stair.detection.acceleration_threshold;
    dbg[14] = stair_detection.deviation[0];
    dbg[15] = stair_detection.deviation[1];
    dbg[16] = machine->stair.detection.confirm_time;
    dbg[17] = machine->stair.detection.acceleration_tau;
    dbg[18] = leg_l.output.virtual_leg_angle;
    dbg[19] = leg_r.output.virtual_leg_angle;
    dbg[20] = leg_l.output.d_virtual_leg_angle;
    dbg[21] = leg_r.output.d_virtual_leg_angle;
    dbg[22] = stair_detection.acceleration[0].out;
    dbg[23] = stair_detection.acceleration[1].out;
    dbg[24] = leg_l.output.virtual_leg_length;
    dbg[25] = leg_r.output.virtual_leg_length;
    dbg[26] = stair_process.length_target[0];
    dbg[27] = stair_process.length_target[1];
    dbg[28] = stair_process.angle_ramp[0].out;
    dbg[29] = stair_process.angle_ramp[1].out;
    dbg[30] = lqr_state.ds_kf; /* 融合前向速度m/s，台阶接管时冻结。 */
    dbg[31] = machine->stair.detection.minimum_forward_speed;
    dbg[32] = stair_process.torque[0];
    dbg[33] = stair_process.torque[1];
    dbg[34] = imu_state.pitch_world;
    dbg[35] = imu_state.gyro_rad_s[1];
    for (uint8_t i = 0u; i < 4u; i++)
    {
        dbg[36u + i] = output_dm_torque_nm[i];
        dbg[40u + i] = motor_state.dm.trq_nm[i];
    }
    dbg[44] = output_wheel_torque_nm[0];
    dbg[45] = output_wheel_torque_nm[1];
    dbg[46] = (float)stair_detection.events;
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
#endif
    /* 台阶动作参数/跟踪48路：仅观察既有状态；几何摆角目标不用pitch_world相减。 */
    taskENTER_CRITICAL();
    dbg[1] = (float)(state_bits | (Stair_Process_Owns_Output(&stair_process) ? 0x400u : 0u) | (torque_output_enabled ? 0x800u : 0u));
    dbg[3] = (float)stair_process.phase;
    dbg[4] = (float)stair_process.exit_reason;
    dbg[5] = (float)stair_process.result;
    dbg[6] = stair_process.elapsed;
    dbg[7] = stair_process.total_elapsed;
    dbg[8] = machine->stair.process.angle_rate;
    dbg[9] = fallen_process_param.length_rate;
    dbg[10] = fallen_process_param.angle_speed_max;
    dbg[11] = machine->hip.dm_trq_clamp;
    dbg[12] = lqr_state.ds_kf; /* m/s；台阶接管时普通估计暂停，此值冻结。 */
    dbg[13] = rc_command.velocity_target; /* m/s；仅对照松杆，不参与检测门槛。 */
    dbg[14] = fallen_process_param.angle_speed_kp;
    dbg[15] = fallen_process_param.angle_speed_kd;
    dbg[16] = fallen_process_param.rear_angle;
    dbg[17] = fallen_process_param.retract_len;
    dbg[18] = leg_l.output.virtual_leg_angle;
    dbg[19] = leg_r.output.virtual_leg_angle;
    dbg[20] = leg_l.output.d_virtual_leg_angle;
    dbg[21] = leg_r.output.d_virtual_leg_angle;
    dbg[22] = leg_l.output.virtual_leg_length;
    dbg[23] = leg_r.output.virtual_leg_length;
    dbg[24] = leg_l.output.d_virtual_leg_length;
    dbg[25] = leg_r.output.d_virtual_leg_length;
    dbg[26] = stair_process.length_target[0];
    dbg[27] = stair_process.length_target[1];
    dbg[28] = stair_process.angle_ramp[0].out;
    dbg[29] = stair_process.angle_ramp[1].out;
    dbg[30] = (float)control_condition.condition;
    dbg[31] = (float)((control_condition.valid ? 1u : 0u) | (control_condition.normal_ready ? 2u : 0u) |
        (air_detection.valid ? 4u : 0u) | (air_detection.initialized ? 8u : 0u) |
        (air_detection.flight ? 16u : 0u) | (air_detection.applied ? 32u : 0u));
    dbg[32] = air_detection.valid ? air_detection.mean_force : 0.0f; /* N；无效时0仅占位。 */
    dbg[33] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) |
        (lqr_state.air_control ? 4u : 0u) | (lqr_state.air_release ? 8u : 0u));
    dbg[34] = imu_state.pitch_world;
    dbg[35] = imu_state.gyro_rad_s[1];
    for (uint8_t i = 0u; i < 4u; i++)
    {
        dbg[36u + i] = output_dm_torque_nm[i];
        dbg[40u + i] = motor_state.dm.trq_nm[i];
    }
    dbg[44] = output_wheel_torque_nm[0];
    dbg[45] = output_wheel_torque_nm[1];
    dbg[46] = (float)stair_process.sequence;
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
    Vofa_Send(dbg, VOFA_MAX_CH);
}
