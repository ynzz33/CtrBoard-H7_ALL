/*
 * 文件职责：固定48通道赋值与VOFA发送（vofa_debug.c）。
 * 输入/来源：共享反馈/控制/离地/补偿诊断及驱动记录。
 * 输出/去向：后台每次调用固定赋值后提交一帧；改通道时手动注释旧赋值、启用新赋值。
 * 关键边界：只读观测，不推进检测或修改目标；异步来源同帧不等于同一物理采样时刻，通道表见md/file-guide.md#vofa。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "vofa_debug.h"
#include "robot_state.h"
#include "dm.h"
#include "air_detection.h"
#include "control_detection.h"
#include "control_condition.h"
#include "fallen_process.h"
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
//     dbg[3] = air_debug.mean_force;
//     dbg[4] = (float)air_debug.flight;
//     dbg[5] = (float)air_debug.valid;
//     dbg[6] = (float)air_debug.initialized;
//     dbg[7] = (float)air_debug.fault;
//     dbg[8] = air_debug.elapsed;
//     dbg[9] = air_debug.initial_unload_elapsed;
//     dbg[10] = air_debug.compression[0];
//     dbg[11] = air_debug.compression[1];
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
//     dbg[45] = (float)(air_debug.flight && air_param.compression_enabled && (air_debug.landing_elapsed[0] > 0.0f || air_debug.landing_elapsed[1] > 0.0f));
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
//     dbg[19] = machine->lqr.lpf_alpha[3];
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
//     dbg[46] = machine->lqr.lpf_alpha[2];
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

    /* 腿长差排查布局：只读执行拍已发布状态，不推进控制器。
     * 4~9   左右虚拟腿长、腿长差、腿长均值、左右伸缩速度(m,m/s)
     * 10~13 四髋反馈关节角 左前/左后/右前/右后(rad，已含驱动标定)；用于看机构是否在剪叉
     * 14~16 左右轴向力请求N、向心项N；17偏置共模N
     * 18~20 最终腿长目标、左右斜坡目标m
     * 21~23 机体俯仰rad、左右世界系腿摆角rad
     * 24~26 roll rad、roll角速度原始/滤后 rad/s
     * 27~28 投影高度及斜坡目标m
     * 29~31 实测前进速度m/s、yaw角速度及目标rad/s
     * 32~35 左右轮力矩请求Nm、左右轮反馈电流raw(未加载的轮电流会明显偏小)
     * 36~39 AIR每腿支持力估计N、由反馈反解的每腿电机轴向力N
     * 40~42 左腿腿长雅可比两列(m/rad)、左腿力映射行列式(接近0说明接近奇异)
     * 43~46 四髋最终力矩请求Nm 左前/左后/右前/右后；47毫秒时间
     * 换版本时手工恢复上一段即可，不按模式选帧。 */
    taskENTER_CRITICAL();
    dbg[3] = (float)((lqr_state.valid ? 1u : 0u) | (lqr_state.gain_valid ? 2u : 0u) | (leg_balance.roll_height.valid ? 4u : 0u) | (control_condition.valid ? 8u : 0u) | (lqr_state.air_control ? 16u : 0u) | (lqr_state.air_release ? 32u : 0u) | (air_detection.flight ? 64u : 0u) | (leg_balance.compensation.saturated ? 128u : 0u) | (leg_balance.roll_height.observer.permit ? 256u : 0u) | (leg_balance.roll_height.observer.updated ? 512u : 0u) | (leg_balance.roll_height.observer.limited ? 1024u : 0u) | (leg_balance.roll_height.observer.seeded ? 2048u : 0u) | ((uint32_t)control_condition.condition << 12) | (output_debug_dm_sent ? 16384u : 0u) | (output_debug_dji_sent ? 32768u : 0u));
    dbg[4] = leg_l.output.virtual_leg_length;
    dbg[5] = leg_r.output.virtual_leg_length;
    dbg[6] = leg_l.output.virtual_leg_length - leg_r.output.virtual_leg_length;
    dbg[7] = 0.5f * (leg_l.output.virtual_leg_length + leg_r.output.virtual_leg_length);
    dbg[8] = leg_l.output.d_virtual_leg_length;
    dbg[9] = leg_r.output.d_virtual_leg_length;
    dbg[10] = leg_l.input.hip_f;
    dbg[11] = leg_l.input.hip_b;
    dbg[12] = leg_r.input.hip_f;
    dbg[13] = leg_r.input.hip_b;
    dbg[14] = leg_balance.F[0];
    dbg[15] = leg_balance.F[1];
    dbg[16] = leg_balance.turn_ff;
    dbg[17] = 0.5f * (leg_balance.roll_height.observer.bias[0] + leg_balance.roll_height.observer.bias[1]);
    dbg[18] = lqr_state.leg_length_target[0];
    dbg[19] = leg_balance.length_ramp[0].out;
    dbg[20] = leg_balance.length_ramp[1].out;
    dbg[21] = lqr_state.x[LQR_X_THB];
    dbg[22] = lqr_state.x[LQR_X_THL];
    dbg[23] = lqr_state.x[LQR_X_THR];
    dbg[24] = leg_balance.roll_height.x[0];
    dbg[25] = lqr_state.roll_rate;
    dbg[26] = leg_balance.roll_height.x[1];
    dbg[27] = leg_balance.roll_height.x[2];
    dbg[28] = leg_balance.roll_height.target[2];
    dbg[29] = lqr_state.x[LQR_X_DS];
    dbg[30] = lqr_state.x[LQR_X_DPHI];
    dbg[31] = lqr_state.target[LQR_X_DPHI];
    dbg[32] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT];
    dbg[33] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT];
    dbg[34] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    dbg[35] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    dbg[36] = air_detection.leg[0].force;
    dbg[37] = air_detection.leg[1].force;
    dbg[38] = air_detection.leg[0].motor_force;
    dbg[39] = air_detection.leg[1].motor_force;
    dbg[40] = leg_l.output.leg_jac[0][0];
    dbg[41] = leg_l.output.leg_jac[0][1];
    dbg[42] = leg_l.output.force_det;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        dbg[43u + i] = output_dm_torque_nm[i];
    }
    dbg[47] = (float)HAL_GetTick();
    taskEXIT_CRITICAL();
    Vofa_Send(dbg, VOFA_MAX_CH);
}
