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
    const lqr_gain_desc_t *gain;
    float term;
    uint8_t group;

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

    /* LQR静止抖动分项：只读已有K/状态，不重算增益、不推进控制器。
     * 27~46按四输出各五组记录限幅前贡献，单位统一Nm。
     * 世界腿角+pitch即相对机身腿角；长度与状态取自LQR本身。
     */
    taskENTER_CRITICAL();
    gain = LQR_Gain_Info();
    for (uint8_t j = 0u; j < LQR_X_NUM; j++)
    {
        dbg[3u + j] = lqr_state.x[j];
    }
    dbg[13] = lqr_state.target[LQR_X_S] + gain->x_eq[LQR_X_S];
    dbg[14] = lqr_state.target[LQR_X_DS] + gain->x_eq[LQR_X_DS];
    dbg[15] = lqr_state.target[LQR_X_PHI] + gain->x_eq[LQR_X_PHI];
    dbg[16] = lqr_state.target[LQR_X_DPHI] + gain->x_eq[LQR_X_DPHI];
    dbg[17] = lqr_state.target[LQR_X_THL] + gain->x_eq[LQR_X_THL];
    dbg[18] = lqr_state.target[LQR_X_THR] + gain->x_eq[LQR_X_THR];
    dbg[19] = lqr_state.target[LQR_X_THB] + gain->x_eq[LQR_X_THB];
    dbg[20] = lqr_state.len[0];
    dbg[21] = lqr_state.len[1];
    /* bit0状态有效/1增益有效/2 AIR反馈/3轴向释放/4离地/5三态有效/6四髋限幅/7四态有效；bit8~9三态。 */
    dbg[22] = (float)((lqr_state.valid ? 0x01u : 0u) |
                      (lqr_state.gain_valid ? 0x02u : 0u) |
                      (lqr_state.air_control ? 0x04u : 0u) |
                      (lqr_state.air_release ? 0x08u : 0u) |
                      (air_detection.flight ? 0x10u : 0u) |
                      (control_condition.valid ? 0x20u : 0u) |
                      (leg_balance.compensation.saturated ? 0x40u : 0u) |
                      (leg_balance.roll_height.valid ? 0x80u : 0u) |
                      ((uint16_t)control_condition.condition << 8));
    /* 最终门控后的轮矩请求；不能再用未投入时的LQR缓存代表已下发输出。 */
    dbg[23] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_LFT];
    dbg[24] = output_wheel_torque_nm[DJI_MOTOR_WHEEL_RGT];
    dbg[25] = lqr_state.u[LQR_U_BL]; /* 虚拟髋模型输出，尚未组合轴向力/补偿。 */
    dbg[26] = lqr_state.u[LQR_U_BR];
    /* 仅在LQR投入且状态/增益有效时重建；后台读取，不保证与缓存u严格同拍。
     * 每输出：速度、左腿角速度、右腿角速度、pitch角速度、其余状态+u_eq。
     * AIR掩码、开关和角度环绕与LQR_Control_Update保持一致。
     */
    if (output_task_lqr_engaged() && lqr_state.valid && lqr_state.gain_valid)
    {
        for (uint8_t i = 0u; i < LQR_U_NUM; i++)
        {
            dbg[31u + i * 5u] = lqr_state.air_control ? 0.0f : gain->u_eq[i];
            for (uint8_t j = 0u; j < LQR_X_NUM; j++)
            {
                if (lqr_state.air_control && (i == LQR_U_WL || i == LQR_U_WR || j < LQR_X_THL || j > LQR_X_DTHR))
                {
                    term = 0.0f;
                }
                else if (j == LQR_X_PHI)
                {
                    term = lqr_debug.yaw_hold ? lqr_state.K[i][j] * Angle_Wrap_Pi(lqr_state.target[j] + gain->x_eq[j] - lqr_state.x[j]) : 0.0f;
                }
                else if ((j == LQR_X_DPHI && !lqr_debug.yaw_rate_hold) || (j == LQR_X_S && !lqr_debug.pos_hold))
                {
                    term = 0.0f;
                }
                else if (lqr_state.air_control && (j == LQR_X_THL || j == LQR_X_THR))
                {
                    term = lqr_state.K[i][j] * remainderf(lqr_state.target[j] + gain->x_eq[j] - lqr_state.x[j], LEG_2PI);
                }
                else
                {
                    term = lqr_state.K[i][j] * (lqr_state.target[j] + gain->x_eq[j] - lqr_state.x[j]);
                }
                switch (j)
                {
                case LQR_X_DS:   group = 0u; break;
                case LQR_X_DTHL: group = 1u; break;
                case LQR_X_DTHR: group = 2u; break;
                case LQR_X_DTHB: group = 3u; break;
                default:         group = 4u; break;
                }
                dbg[27u + i * 5u + group] += term;
            }
        }
    }
    dbg[47] = (float)HAL_GetTick(); /* 毫秒时间戳；按它识别掉帧，不将记录强当固定周期。 */
    taskEXIT_CRITICAL();

    /* RL布局参考：obs39及恢复诊断，切换观测时手动启用。 */
    // taskENTER_CRITICAL();
    // for (uint8_t i = 0u; i < RL_OBS_SIZE; i++)
    // {
    //     dbg[3u + i] = rl_control.observation.obs[i];
    // }

    // // dbg[35] = (float)motor_state.dm.pos_zero_rad[0];
    // // dbg[36] = (float)motor_state.dm.pos_zero_rad[1];
    // // dbg[37] = (float)motor_state.dm.pos_zero_rad[2];
    // // dbg[38] = (float)motor_state.dm.pos_zero_rad[3];
    // dbg[42] = (float)rl_control.policy.run_us;
    // dbg[43] = (float)rl_control.recovery.phase; /* 0未投入/1启动保持/2恢复/3正常/4失败。 */
    // dbg[44] = robot_state.height;
    // dbg[45] = (float)robot_state.height_valid;
    // dbg[46] = (float)rl_control.recovery.stable_ms;
    // dbg[47] = (float)rl_control.recovery.fallen_ms;
    // taskEXIT_CRITICAL();

    Vofa_Send(dbg, VOFA_MAX_CH);
}
