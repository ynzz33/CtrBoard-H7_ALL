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
#include "Vofa_send.h"
#include "dr16.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

void Vofa_Debug_Send(void)
{
    static float dbg[VOFA_MAX_CH];
    static uint32_t observation_sequence;
    uint8_t online_mask;
    uint16_t state_bits;

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

    /* 本轮只替换VOFA通道：后台读取已有最新值，不增加控制拍采集或发布接口。
     * 这些值来自不同生产任务，不保证与LQR本拍输入同一物理采样时刻。
     * 当前通道含义以本函数赋值为准；31~34为四元数，42~44为含重力比力g。
     */
    taskENTER_CRITICAL();
    dbg[3] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT] * machine->wheel_r;
    dbg[4] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT] * machine->wheel_r;
    dbg[5] = motor_state.dji.angle_total_rad[DJI_MOTOR_WHEEL_LFT];
    dbg[6] = motor_state.dji.angle_total_rad[DJI_MOTOR_WHEEL_RGT];
    dbg[7] = 0.5f * (dbg[3] + dbg[4]); /* 原始轮线速度均值m/s */
    dbg[8] = lqr_state.whl[0];        /* LQR已补偿的左轮角速度rad/s */
    dbg[9] = lqr_state.whl[1];        /* LQR已补偿的右轮角速度rad/s */
    dbg[10] = lqr_state.roll;         /* LQR横滚角rad */
    dbg[11] = lqr_state.ds_raw;       /* 运动学速度m/s */
    dbg[12] = lqr_state.a_fwd;        /* IMU前向加速度m/s^2 */
    dbg[13] = lqr_state.ds_kf;        /* 融合速度m/s */
    dbg[14] = imu_state.accel_vertical; /* 已有竖直加速度m/s^2 */
    dbg[15] = imu_state.gyro_rad_s[0];
    dbg[16] = imu_state.gyro_rad_s[2];
    dbg[17] = lqr_state.x[LQR_X_THB];
    dbg[18] = imu_state.gyro_rad_s[1]; /* 当前pitch原始角速度rad/s */
    dbg[19] = lqr_state.x[LQR_X_DTHB]; /* pitch低通后角速度rad/s */
    dbg[20] = leg_l.output.virtual_leg_angle;
    dbg[21] = leg_l.output.d_virtual_leg_angle;
    dbg[22] = leg_r.output.virtual_leg_angle;
    dbg[23] = leg_r.output.d_virtual_leg_angle;
    dbg[24] = leg_l.output.virtual_leg_length;
    dbg[25] = leg_r.output.virtual_leg_length;
    dbg[26] = leg_l.output.d_virtual_leg_length;
    dbg[27] = leg_r.output.d_virtual_leg_length;
    dbg[28] = lqr_state.x[LQR_X_S];
    dbg[29] = lqr_state.target[LQR_X_DS];
    dbg[30] = lqr_state.target[LQR_X_DPHI];
    for (uint8_t i = 0u; i < 4u; i++)
    {
        dbg[31u + i] = imu_state.quat[i]; /* 已映射/归一化，w/x/y/z */
    }
    dbg[35] = (float)lqr_state.air_control;
    dbg[36] = (float)lqr_state.valid;
    dbg[37] = sqrtf(imu_state.acc_g[0] * imu_state.acc_g[0] +
                   imu_state.acc_g[1] * imu_state.acc_g[1] +
                   imu_state.acc_g[2] * imu_state.acc_g[2]);
    dbg[38] = (float)imu_state.last_timestamp_ms; /* IMU模块时间戳ms */
    dbg[39] = lqr_state.u[LQR_U_WL];
    dbg[40] = lqr_state.u[LQR_U_WR];
    dbg[41] = lqr_state.u[LQR_U_BL];
    for (uint8_t i = 0u; i < 3u; i++)
    {
        dbg[42u + i] = imu_state.acc_g[i]; /* 已映射三轴比力g，含重力 */
    }
    dbg[45] = lqr_state.u[LQR_U_BR];
    dbg[46] = (float)HAL_GetTick();    /* 本次后台读取时间ms */
    dbg[47] = (float)++observation_sequence; /* VOFA调用序号，非控制拍序号 */
    taskEXIT_CRITICAL();
    // dbg[23] = lqr_state.a_fwd;

    /* RL布局参考：obs39及恢复诊断，切换观测时手动启用。 */
    // taskENTER_CRITICAL();
    // for (uint8_t i = 0u; i < RL_OBS_SIZE; i++)
    // {
    //     dbg[3u + i] = rl_control.observation.obs[i];
    // }
    // // dbg[35] = (float)motor_state.dm.pos_continuous_rad[0];
    // // dbg[36] = (float)motor_state.dm.pos_continuous_rad[1];
    // // dbg[37] = (float)motor_state.dm.pos_continuous_rad[2];
    // // dbg[38] = (float)motor_state.dm.pos_continuous_rad[3];
    // dbg[42] = (float)rl_control.policy.run_us;
    // dbg[43] = (float)rl_control.recovery.phase; /* 0未投入/1启动保持/2恢复/3正常/4失败。 */
    // dbg[44] = robot_state.height;
    // dbg[45] = (float)robot_state.height_valid;
    // dbg[46] = (float)rl_control.recovery.stable_ms;
    // dbg[47] = (float)rl_control.recovery.fallen_ms;
    // taskEXIT_CRITICAL();

    Vofa_Send(dbg, VOFA_MAX_CH);
}
