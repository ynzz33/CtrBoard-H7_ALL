/*
 * 文件职责：普通VOFA状态快照与39通道打包（vofa_debug.c）。
 * 输入/来源：共享反馈/控制/离地/补偿诊断及驱动记录。
 * 输出/去向：按air_param.vofa_page构造JustFloat帧，按发送周期/忙状态提交传输。
 * 关键边界：只读观测，不推进检测或修改目标；异步来源同帧不等于同一物理采样时刻，通道表见md/vofa-debug.md。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "vofa_debug.h"
#include "robot_state.h"
#include "dm.h"
#include "dji.h"
#include "air_detection.h"
#include "Attitude_Algorithm.h"
#include "Vofa_send.h"
#include "mono_ns.h"
#include "dr16.h"

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

    dbg[3] = (float)leg_l.input.hip_f;
    dbg[4] = (float)leg_l.input.hip_b;
    dbg[5] = (float)leg_r.input.hip_f;
    dbg[6] = (float)leg_r.input.hip_b;
    dbg[7] = (float)leg_l.output.virtual_leg_length;
    dbg[8] = (float)leg_l.output.virtual_leg_angle;
    dbg[9] = (float)leg_l.output.virtual_shank_angle;
    dbg[10] = (float)leg_r.output.virtual_leg_length;
    dbg[11] = (float)leg_r.output.virtual_leg_angle;
    dbg[12] = (float)leg_r.output.virtual_shank_angle;
    dbg[13] = (float)leg_l.output.d_virtual_leg_length;
    dbg[14] = (float)leg_l.output.d_virtual_leg_angle;
    dbg[15] = (float)leg_l.output.d_virtual_shank_angle;
    dbg[16] = (float)leg_r.output.d_virtual_leg_length; 
    dbg[17] = (float)leg_r.output.d_virtual_leg_angle;
    dbg[18] = (float)leg_r.output.d_virtual_shank_angle;


    // for(int i = 0;i<10;i++)
    // {
    //     dbg[i + 3] = lqr_state.x[i];
    // }
    // for(int i = 0;i<10;i++)
    // {
    //     dbg[i + 13] = lqr_state.target[i];
    // }
    // dbg[23] = lqr_state.a_fwd;

    Vofa_Send(dbg, VOFA_MAX_CH);
}
