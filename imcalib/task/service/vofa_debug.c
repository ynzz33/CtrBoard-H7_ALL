/*
 * 在短临界区复制控制诊断到浮点通道数组，退出临界区后交给Vofa_Send。
 * 当前有效布局为双MPC性能布局（46号=5005）。
 */

#include "vofa_debug.h"
#include "task_balance_mpc.h"
#include "robot_state.h"
#include "dm.h"
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

    taskENTER_CRITICAL();
    dbg[3]=(float)control_condition.condition;
    dbg[4]=(float)((MACHINE_BALANCE_USE_MPC ? 1u : 0u) | (MACHINE_ROLL_HEIGHT_USE_MPC ? 2u : 0u) | (lqr_state.valid ? 4u : 0u) | (lqr_state.gain_valid ? 8u : 0u) | (output_task_lqr_engaged() ? 16u : 0u) | (torque_output_enabled ? 32u : 0u) | (rc_command.mode.engage ? 64u : 0u) | (rc_command.online ? 128u : 0u) | (lqr_state.air_control || lqr_state.air_release ? 256u : 0u) | (output_debug_dm_sent ? 512u : 0u) | (output_debug_dji_sent ? 1024u : 0u) | (leg_balance.compensation.saturated ? 2048u : 0u) | (MACHINE_MPC_REFERENCE_PREDICT ? 4096u : 0u) | (lqr_state.pos_armed ? 16384u : 0u) | (balance_mpc_debug.ready ? 32768u : 0u) | ((uint32_t)BALANCE_MPC_N<<16u));
    dbg[5]=(float)(1000u*(uint32_t)balance_mpc_debug.status+10u*(uint32_t)balance_mpc_debug.reason+(uint32_t)balance_mpc_debug.backend);
    dbg[6]=balance_mpc_debug.backend==1u ? (float)balance_mpc_debug.age_us*.001f : -1.0f;
    dbg[7]=lqr_state.x[LQR_X_THB];dbg[8]=lqr_state.target[LQR_X_THB];dbg[9]=lqr_state.x[LQR_X_DTHB];
    dbg[10]=lqr_state.target[LQR_X_DS];dbg[11]=lqr_state.x[LQR_X_DS];
    dbg[12]=leg_balance.roll_height.x[0];dbg[13]=leg_balance.roll_height.x[1];
    dbg[14]=leg_balance.roll_height.x[2];dbg[15]=leg_balance.roll_height.target[2];
    dbg[16]=leg_balance.roll_height.x[3];dbg[17]=leg_balance.roll_height.target[3];
    dbg[18]=leg_balance.F[0];dbg[19]=leg_balance.F[1];
    /* ch20~23 改为腿长不对称诊断：左右前/后髋角。
     * 同目标下若左右髋角不等即为髋标定/指令差异；髋角相等而腿长(ch44/45)不等则为几何解算。
     * ch46=5007标识此布局。 */
    dbg[20]=lqr_state.x[3];
    dbg[21]=lqr_state.target[3];
    dbg[22]=leg_r.input.hip_f;
    dbg[23]=leg_r.input.hip_b;
    dbg[24]=leg_balance.roll_height.permit && leg_balance.roll_height.valid ? (float)leg_balance.roll_height.mpc.backend : -1.0f;
    dbg[25]=(float)leg_balance.roll_height.mpc.result.status;dbg[26]=(float)leg_balance.roll_height.mpc.result.iterations;
    dbg[27]=leg_balance.roll_height.mpc.result.primal;dbg[28]=leg_balance.roll_height.mpc.result.dual;
    dbg[29]=(float)leg_balance.roll_height.mpc.elapsed_us;
    dbg[30]=(float)balance_mpc_debug.iterations;dbg[31]=(float)balance_mpc_debug.solve_us;
    dbg[32]=balance_mpc_debug.primal;dbg[33]=balance_mpc_debug.dual;
    dbg[34]=(float)balance_mpc_debug.worker_period_us;dbg[35]=(float)balance_mpc_debug.worker_deadline_count;
    dbg[36]=(float)balance_mpc_debug.model_us;dbg[37]=(float)leg_balance.roll_height.mpc.fallback_count;
    for(uint8_t j=0u;j<4u;j++)
    {
        dbg[38u+j]=output_dm_torque_nm[j];
    }
    dbg[42]=output_wheel_torque_nm[0];dbg[43]=output_wheel_torque_nm[1];
    dbg[44]=lqr_state.len[0];dbg[45]=lqr_state.len[1];
    dbg[46]=5007.0f;dbg[47]=(float)HAL_GetTick();
    taskEXIT_CRITICAL();
    Vofa_Send(dbg, VOFA_MAX_CH);
}
