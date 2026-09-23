#include "robot_control.h"
#include "Attitude_Algorithm.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"
#include "Vofa_send.h"
#include "ws2812.h"
#include "task.h"
#include "../Sysid/sysid_config.h"
#if SYSID_ENABLE
#include "../Sysid/sysid_log.h"
#endif

#include <math.h>

/* 更新电机状态 */
static void Motor_State_Update(void)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];

        motor_state.dm.pos_rad[i] = feedback->pos_rad;
        motor_state.dm.pos_zero_rad[i] = feedback->pos_zero_rad;
        motor_state.dm.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dm.trq_nm[i] = feedback->trq_nm;
        motor_state.dm.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

        motor_state.dji.angle_rad[i] = feedback->angle_rad;
        motor_state.dji.angle_total_rad[i] = feedback->angle_total_rad;
        motor_state.dji.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dji.current_raw[i] = feedback->current_raw;
        motor_state.dji.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dji.online[i] = (uint8_t)Dji_Is_Online(i);
    }
    motor_state.timestamp_ms = HAL_GetTick();
    motor_state.updated = 1u;
}

/* 更新腿部状态 */
static void Leg_State_Update(void)
{
    if (leg_map_l.configured)
    {
        leg_l.input.hip_f = motor_state.dm.pos_zero_rad[leg_map_l.dm_front] + LEG_PI;
        leg_l.input.hip_b = motor_state.dm.pos_zero_rad[leg_map_l.dm_rear];
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_zero_rad[leg_map_r.dm_front] + LEG_PI;
        leg_r.input.hip_b = motor_state.dm.pos_zero_rad[leg_map_r.dm_rear];
        leg_r.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_r.dm_front];
        leg_r.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_r.dm_rear];
    }
    vTaskSuspendAll();
    (void)Leg_Solve(&leg_l);
    (void)Leg_Solve(&leg_r);
    (void)xTaskResumeAll();
}

/* 更新遥控: 指令解算 + 使能 */
static void Remote_Control_Update(void)
{
    dr16_t remote;

    DR16_Process();
    remote = DR16_Snapshot();
    Rc_Command_Update(&rc_command, &remote);
    robot_state.rc_enable = (uint8_t)(remote.online && remote.s1 != DR16_SW_DOWN);
    input_command.mode = remote.online ? remote.s1 : 0u;
}

/* 更新故障状态 */
static void Robot_Fault_Update(void)
{
    uint32_t fault;
    uint8_t motors_ok;

    fault = FAULT_NONE;
    motors_ok = 1u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i] || Dm_Has_Fault(i))
        {
            motors_ok = 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i])
        {
            motors_ok = 0u;
        }
    }
    if (!imu_state.online)
    {
        fault |= FAULT_IMU;
    }
    if (!DR16_Online())
    {
        fault |= FAULT_RC;
    }
    if (!Can_Bus_Online(true))
    {
        fault |= FAULT_CAN;
    }
    if (!motors_ok)
    {
        fault |= FAULT_MOTOR;
    }
    if (robot_state.rc_enable
        && (!action_state.updated || HAL_GetTick() - action_state.last_ok_tick >= 100u))
    {
        fault |= FAULT_ACTION;
    }
    ctrl_fault = fault;
}

/* 更新翻倒状态 */
static void Robot_Fallen_Update(void)
{
    float pitch_abs;

    pitch_abs = fabsf(imu_state.euler_rad[ATTITUDE_PITCH]);
    if (pitch_abs > 1.4f)
    {
        robot_state.fallen = 1u;
    }
    else if (pitch_abs < 1.0f)
    {
        robot_state.fallen = 0u;
    }
}

/* 更新使能状态: 使能沿发使能, 失能沿发失能; 两个方向都有看门狗兜底 */
static void Robot_Enable_Update(void)
{
    uint8_t enable_request;

    enable_request = (uint8_t)(robot_state.rc_enable
        && ctrl_fault == FAULT_NONE && !robot_state.fallen);
    if (enable_request && !robot_state.motor_enabled)
    {
        robot_state.motor_enabled = 1u;
        (void)Dm_All_Enable();
    }
    else if (!enable_request && robot_state.motor_enabled)
    {
        robot_state.motor_enabled = 0u;
        (void)Dji_All_Stop();
        (void)Dm_All_Disable();
    }

    if (robot_state.motor_enabled)
    {
        Dm_Enable_Watchdog();
    }
    else
    {
        Dm_Disable_Watchdog();
    }
}

/*
 * VOFA 观测帧 (JustFloat, 32 通道) — 当前为"LQR 分层验证"帧 (2026-09-23 第六版)
 * ch0  在线掩码: bit0 IMU / bit1 遥控 / bit2~5 髋(前左后左前右后右) / bit6~7 轮(左/右)
 * ch1  状态位: bit0 使能 / bit1 跌倒 / bit2 左腿有效 / bit3 右腿有效 / bit4~7 四髋使能 / bit8 LQR·手动腿测已投入 / bit9 RL 已投入
 * ch2  策略号: 0 RL / 1 LQR / 2 测试 / 3 手动腿测 / 4 失能
 * ch3~4   偏航角 x[2] / 偏航角速度 x[3] (rad, rad/s)
 * ch5~6   位移积分 pos (m) / 偏航角目标 yaw_tgt (rad)
 * ch7~8   解算摆角 (rad, 机体系, 前摆为正): 左 / 右      ← 腿竖直时读零位
 * ch9~10  腿长 (m): 左 / 右
 * ch11~12 轮对地角速度估计 whl[] (rad/s): 左 / 右          ← 卡轮摇车身应贴 0 (pitch_comp_sign A/B)
 * ch13    速度目标 vel_tgt (m/s, 斜坡后)
 * ch14~15 左/右轮电流目标 (DJI raw 指令值, 已应用输出极性)
 * ch16~17 轮速反馈 (rad/s): 左 / 右
 * ch18~19 轮电流反馈 raw: 左 / 右                          ← 电流涨轮速不动 = 静摩擦
 * ch20~21 足端力 F (N): 左 / 右
 * ch22~23 虚拟髋扭矩 Tp (N·m): 左 / 右
 * ch24~25 LQR 轮输出原值 u (N·m, 未门控未限幅, 只在 LQR 投入时更新): 左 / 右   ← 轮不出力也能看方向
 * ch26    速度估计 x[1] (m/s, LQR 实际吃到的: vel_src=1 卡尔曼 / 0 低通)
 * ch27    左腿摆角世界系 x[4] (rad)                          ← 匀速漂移段与 ch28 对比定偏置来源
 * ch28~29 俯仰角 (rad, 已扣 pitch_off) / 俯仰角速度 (rad/s), LQR 吃到的
 * ch30    横滚角 (rad, LQR 吃到的)                        ← 车身摆平时应≈0
 * ch31    横滚补偿力 roll.pos_out (N, 左腿 +/右腿 −)      ← 查腿长左右差
 * ch3~14、26~30 由每拍必算的状态估计更新 (需 IMU 在线 + 两腿有效); ch20~25、31 未投入时为 0
 * 左轮十列分项 lqr_state.u_col[0..9] 在调试器 Watch 看
 *
 * RL 输出/反馈帧 (左拨杆上位):
 * ch0 在线掩码; ch1 状态位; ch2 RL 状态位(网络/历史/动作/投入/阶段/观测/映射/失败数/bit16 DM发送成功/bit17 DJI发送成功)
 * ch3~8 RL 逻辑力矩请求(N·m, 驱动器符号处理前): DM 前左/后左/前右/后右, 训练轮左/右
 * ch9~12 DM 电机力矩反馈(N·m), 已应用反馈极性, 按 DM 电机枚举序
 * ch13~16 训练空间腿角观测(q−default, rad): lf0/lf1/rf0/rf1
 * ch17~22 训练空间关节速度观测(rad/s × 0.05): lf0/lf1/lfwheel/rf0/rf1/rfwheel
 * ch23~28 上一拍训练动作: lf0/lf1/lfwheel/rf0/rf1/rfwheel
 * ch29~30 DJI 电流反馈(raw, 已应用反馈极性): 物理左/右 (反馈源按台架发现的左右交叉重排)
 * ch31 推理耗时(us)
 */
static void Robot_Control_Send_Vofa(void)
{
    static float dbg[VOFA_MAX_CH];
    uint8_t online_mask;
    uint16_t state_bits;

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

    /* ch2 非 RL 模式为策略号; RL 输出测试模式为状态位 */
    dbg[2] = (float)ctrl_strategy;

    // dbg[3] = lqr_state.x[LQR_X_PHI];
    // dbg[4] = lqr_state.x[LQR_X_DPHI];
    // dbg[5] = lqr_state.pos;
    // dbg[6] = lqr_state.yaw_tgt;

    // /* ch7~10 腿解算: 摆角, 腿长 */
    // dbg[7]  = leg_l.output.virtual_leg_angle;
    // dbg[8]  = leg_r.output.virtual_leg_angle;
    // dbg[9]  = leg_l.output.virtual_leg_length;
    // dbg[10] = leg_r.output.virtual_leg_length;

    // /* ch11~14 轮对地角速度 左/右, 速度目标(斜坡后), 位移积分已启动 */
    // dbg[11] = lqr_state.whl[0];
    // dbg[12] = lqr_state.whl[1];
    // dbg[13] = lqr_state.vel_tgt;

    // dbg[14] = wheel_current[0];
    // dbg[15] = wheel_current[1];

    // /* ch16~19 轮速 左/右, 轮电流 raw 左/右 */
    // dbg[16] = motor_state.dji.vel_rad_s[0];
    // dbg[17] = motor_state.dji.vel_rad_s[1];
    // dbg[18] = motor_state.dji.current_raw[0];
    // dbg[19] = motor_state.dji.current_raw[1];

    // /* ch20~23 力向量: F 左/右 (N), Tp 左/右 (N·m) */
    // dbg[20] = leg_balance.F[0];
    // dbg[21] = leg_balance.F[1];
    // dbg[22] = leg_balance.Tp[0];
    // dbg[23] = leg_balance.Tp[1];

    // /* ch24~25 LQR 轮输出原值 (N·m) */
    // dbg[24] = lqr_state.u[LQR_U_WL];
    // dbg[25] = lqr_state.u[LQR_U_WR];

    // /* ch26~27 速度估计 / 左腿摆角世界系 */
    // dbg[26] = lqr_state.x[LQR_X_DS];
    // dbg[27] = lqr_state.x[LQR_X_THL];

    // /* ch28~29 俯仰角 / 俯仰角速度 */
    // dbg[28] = lqr_state.x[LQR_X_THB];
    // dbg[29] = lqr_state.x[LQR_X_DTHB];

    // /* ch30~31 横滚角 / 横滚补偿力 */
    // dbg[30] = lqr_state.roll;
    // dbg[31] = leg_balance.roll.pos_out;     /* 横滚补偿力 */

        uint32_t rl_bits;

        rl_bits  = rl_control.policy.ready ? 0x01u : 0x00u;
        rl_bits |= rl_control.observation.history_ready ? 0x02u : 0x00u;
        rl_bits |= action_state.rl_ready ? 0x04u : 0x00u;
        rl_bits |= output_task_rl_engaged() ? 0x08u : 0x00u;
        rl_bits |= (uint32_t)rl_control.infer_phase << 4;
        rl_bits |= rl_control.observation.valid ? 0x40u : 0x00u;
        rl_bits |= machine->rl.configured ? 0x80u : 0x00u;
        rl_bits |= (rl_control.policy.run_fail & 0xFFu) << 8;
        rl_bits |= output_debug_dm_sent ? 0x00010000u : 0x00u;
        rl_bits |= output_debug_dji_sent ? 0x00020000u : 0x00u;
        dbg[2] = (float)rl_bits;
        for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
        {
            dbg[3u + i] = rl_output_dm_cmd_nm[i];
            dbg[9u + i] = motor_state.dm.trq_nm[i];
            dbg[13u + i] = rl_control.observation.obs[RL_OBS_L_THIGH + i];
        }
        dbg[7] = rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_LFT];
        dbg[8] = rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_RGT];
        for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
        {
            dbg[29u + i] = motor_state.dji.current_raw[
                (i == DJI_MOTOR_WHEEL_LFT) ? DJI_MOTOR_WHEEL_RGT : DJI_MOTOR_WHEEL_LFT];
        }
        for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
        {
            dbg[17u + i] = rl_control.observation.obs[RL_OBS_L_THIGH_VEL + i];
            dbg[23u + i] = rl_control.observation.obs[RL_OBS_LAST_ACTION + i];
        }
        dbg[31] = (float)rl_control.policy.run_us;

    Vofa_Send(dbg, 32u);
}

/* 通信单周期 */
void comm_task_body(void)
{
    Dm_Parse();
    Dji_Parse();
    Motor_State_Update();
    Leg_State_Update();
    WS2812_RainbowBlink();
    Remote_Control_Update();
    Robot_Fallen_Update();
    Robot_Fault_Update();
    Robot_Enable_Update();
    Robot_Control_Send_Vofa();
}
