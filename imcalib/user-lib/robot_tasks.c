#include "robot_tasks.h"
#include "dr16.h"
#include "hi229.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "Vofa_send.h"
#include "tim.h"
#include "Attitude_Algorithm.h"
#include "ws2812.h"

#include <math.h>
#include <string.h>

/* 共享状态 */
imu_state_t       imu_state;
motor_state_t     motor_state;
leg_state_t       leg_l;
leg_state_t       leg_r;
action_state_t    action_state;
command_state_t   command_state;
leg_map_t         leg_map_l;
leg_map_t         leg_map_r;
robot_state_t     robot_state;
rl_control_state_t rl_control;
volatile uint32_t ctrl_fault;
uint8_t torque_output_enabled;

/* 500Hz 节拍信号量 */
osSemaphoreDef(ctrl_tick_sem);
osSemaphoreId ctrl_tick_sem_handle = NULL;

static volatile uint32_t tick_count;

/* 清空动作 */
static void Action_State_Clear(void)
{
    memset(action_state.a, 0, sizeof(action_state.a));
    action_state.updated = 0u;
}

/* 电机反馈 -> 五连杆 */
static void Leg_State_Update(void)
{
    if (leg_map_l.configured)
    {
        leg_l.input.hip_f = motor_state.dm.pos_rad[leg_map_l.dm_front]
            + leg_l.config.offset_f;
        leg_l.input.hip_b = motor_state.dm.pos_rad[leg_map_l.dm_rear]
            + leg_l.config.offset_b;
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_rad[leg_map_r.dm_front]
            + leg_r.config.offset_f;
        leg_r.input.hip_b = motor_state.dm.pos_rad[leg_map_r.dm_rear]
            + leg_r.config.offset_b;
        leg_r.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_r.dm_front];
        leg_r.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_r.dm_rear];
    }
    (void)Leg_Solve(&leg_l);
    (void)Leg_Solve(&leg_r);
}

/* 检查电机 */
static uint8_t RL_Motors_Online(void)
{
    uint8_t i;

    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i]) return 0u;
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i]) return 0u;
    }
    return 1u;
}

/* 构建观测 */
static uint8_t RL_Control_Update_Observation(void)
{
    float command[3];       /* 控制指令 */
    float joint_pos[4];     /* 关节角度 */
    float joint_vel[6];     /* 关节速度 */
    uint8_t source_valid;

    command[0] = command_state.vx;
    command[1] = command_state.yaw_rate;
    command[2] = command_state.height;

    joint_pos[0] = leg_l.input.hip_f;
    joint_pos[1] = leg_l.output.virtual_shank;
    joint_pos[2] = leg_r.input.hip_f;
    joint_pos[3] = leg_r.output.virtual_shank;

    joint_vel[0] = leg_l.input.d_hip_f;
    joint_vel[1] = leg_l.output.d_virtual_shank;
    joint_vel[2] = motor_state.dji.vel_rad_s[0];
    joint_vel[3] = leg_r.input.d_hip_f;
    joint_vel[4] = leg_r.output.d_virtual_shank;
    joint_vel[5] = motor_state.dji.vel_rad_s[1];

    source_valid = (uint8_t)(imu_state.online && leg_l.output.valid
        && leg_r.output.valid && RL_Motors_Online());
    if (!RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu_state.input.gyro_rad_s, imu_state.output.quat, command,
        joint_pos, joint_vel, source_valid))
        return 0u;

    RL_Observation_Update_History(&rl_control.observation);
    return rl_control.observation.history_ready;
}

/* 更新电机状态 */
static void Motor_State_Update(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];
        motor_state.dm.pos_rad[i] = feedback->pos_rad;
        motor_state.dm.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dm.trq_nm[i] = feedback->trq_nm;
        motor_state.dm.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }

    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
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

/* 死区 + 限幅 */
static int16_t DR16_Command_Axis(int16_t input)
{
    int16_t value;

    value = DR16_Deadline(input, 20u);
    if (value > DR16_CH_LIMIT)
    {
        value = DR16_CH_LIMIT;
    }
    else if (value < -DR16_CH_LIMIT)
    {
        value = -DR16_CH_LIMIT;
    }
    return value;
}

/* 初始化：建信号量 */
void Robot_Control_Init(void)
{
    uint8_t i;

    ctrl_tick_sem_handle = osSemaphoreCreate(osSemaphore(ctrl_tick_sem), 1);
    Leg_Init(&leg_l);
    Leg_Init(&leg_r);
    leg_l.config.lu = 0.13087f;
    leg_l.config.lg = 0.15240f;
    /* 软件零位 = 电机反馈 - 标定读数 */
    leg_l.config.offset_f = -0.03f;
    leg_l.config.offset_b = -0.04f;
    leg_l.config.mirror = 1;
    leg_l.config.configured = 1u;
    leg_r.config.lu = 0.13087f;
    leg_r.config.lg = 0.15240f;
    leg_r.config.offset_f = -0.038f;
    leg_r.config.offset_b = -0.023f;
    leg_r.config.mirror = 1;
    leg_r.config.configured = 1u;
    leg_map_l.dm_front = DM_MOTOR_LEG_F_LFT;
    leg_map_l.dm_rear = DM_MOTOR_LEG_B_LFT;
    leg_map_l.configured = 1u;
    leg_map_r.dm_front = DM_MOTOR_LEG_F_RGT;
    leg_map_r.dm_rear = DM_MOTOR_LEG_B_RGT;
    leg_map_r.configured = 1u;
    RL_Observation_Init(&rl_control.observation);
    RL_Observation_Param_Init(&rl_control.param);
    RL_Policy_Reset(&rl_control.policy);
    for (i = 0u; i < RL_MODEL_COUNT; i++)
        RL_Torque_Param_Init(&rl_control.torque_param[i], (rl_model_t)i);
    RL_Torque_State_Init(&rl_control.torque_state);
    Action_State_Clear();
    /* 实测确认前保持全关: leg_map / leg config 均未配置 */
}

/* IMU 初始化 */
void imu_task_init(void)
{
    Attitude_Init(&imu_state);
}

/* IMU 单周期 */
void imu_task_body(void)
{
    hi229_data_t s;
    uint8_t first_sample;
    uint8_t new_sample;

    HI229_Process();
    if (!HI229_Online())
    {
        imu_state.online = 0u;
        imu_state.timestamp_valid = 0u;
        return;
    }

    s = HI229_Snapshot();
    first_sample = (uint8_t)!imu_state.timestamp_valid;
    new_sample = (uint8_t)(first_sample
        || s.ts != imu_state.last_timestamp_ms);
    if (!new_sample)
    {
        return;
    }

    if (first_sample)
    {
        Attitude_Init(&imu_state);
        imu_state.timestamp_valid = 1u;
    }

    imu_state.input.gyro_dps[0] = HI229_GYR_SIGN_X * s.gyr[0];
    imu_state.input.gyro_dps[1] = HI229_GYR_SIGN_Y * s.gyr[1];
    imu_state.input.gyro_dps[2] = HI229_GYR_SIGN_Z * s.gyr[2];
    imu_state.input.accel_g[0] = HI229_ACC_SIGN_X * s.acc[0];
    imu_state.input.accel_g[1] = HI229_ACC_SIGN_Y * s.acc[1];
    imu_state.input.accel_g[2] = HI229_ACC_SIGN_Z * s.acc[2];
    imu_state.input.timestamp_ms = s.ts;
    imu_state.reference.quat[0] = s.quat[0];
    imu_state.reference.quat[1] = HI229_QUAT_SIGN_X * s.quat[1];
    imu_state.reference.quat[2] = HI229_QUAT_SIGN_Y * s.quat[2];
    imu_state.reference.quat[3] = HI229_QUAT_SIGN_Z * s.quat[3];
    imu_state.reference.euler_deg[0] = HI229_EUL_SIGN_ROLL * s.eul[0];
    imu_state.reference.euler_deg[1] = HI229_EUL_SIGN_PITCH * s.eul[1];
    imu_state.reference.euler_deg[2] = HI229_EUL_SIGN_YAW * s.eul[2];
    IMU_State_Convert_Unit(&imu_state);

    if (!first_sample && s.ts > imu_state.last_timestamp_ms)
    {
        imu_state.input.dt_s = (float)(s.ts - imu_state.last_timestamp_ms) * 0.001f;
    }
    else
    {
        imu_state.input.dt_s = 0.005f;
    }

    imu_state.online = Attitude_Update_From_HI229(&imu_state) ? 1u : 0u;
    imu_state.last_timestamp_ms = s.ts;
}

/* 控制初始化 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}

/* 控制单周期 */
void ctrl_task_body(void)
{
    float action[RL_ACTION_SIZE];  /* 推理动作 */

    if (!RL_Control_Update_Observation()
        || !RL_Policy_Run(&rl_control.policy, &rl_control.observation, action))
    {
        Action_State_Clear();
        return;
    }

    RL_Observation_Set_Last_Action(&rl_control.observation, action);
    memcpy(action_state.a, action, sizeof(action_state.a));
    action_state.updated = 1u;
    action_state.last_ok_tick = HAL_GetTick();
}

/* 切换模型 */
uint8_t RL_Control_Select_Model(rl_model_t model)
{
    if (!RL_Policy_Select(&rl_control.policy, model)) return 0u;
    RL_Observation_Reset(&rl_control.observation);
    Action_State_Clear();
    return 1u;
}

/* 遥控通道映射 */
static void Remote_Control_Update(void)
{
    int16_t vx_cmd;
    int16_t yaw_cmd;
    int16_t height_cmd;

    DR16_Process();
    if (DR16_Online())
    {
        vx_cmd = DR16_Command_Axis(dr16.ch3);
        yaw_cmd = DR16_Command_Axis(dr16.ch0);
        height_cmd = DR16_Command_Axis(dr16.wheel);

        command_state.vx       = (float)vx_cmd;
        command_state.yaw_rate = (float)yaw_cmd;
        command_state.height   = (dr16.s2 == DR16_SW_UP)
            ? (float)height_cmd : 0.0f;
        command_state.mode = dr16.s1;
        robot_state.rc_enable = (uint8_t)(dr16.s1 == DR16_SW_UP
            || dr16.s1 == DR16_SW_MID);
    }
    else
    {
        command_state.vx = 0.0f;
        command_state.yaw_rate = 0.0f;
        command_state.height = 0.0f;
        command_state.mode = 0u;
        robot_state.rc_enable = 0u;
    }
}

/* 遥控使能迁移 */
static void Remote_Enable_Update(void)
{
    uint8_t enable_ok;

    enable_ok = (uint8_t)(robot_state.rc_enable
        && ctrl_fault == FAULT_NONE && !robot_state.fallen);
    if (robot_state.enabled == enable_ok)
    {
        return;
    }

    robot_state.enabled = enable_ok;
    if (enable_ok)
    {
        (void)Dm_All_Enable();
    }
    else
    {
        (void)Dji_All_Stop();
        (void)Dm_All_Disable();
    }
}

/* 输出计算 + 保底 */
static void Motor_Output_Update(void)
{
    float torque[RL_TQ_NUM];
    float wheel_vel[2];
    uint8_t action_fresh;

    if (!robot_state.rc_enable)
    {
        (void)Dji_All_Stop();
        (void)Dm_Send_Zero();
        (void)Dm_All_Disable();
        return;
    }

    action_fresh = (uint8_t)(action_state.updated
        && (HAL_GetTick() - action_state.last_ok_tick) < 100u);
    memset(torque, 0, sizeof(torque));
    if (robot_state.enabled && action_fresh)
    {
        wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
        wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
        (void)RL_Torque_Compute(&leg_l, &leg_r,
            &rl_control.torque_param[rl_control.policy.selected_model],
            wheel_vel, action_state.a, &rl_control.torque_state, torque);
    }

    if (!robot_state.enabled || !torque_output_enabled)
    {
        (void)Dji_All_Stop();
        (void)Dm_Send_Zero();
        return;
    }
    (void)Dm_Send_Torque(torque);
    (void)Dji_Send_Wheel_Torque(torque[RL_TQ_L_WHEEL],
        torque[RL_TQ_R_WHEEL]);
}

/* 输出初始化 */
void output_task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/* 输出单周期 */
void output_task_body(void)
{
    tick_count++;
    Motor_Output_Update();
}

/* 通信单周期 */
void comm_task_body(void)
{
    static uint8_t vofa_div = 0;
    static float dbg[32];
    bool can_ok;
    float pitch_abs;
    uint32_t fault;
    uint8_t motors_ok;
    uint8_t motor_mask;
    uint8_t i;

    Dm_Parse();
    Dji_Parse();
    Motor_State_Update();
    Leg_State_Update();
    WS2812_RainbowBlink();
    Remote_Control_Update();

    can_ok = Can_Bus_Online(true);
    fault = FAULT_NONE;
    motors_ok = 1u;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!Dm_Is_Online(i))
        {
            motors_ok = 0u;
        }
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!Dji_Is_Online(i))
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
    if (!can_ok)
    {
        fault |= FAULT_CAN;
    }
    if (!motors_ok)
    {
        fault |= FAULT_MOTOR;
    }
    if (robot_state.rc_enable
        && (!action_state.updated
            || (HAL_GetTick() - action_state.last_ok_tick) >= 100u))
    {
        fault |= FAULT_ACTION;
    }
    ctrl_fault = fault;
    Remote_Enable_Update();

    pitch_abs = fabsf(imu_state.output.euler_rad[ATTITUDE_PITCH]);
    if (pitch_abs > 1.4f)
    {
        robot_state.fallen = 1u;
    }
    else if (pitch_abs < 1.0f)
    {
        robot_state.fallen = 0u;
    }

    if (++vofa_div < 5u)
    {
        return;
    }
    vofa_div = 0u;

    dbg[0] = leg_l.input.hip_f;
    dbg[1] = leg_l.input.hip_b;
    dbg[2] = leg_r.input.hip_f;
    dbg[3] = leg_r.input.hip_b;
    dbg[4] = motor_state.dm.vel_rad_s[DM_MOTOR_LEG_F_LFT];
    dbg[5] = motor_state.dm.vel_rad_s[DM_MOTOR_LEG_B_LFT];
    dbg[6] = motor_state.dm.vel_rad_s[DM_MOTOR_LEG_F_RGT];
    dbg[7] = motor_state.dm.vel_rad_s[DM_MOTOR_LEG_B_RGT];

    dbg[8] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].angle_total;
    dbg[9] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].angle_total;
    dbg[10] = motor_state.dji.angle_total_rad[DJI_MOTOR_WHEEL_LFT];
    dbg[11] = motor_state.dji.angle_total_rad[DJI_MOTOR_WHEEL_RGT];
    dbg[12] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].vel_raw;
    dbg[13] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].vel_raw;
    dbg[14] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    dbg[15] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    dbg[16] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].angle_raw;
    dbg[17] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].angle_raw;
    dbg[18] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].current_raw;
    dbg[19] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].current_raw;
    dbg[20] = leg_l.output.l0;
    dbg[21] = leg_l.output.phi0;
    dbg[22] = leg_l.output.virtual_shank;
    dbg[23] = leg_l.output.d_virtual_shank;
    dbg[24] = leg_r.output.l0;
    dbg[25] = leg_r.output.phi0;
    dbg[26] = leg_r.output.virtual_shank;
    dbg[27] = leg_r.output.d_virtual_shank;
    dbg[28] = (float)((leg_l.output.valid ? 1u : 0u)
        | (leg_r.output.valid ? 2u : 0u)
        | (leg_l.output.force_valid ? 4u : 0u)
        | (leg_r.output.force_valid ? 8u : 0u));
    dbg[29] = (float)robot_state.rc_enable;
    dbg[30] = (float)ctrl_fault;

    motor_mask = 0u;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (motor_state.dm.online[i])
        {
            motor_mask |= (uint8_t)(1u << i);
        }
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (motor_state.dji.online[i])
        {
            motor_mask |= (uint8_t)(1u << (4u + i));
        }
    }
    if (robot_state.enabled)
    {
        motor_mask |= (uint8_t)(1u << 6);
    }
    if (torque_output_enabled)
    {
        motor_mask |= (uint8_t)(1u << 7);
    }
    dbg[31] = (float)motor_mask;

    Vofa_Send(dbg, 32);
}
