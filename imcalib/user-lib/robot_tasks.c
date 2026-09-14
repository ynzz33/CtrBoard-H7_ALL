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

#define LEG_GEOMETRY_PI 3.14159265358979f
#define FORCE_TEST_FORCE_MAX     8.0f
#define FORCE_TEST_TORQUE_MAX    1.0f
#define FORCE_TEST_MOTOR_LIMIT   1.0f
#define REMOTE_COMMAND_SCALE     0.1f

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

typedef struct {
    float l0;
    float phi0;
    float virtual_shank;
    float measured[3];
    float predicted[3];
    float residual[3];
    uint32_t tick_ms;
    uint8_t ready;
} leg_debug_history_t;

static leg_debug_history_t leg_debug_l;
static leg_debug_history_t leg_debug_r;

typedef enum {
    FORCE_TEST_NONE = 0u,
    FORCE_TEST_LEFT,
    FORCE_TEST_RIGHT,
} force_test_leg_t;

typedef struct {
    float force;
    float torque;
    float motor_torque[RL_TQ_NUM];
    force_test_leg_t leg;
    uint8_t mode;
    uint8_t active;
} force_map_test_t;

static force_map_test_t force_map_test;
static volatile uint8_t remote_debug_online;
static volatile uint8_t remote_debug_s1;
static volatile uint8_t remote_debug_s2;
static volatile int16_t remote_debug_ch3;
static volatile int16_t remote_debug_ch0;
static volatile uint8_t output_debug_dm_sent;
static volatile uint8_t output_debug_dji_sent;

static void Leg_Debug_Validate(const leg_state_t *leg,
                               leg_debug_history_t *history);

/* 限制测试力矩 */
static float Force_Test_Clip(float value)
{
    if (value > FORCE_TEST_MOTOR_LIMIT)
    {
        return FORCE_TEST_MOTOR_LIMIT;
    }
    if (value < -FORCE_TEST_MOTOR_LIMIT)
    {
        return -FORCE_TEST_MOTOR_LIMIT;
    }
    return value;
}

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
            + LEG_GEOMETRY_PI + leg_l.config.offset_f;
        leg_l.input.hip_b = motor_state.dm.pos_rad[leg_map_l.dm_rear]
            + leg_l.config.offset_b;
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_rad[leg_map_r.dm_front]
            + LEG_GEOMETRY_PI + leg_r.config.offset_f;
        leg_r.input.hip_b = motor_state.dm.pos_rad[leg_map_r.dm_rear]
            + leg_r.config.offset_b;
        leg_r.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_r.dm_front];
        leg_r.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_r.dm_rear];
    }
    (void)Leg_Solve(&leg_l);
    (void)Leg_Solve(&leg_r);
    Leg_Debug_Validate(&leg_l, &leg_debug_l);
    Leg_Debug_Validate(&leg_r, &leg_debug_r);
}

/* 速度层自检 */
static float Leg_Debug_Angle_Diff(float current, float previous)
{
    float diff;

    diff = current - previous;
    while (diff > 3.14159265358979f)
    {
        diff -= 6.28318530717959f;
    }
    while (diff < -3.14159265358979f)
    {
        diff += 6.28318530717959f;
    }
    return diff;
}

static void Leg_Debug_Validate(const leg_state_t *leg,
                               leg_debug_history_t *history)
{
    float dt_s;
    uint32_t now_ms;

    if (leg == NULL || history == NULL || !leg->output.valid)
    {
        if (history != NULL)
        {
            history->ready = 0u;
        }
        return;
    }
    now_ms = HAL_GetTick();
    if (!history->ready)
    {
        history->l0 = leg->output.l0;
        history->phi0 = leg->output.phi0;
        history->virtual_shank = leg->output.virtual_shank;
        history->tick_ms = now_ms;
        history->ready = 1u;
        return;
    }
    if (now_ms == history->tick_ms)
    {
        return;
    }

    dt_s = (float)(now_ms - history->tick_ms) * 0.001f;
    history->measured[0] = (leg->output.l0 - history->l0) / dt_s;
    history->measured[1] = Leg_Debug_Angle_Diff(leg->output.phi0,
        history->phi0) / dt_s;
    history->measured[2] = Leg_Debug_Angle_Diff(leg->output.virtual_shank,
        history->virtual_shank) / dt_s;
    history->predicted[0] = leg->output.dl0;
    history->predicted[1] = leg->output.dphi0;
    history->predicted[2] = leg->output.d_virtual_shank;
    history->residual[0] = history->predicted[0] - history->measured[0];
    history->residual[1] = history->predicted[1] - history->measured[1];
    history->residual[2] = history->predicted[2] - history->measured[2];
    history->l0 = leg->output.l0;
    history->phi0 = leg->output.phi0;
    history->virtual_shank = leg->output.virtual_shank;
    history->tick_ms = now_ms;
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
    torque_output_enabled = 1u;
    Leg_Init(&leg_l);
    Leg_Init(&leg_r);
    leg_l.config.lu = 0.13087f;
    leg_l.config.lg = 0.15240f;
    /* 软件零位 = 电机反馈 - 标定读数 */
    leg_l.config.offset_f = -0.03f;
    leg_l.config.offset_b = -0.04f;
    leg_l.config.offset_phi0 = -0.13f;
    leg_l.config.mirror = 1;
    leg_l.config.configured = 1u;
    leg_r.config.lu = 0.13087f;
    leg_r.config.lg = 0.15240f;
    leg_r.config.offset_f = -0.038f;
    leg_r.config.offset_b = -0.023f;
    leg_r.config.offset_phi0 = -0.07f;
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
    dr16_t remote;
    int16_t vx_cmd;
    int16_t yaw_cmd;
    int16_t height_cmd;
    uint8_t remote_online;
    uint8_t enable_request;
    uint8_t height_request;
    uint32_t enable_faults;

    DR16_Process();
    remote_online = (uint8_t)DR16_Online();
    remote = DR16_Snapshot();
    remote_debug_online = remote_online;
    remote_debug_s1 = remote.s1;
    remote_debug_s2 = remote.s2;
    remote_debug_ch3 = remote.ch3;
    remote_debug_ch0 = remote.ch0;
    enable_request = 0u;
    height_request = 0u;
    if (remote_online)
    {
        if (remote.s1 == DR16_SW_UP || remote.s1 == DR16_SW_MID)
        {
            enable_request = 1u;
        }
        if (remote.s2 == DR16_SW_UP)
        {
            height_request = 1u;
        }
    }

    force_map_test.active = 0u;
    force_map_test.mode = 0u;
    force_map_test.leg = FORCE_TEST_NONE;
    force_map_test.force = 0.0f;
    force_map_test.torque = 0.0f;
    if (remote_online && remote.s1 == DR16_SW_UP)
    {
        force_map_test.mode = 1u;
        if (remote.s2 == DR16_SW_UP)
        {
            force_map_test.leg = FORCE_TEST_LEFT;
        }
        else if (remote.s2 == DR16_SW_MID)
        {
            force_map_test.leg = FORCE_TEST_RIGHT;
        }
        if (force_map_test.leg != FORCE_TEST_NONE)
        {
            force_map_test.active = 1u;
            force_map_test.force = (float)DR16_Command_Axis(remote.ch3)
                / (float)DR16_CH_LIMIT * FORCE_TEST_FORCE_MAX;
            force_map_test.torque = (float)DR16_Command_Axis(remote.ch0)
                / (float)DR16_CH_LIMIT * FORCE_TEST_TORQUE_MAX;
        }
    }

    if (remote_online)
    {
        vx_cmd = DR16_Command_Axis(remote.ch3);
        yaw_cmd = DR16_Command_Axis(remote.ch0);
        height_cmd = DR16_Command_Axis(remote.wheel);

        command_state.vx       = (float)vx_cmd / (float)DR16_CH_LIMIT
            * REMOTE_COMMAND_SCALE;
        command_state.yaw_rate = (float)yaw_cmd / (float)DR16_CH_LIMIT
            * REMOTE_COMMAND_SCALE;
        command_state.height   = height_request
            ? (float)height_cmd / (float)DR16_CH_LIMIT
                * REMOTE_COMMAND_SCALE : 0.0f;
        command_state.mode = remote.s1;
    }
    else
    {
        command_state.vx = 0.0f;
        command_state.yaw_rate = 0.0f;
        command_state.height = 0.0f;
        command_state.mode = 0u;
    }

    robot_state.rc_enable = enable_request;

    enable_faults = ctrl_fault;
    if (!torque_output_enabled || force_map_test.mode)
    {
        enable_faults &= ~FAULT_ACTION;
    }
    if (robot_state.rc_enable && enable_faults == FAULT_NONE
        && !robot_state.fallen)
    {
        if (!robot_state.enabled)
        {
            robot_state.enabled = 1u;
            (void)Dm_All_Enable();
        }
    }
    else
    {
        if (robot_state.enabled)
        {
            robot_state.enabled = 0u;
            (void)Dji_All_Stop();
            (void)Dm_All_Disable();
        }
    }
}

/* 输出计算 + 保底 */
static void Motor_Output_Update(void)
{
    float torque[RL_TQ_NUM];
    float wheel_vel[2];
    float leg_torque[2];
    HAL_StatusTypeDef dm_status;
    HAL_StatusTypeDef dji_status;
    uint8_t action_fresh;
    uint32_t i;

    if (!robot_state.rc_enable)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dji_All_Stop();
        (void)Dm_Send_Zero();
        return;
    }

    action_fresh = (uint8_t)(action_state.updated
        && (HAL_GetTick() - action_state.last_ok_tick) < 100u);
    memset(torque, 0, sizeof(torque));
    memset(force_map_test.motor_torque, 0, sizeof(force_map_test.motor_torque));
    if (robot_state.enabled && action_fresh && !force_map_test.mode)
    {
        wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
        wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
        (void)RL_Torque_Compute(&leg_l, &leg_r,
            &rl_control.torque_param[rl_control.policy.selected_model],
            wheel_vel, action_state.a, &rl_control.torque_state, torque);
    }
    if (robot_state.enabled && force_map_test.active)
    {
        leg_torque[0] = 0.0f;
        leg_torque[1] = 0.0f;
        if (force_map_test.leg == FORCE_TEST_LEFT
            && Leg_Force_Map_Forward(&leg_l, force_map_test.force,
                force_map_test.torque, leg_torque))
        {
            torque[RL_TQ_L_THIGH] = Force_Test_Clip(leg_torque[0]);
            torque[RL_TQ_L_SHANK] = Force_Test_Clip(leg_torque[1]);
        }
        else if (force_map_test.leg == FORCE_TEST_RIGHT
            && Leg_Force_Map_Forward(&leg_r, force_map_test.force,
                force_map_test.torque, leg_torque))
        {
            torque[RL_TQ_R_THIGH] = Force_Test_Clip(leg_torque[0]);
            torque[RL_TQ_R_SHANK] = Force_Test_Clip(leg_torque[1]);
        }
    }
    for (i = 0u; i < RL_TQ_NUM; i++)
    {
        force_map_test.motor_torque[i] = torque[i];
    }

    if (!robot_state.enabled || !torque_output_enabled)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dji_All_Stop();
        (void)Dm_Send_Zero();
        return;
    }
    dm_status = Dm_Send_Torque(torque);
    dji_status = Dji_Send_Wheel_Torque(torque[RL_TQ_L_WHEEL],
        torque[RL_TQ_R_WHEEL]);
    output_debug_dm_sent = (uint8_t)(dm_status == HAL_OK);
    output_debug_dji_sent = (uint8_t)(dji_status == HAL_OK);
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
    uint8_t gravity_valid;
    uint8_t i;
    float gravity[3];
    float gravity_norm;
    float quat_norm;

    Dm_Parse();
    Dji_Parse();
    Motor_State_Update();
    Leg_State_Update();
    gravity_valid = RL_Observation_Project_Gravity(imu_state.output.quat,
        gravity);
    if (!gravity_valid)
    {
        gravity[0] = 0.0f;
        gravity[1] = 0.0f;
        gravity[2] = 0.0f;
    }
    gravity_norm = sqrtf(gravity[0] * gravity[0] + gravity[1] * gravity[1]
        + gravity[2] * gravity[2]);
    quat_norm = sqrtf(imu_state.output.quat[0] * imu_state.output.quat[0]
        + imu_state.output.quat[1] * imu_state.output.quat[1]
        + imu_state.output.quat[2] * imu_state.output.quat[2]
        + imu_state.output.quat[3] * imu_state.output.quat[3]);
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

    dbg[0] = imu_state.input.gyro_rad_s[0];
    dbg[1] = imu_state.input.gyro_rad_s[1];
    dbg[2] = imu_state.input.gyro_rad_s[2];
    dbg[3] = imu_state.input.accel_g[0];
    dbg[4] = imu_state.input.accel_g[1];
    dbg[5] = imu_state.input.accel_g[2];
    dbg[6] = imu_state.output.quat[0];
    dbg[7] = imu_state.output.quat[1];
    dbg[8] = imu_state.output.quat[2];
    dbg[9] = imu_state.output.quat[3];
    dbg[10] = gravity[0];
    dbg[11] = gravity[1];
    dbg[12] = gravity[2];
    dbg[13] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    dbg[14] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    dbg[15] = motor_state.dji.angle_total_rad[DJI_MOTOR_WHEEL_LFT];
    dbg[16] = motor_state.dji.angle_total_rad[DJI_MOTOR_WHEEL_RGT];
    dbg[17] = command_state.vx;
    dbg[18] = command_state.yaw_rate;
    dbg[19] = command_state.height;
    dbg[20] = (float)remote_debug_s1;
    dbg[21] = (float)remote_debug_s2;
    dbg[22] = (float)motor_state.dji.current_raw[DJI_MOTOR_WHEEL_LFT];
    dbg[23] = (float)motor_state.dji.current_raw[DJI_MOTOR_WHEEL_RGT];
    dbg[24] = (float)imu_state.online;
    dbg[25] = (float)gravity_valid;
    dbg[26] = imu_state.output.euler_rad[ATTITUDE_ROLL];
    dbg[27] = imu_state.output.euler_rad[ATTITUDE_PITCH];
    dbg[28] = imu_state.output.euler_rad[ATTITUDE_YAW];
    dbg[29] = gravity_norm;
    dbg[30] = quat_norm;
    dbg[31] = (float)((leg_l.output.force_valid ? 1u : 0u)
        | (leg_r.output.valid ? 2u : 0u)
        | (leg_l.output.force_valid ? 4u : 0u)
        | (leg_r.output.force_valid ? 8u : 0u));
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
