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
#include "pid.h"

#include <math.h>
#include <string.h>

#define LEG_GEOMETRY_PI 3.14159265358979f
#if 0 /* 旧单关节极性测试参数，暂时停用并保留 */
#define FORCE_TEST_FORCE_MAX     8.0f
#define FORCE_TEST_TORQUE_MAX    1.0f
#define FORCE_TEST_MOTOR_LIMIT   1.0f
#endif
#define REMOTE_COMMAND_SCALE     0.1f
#define ACTION_TEST_POS_LIMIT     3.0f
#define ACTION_TEST_LEG_LIMIT     10.0f
#define ACTION_TEST_POS_SCALE     0.5f

/* 共享状态 */
imu_state_t       imu_state;
motor_state_t     motor_state;
leg_state_t       leg_l;
leg_state_t       leg_r;
action_state_t    action_state;
input_command_t   input_command;
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
    ACTION_TEST_NONE = 0u,
    ACTION_TEST_LEFT,
    ACTION_TEST_RIGHT,
} action_test_side_t;

typedef struct {
    float action[RL_ACTION_SIZE];
    float base_action[RL_ACTION_SIZE];
    float thigh_base[4];
    float thigh_target[4];
    pid_t thigh_pid[4];
    action_test_side_t side;
    uint8_t mode;
    uint8_t active;
    uint8_t latched;
    uint8_t thigh_pid_ready;
} action_test_t;

static action_test_t action_test;

#if 0 /* 旧 force-map 测试状态，暂时停用并保留 */
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
#endif
static volatile uint8_t remote_debug_online;
static volatile uint8_t remote_debug_s1;
static volatile uint8_t remote_debug_s2;
static volatile uint8_t output_debug_dm_sent;
static volatile uint8_t output_debug_dji_sent;

static void Leg_Debug_Validate(const leg_state_t *leg,
                               leg_debug_history_t *history);

/* 保持锁存目标不变，仅把单圈反馈换到距离目标最近的等效分支。 */
static float Action_Test_Equivalent_Angle_Feedback(float feedback, float target)
{
    float error = target - feedback;

    while (error > 3.141592653589793f) error -= 6.283185307179586f;
    while (error <= -3.141592653589793f) error += 6.283185307179586f;
    return target - error;
}

static void Action_Test_Thigh_Compute(float torque[RL_TQ_NUM])
{
    const rl_torque_param_t *param;
    float tau_lf;
    float tau_lb;
    float tau_rf;
    float tau_rb;

    param = &rl_control.torque_param[rl_control.policy.selected_model];
    if (!action_test.thigh_pid_ready)
    {
        memset(action_test.thigh_pid, 0, sizeof(action_test.thigh_pid));
        PID_struct_init(&action_test.thigh_pid[0], POSITION_PID, 1000.0f,
            0.0f, param->p_gains[0], 0.0f, param->d_gains[0], 0.0f, 0.0f);
        PID_struct_init(&action_test.thigh_pid[1], POSITION_PID, 1000.0f,
            0.0f, param->p_gains[0], 0.0f, param->d_gains[0], 0.0f, 0.0f);
        PID_struct_init(&action_test.thigh_pid[2], POSITION_PID, 1000.0f,
            0.0f, param->p_gains[3], 0.0f, param->d_gains[3], 0.0f, 0.0f);
        PID_struct_init(&action_test.thigh_pid[3], POSITION_PID, 1000.0f,
            0.0f, param->p_gains[3], 0.0f, param->d_gains[3], 0.0f, 0.0f);
        action_test.thigh_pid_ready = 1u;
    }

    tau_lf = pid_calc(&action_test.thigh_pid[0],
        Action_Test_Equivalent_Angle_Feedback(leg_l.input.hip_f,
            action_test.thigh_target[0]), action_test.thigh_target[0], 0.002f);
    tau_lb = pid_calc(&action_test.thigh_pid[1],
        Action_Test_Equivalent_Angle_Feedback(leg_l.input.hip_b,
            action_test.thigh_target[1]), action_test.thigh_target[1], 0.002f);
    tau_rf = pid_calc(&action_test.thigh_pid[2],
        Action_Test_Equivalent_Angle_Feedback(leg_r.input.hip_f,
            action_test.thigh_target[2]), action_test.thigh_target[2], 0.002f);
    tau_rb = pid_calc(&action_test.thigh_pid[3],
        Action_Test_Equivalent_Angle_Feedback(leg_r.input.hip_b,
            action_test.thigh_target[3]), action_test.thigh_target[3], 0.002f);

    torque[RL_TQ_L_THIGH] = -tau_lf;
    torque[RL_TQ_L_SHANK] = -tau_lb;
    torque[RL_TQ_R_THIGH] = -tau_rf;
    torque[RL_TQ_R_SHANK] = -tau_rb;
}

#if 0 /* 旧测试力矩限幅，暂时停用并保留 */
static float Force_Test_Clip(float value)
{
    if (value > FORCE_TEST_MOTOR_LIMIT) return FORCE_TEST_MOTOR_LIMIT;
    if (value < -FORCE_TEST_MOTOR_LIMIT) return -FORCE_TEST_MOTOR_LIMIT;
    return value;
}
#endif

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

    command[0] = input_command.vx_cmd;
    command[1] = input_command.yaw_cmd;
    command[2] = input_command.height_cmd;

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

    DR16_Process();
    remote = DR16_Snapshot();

    if (!remote.online || remote.s1 == DR16_SW_DOWN)
    {
        memset(action_test.action, 0, sizeof(action_test.action));
        action_test.mode = 0u;
        action_test.active = 0u;
        action_test.latched = 0u;
        action_test.thigh_pid_ready = 0u;
        action_test.side = ACTION_TEST_NONE;
        robot_state.rc_enable = DISABLE;
    }
    else
    {
        robot_state.rc_enable = ENABLE;
    }


    if (remote.online)
    {
        input_command.vx_cmd     = (float)DR16_Command_Axis(remote.ch3)   / (float)DR16_CH_LIMIT* REMOTE_COMMAND_SCALE;
        input_command.yaw_cmd    = (float)DR16_Command_Axis(remote.ch0)   / (float)DR16_CH_LIMIT* REMOTE_COMMAND_SCALE;
        input_command.height_cmd = (float)DR16_Command_Axis(remote.wheel) / (float)DR16_CH_LIMIT* REMOTE_COMMAND_SCALE;
        input_command.mode = remote.s1;
    }
    else
    {
        input_command.vx_cmd     = 0.0f;
        input_command.yaw_cmd    = 0.0f;
        input_command.height_cmd = 0.0f;
        input_command.mode = 0u;
    }

    if (robot_state.rc_enable && !robot_state.fallen)
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
    HAL_StatusTypeDef dm_status;
    HAL_StatusTypeDef dji_status;
    uint8_t action_fresh;
    uint8_t action_ready;
    const float *active_action;

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
    active_action = action_state.a;
    action_ready = action_fresh;
    if (action_test.mode)
    {
        active_action = action_test.action;
        action_ready = action_test.active;
    }
    memset(torque, 0, sizeof(torque));
#if 0 /* 旧 force-map 直接输出路径，暂时停用并保留 */
    memset(force_map_test.motor_torque, 0, sizeof(force_map_test.motor_torque));
#endif
    if (robot_state.enabled && action_ready)
    {
        if (action_test.mode)
        {
            /* 临时测试：大腿 action 分解为前后髋各自的位置目标；不使用虚拟小腿。 */
            Action_Test_Thigh_Compute(torque);
            rl_control.torque_state.virtual_torque[1] = 0.0f;
            rl_control.torque_state.virtual_torque[4] = 0.0f;
            torque[RL_TQ_L_WHEEL] = 0.0f;
            torque[RL_TQ_R_WHEEL] = 0.0f;
            rl_control.torque_state.virtual_torque[2] = 0.0f;
            rl_control.torque_state.virtual_torque[5] = 0.0f;
            RL_Torque_Clamp_Output(torque, ACTION_TEST_LEG_LIMIT, 0.0f);
            memcpy(rl_control.torque_state.last_torque, torque,
                sizeof(rl_control.torque_state.last_torque));
        }
        else
        {
            wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
            wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
            (void)RL_Torque_Compute(&leg_l, &leg_r,
                &rl_control.torque_param[rl_control.policy.selected_model],
                wheel_vel, active_action, &rl_control.torque_state, torque);
        }
    }
#if 0 /* 旧 force-map 直接输出路径，暂时停用并保留 */
    if (robot_state.enabled && force_map_test.active)
    {
        float leg_torque[2] = {0.0f, 0.0f};
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
#endif
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

    if (action_test.mode)
    {
        /* 显示 PID 实际使用的等效多圈反馈，避免原始单圈角跨界跳变。 */
        dbg[0] = action_test.thigh_pid[0].get[NOW];
        dbg[1] = action_test.thigh_pid[1].get[NOW];
        dbg[2] = action_test.thigh_pid[2].get[NOW];
        dbg[3] = action_test.thigh_pid[3].get[NOW];
        dbg[4] = action_test.thigh_pid[0].set[NOW];
        dbg[5] = action_test.thigh_pid[1].set[NOW];
        dbg[6] = action_test.thigh_pid[2].set[NOW];
        dbg[7] = action_test.thigh_pid[3].set[NOW];
        dbg[8] = action_test.thigh_pid[0].err[NOW];
        dbg[9] = action_test.thigh_pid[1].err[NOW];
        dbg[10] = action_test.thigh_pid[2].err[NOW];
        dbg[11] = action_test.thigh_pid[3].err[NOW];
        dbg[12] = action_test.thigh_pid[0].pout;
        dbg[13] = action_test.thigh_pid[1].pout;
        dbg[14] = action_test.thigh_pid[2].pout;
        dbg[15] = action_test.thigh_pid[3].pout;
    }
    else
    {
        dbg[0] = leg_l.input.hip_f;
        dbg[1] = leg_l.output.virtual_shank;
        dbg[2] = leg_r.input.hip_f;
        dbg[3] = leg_r.output.virtual_shank;
        dbg[4] = rl_control.torque_state.controller[0].set[NOW];
        dbg[5] = rl_control.torque_state.controller[1].set[NOW];
        dbg[6] = rl_control.torque_state.controller[3].set[NOW];
        dbg[7] = rl_control.torque_state.controller[4].set[NOW];
        dbg[8] = rl_control.torque_state.controller[0].err[NOW];
        dbg[9] = rl_control.torque_state.controller[1].err[NOW];
        dbg[10] = rl_control.torque_state.controller[3].err[NOW];
        dbg[11] = rl_control.torque_state.controller[4].err[NOW];
        dbg[12] = rl_control.torque_state.controller[0].pout;
        dbg[13] = rl_control.torque_state.controller[1].pout;
        dbg[14] = rl_control.torque_state.controller[3].pout;
        dbg[15] = rl_control.torque_state.controller[4].pout;
    }
    dbg[16] = leg_l.output.l0;
    dbg[17] = leg_r.output.l0;
    dbg[18] = rl_control.torque_state.controller[3].dout;
    dbg[19] = rl_control.torque_state.controller[4].dout;
    dbg[20] = rl_control.torque_state.virtual_torque[0];
    dbg[21] = rl_control.torque_state.virtual_torque[1];
    dbg[22] = rl_control.torque_state.virtual_torque[3];
    dbg[23] = rl_control.torque_state.virtual_torque[4];
    dbg[24] = (float)action_test.mode;
    dbg[25] = (float)action_test.side;
    dbg[26] = rl_control.torque_state.last_torque[RL_TQ_L_THIGH];
    dbg[27] = rl_control.torque_state.last_torque[RL_TQ_L_SHANK];
    dbg[28] = rl_control.torque_state.last_torque[RL_TQ_R_THIGH];
    dbg[29] = rl_control.torque_state.last_torque[RL_TQ_R_SHANK];
    dbg[30] = (float)output_debug_dm_sent;
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
