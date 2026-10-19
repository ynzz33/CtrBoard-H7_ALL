/*
 * 文件职责：策略仲裁、控制求解及唯一电机输出出口（task_actuation.c）。
 * 输入/来源：一致反馈快照、拨杆模式、驱动/正常/自救许可和最新RL动作。
 * 输出/去向：选择补偿台架/LQR三态/RL/失能，统一门控后发送四髋/双轮力矩。
 * 关键边界：公共检测与三态只决定选路；原处理生成力矩，零输出/故障在最终出口复查。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "robot_control.h"
#include "robot_state.h"
#include "machine_config.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"
#include "mono_ns.h"
#include "gas_spring.h"
#include "standup.h"
#include "slip.h"
#include "air_detection.h"
#include "control_condition.h"
#include "control_detection.h"
#include "control_air_response.h"
#include <math.h>
#include <string.h>

/*
 * 输出任务三层结构 (作者 2026-09-22 定: 解算只算, 分发唯一):
 *   1 判断层  公共检测/三态 → LQR_State_Update()与原速度融合
 *   2 求解层  solve_*()                      按策略算 torque_output_t, 只写 torque 不碰驱动
 *   3 分发层  output_dispatch()              全文件唯一的 Dm_Send / Dji_Send 调用点
 * 改输出行为 (总开关 / 限幅 / 斜坡 / 极性) 只动 output_dispatch(); 改控制律只动 solve_*()
 */

static slip_state_t slip_control;
static uint8_t lqr_running;     /* 已投入 */
static uint8_t rl_engaged;      /* RL 已投入 */
static uint8_t rl_wait_ticks;   /* 500Hz采样/执行 */
static control_frame_t control_frame;
static compensation_debug_t pending_compensation;
volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];

/* 统一使能许可 */
uint8_t Robot_Control_Enable_Allowed(void)
{
    uint8_t recovery;

    if (rc_command.mode.strategy == CTRL_STRATEGY_RL && rl_control.recovery.phase == RL_RECOVERY_FAILED) { return 0u; }
    recovery = (uint8_t)(standup_control.enabled && standup_control.recovery_enabled && rc_command.mode.strategy == CTRL_STRATEGY_LQR && rc_command.mode.engage);
    recovery |= (uint8_t)(machine->rl.configured && rc_command.mode.strategy == CTRL_STRATEGY_RL && rc_command.mode.engage);
    return (uint8_t)(rc_command.mode.rc_enable && rc_command.online && ctrl_fault == FAULT_NONE && (!robot_state.fallen || recovery));
}

/* 本拍快照与许可 */
/*
 * 读取已发布的反馈快照，再在短临界区收集本拍遥控模式和输出许可。
 * landed只属于本控制拍，先清零；反馈发布时刻与传感器实际接收时刻分别保存。
 */
static void Control_Frame_Read(void)
{
    static robot_feedback_t feedback;

    (void)Robot_Feedback_Read(&feedback);
    control_frame.landed = 0u;

    taskENTER_CRITICAL();
    control_frame.imu = feedback.imu;
    control_frame.left = feedback.left;
    control_frame.right = feedback.right;
    control_frame.rc = feedback.rc;
    control_frame.height = feedback.height;
    control_frame.height_valid = feedback.height_valid;
    control_frame.fallen = robot_state.fallen;
    control_frame.wheel_vel[0] = feedback.motor.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    control_frame.wheel_vel[1] = feedback.motor.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    control_frame.joint_ready = 1u;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        control_frame.motor_pos[i] = feedback.motor.dm.pos_continuous_rad[i];
        control_frame.motor_vel[i] = feedback.motor.dm.vel_rad_s[i];
        control_frame.joint_torque[i] = feedback.motor.dm.trq_nm[i];
        control_frame.joint_ready &= (uint8_t)(feedback.motor.dm.online[i] && Dm_Is_Enabled(i));
    }
    control_frame.motor_valid = feedback.motor.valid;
    control_frame.drive = (uint8_t)(Robot_Control_Enable_Allowed() && control_frame.imu.online && control_frame.imu.pitch_world_valid && feedback.motor.valid && robot_state.motor_enabled && torque_output_enabled);
    /* RL使用实体电机反馈，不依赖虚拟几何valid；其余路径仍需要双腿解算有效。 */
    if (control_frame.rc.mode.strategy == CTRL_STRATEGY_RL)
    {
        control_frame.drive &= (uint8_t)(machine->rl.configured && rl_control.policy.ready);
    }
    else
    {
        control_frame.drive &= (uint8_t)(control_frame.left.output.valid && control_frame.right.output.valid);
    }
    control_frame.normal = (uint8_t)(control_frame.drive && !control_frame.fallen);
    control_frame.recovery = (uint8_t)(control_frame.drive && standup_control.enabled && control_frame.rc.mode.strategy == CTRL_STRATEGY_LQR && control_frame.rc.mode.engage);
    taskEXIT_CRITICAL();
}

/* 恢复仅改变RL许可/观测，不动LQR离地检测。实际投入/退出清历史，模式切换只撤销旧动作就绪。 */
static void Control_RL_Recovery_Update(void)
{
    rl_recovery_state_t recovery;
    rl_recovery_phase_t previous;
    uint8_t request;

    recovery = rl_control.recovery;
    previous = recovery.phase;
    request = (uint8_t)(control_frame.rc.online && control_frame.rc.mode.rc_enable && control_frame.rc.mode.strategy == CTRL_STRATEGY_RL && control_frame.rc.mode.engage);
    RL_Recovery_Update(&recovery, request, (uint8_t)(control_frame.drive && control_frame.height_valid), control_frame.fallen, control_frame.height);
    taskENTER_CRITICAL();
    rl_control.recovery = recovery;
    if (recovery.phase != previous)
    {
        if (previous == RL_RECOVERY_IDLE || recovery.phase == RL_RECOVERY_IDLE || recovery.phase == RL_RECOVERY_FAILED)
        {
            RL_Torque_Reset(&rl_control.torque_state);
            Action_State_Clear();
            memset(rl_control.observation.last_action, 0, sizeof(rl_control.observation.last_action));
        }
        else
        {
            /* 保留训练的原始action历史；拒绝跨模式的在途推理，等待新模式首帧。 */
            action_state.rl_ready = 0u;
            action_state.reset_count++;
        }
    }
    taskEXIT_CRITICAL();
}

/* 输出初始化 */
void output_task_init(void)
{
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/* LQR 是否已投入 */
uint8_t output_task_lqr_engaged(void)
{
    return lqr_running;
}

/* RL实际投入；推理可在零出力状态独立运行。 */
uint8_t output_task_rl_engaged(void)
{
    return rl_engaged;
}

/* ================= 3 分发层: 唯一下发点 ================= */
/* valid=0 或总输出关 → 零力矩; 否则原样下发 (各路限幅已在控制器内做, 极性在驱动边界做) */
/*
 * 唯一电机输出口：发送前再次检查最新许可、有效性和所有力矩有限性。
 * 不满足条件发送零力矩；正常才调用DM/DJI物理力矩接口，避免求解期间许可变化仍继续出力。
 * 记录的是最终门控后的命令与HAL入队结果，不证明实体电机已经执行。
 */
static void output_dispatch(const torque_output_t *torque)
{
    torque_output_t applied;
    compensation_debug_t debug;
    uint8_t zero_output;

    applied = *torque;
    debug = pending_compensation;
    debug.bench = (uint8_t)(ctrl_strategy == CTRL_STRATEGY_COMPENSATION);
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!isfinite(applied.dm[i]))
        {
            applied.valid = 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!isfinite(applied.dji[i]))
        {
            applied.valid = 0u;
        }
    }
    zero_output = (uint8_t)(!applied.valid || !control_frame.drive || ctrl_fault != FAULT_NONE || !torque_output_enabled);
    if (ctrl_strategy == CTRL_STRATEGY_RL && rl_control.recovery.phase != RL_RECOVERY_NORMAL && rl_control.recovery.phase != RL_RECOVERY_RECOVER) { zero_output = 1u; }
    if (zero_output)
    {
        memset(&applied, 0, sizeof(applied));
        memset(&debug, 0, sizeof(debug));
        debug.bench = (uint8_t)(ctrl_strategy == CTRL_STRATEGY_COMPENSATION);
    }
    taskENTER_CRITICAL();
    compensation_debug = debug;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        rl_output_dm_cmd_nm[i] = applied.dm[i];
    }
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_LFT] = applied.dji[DJI_MOTOR_WHEEL_LFT];
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_RGT] = applied.dji[DJI_MOTOR_WHEEL_RGT];
    taskEXIT_CRITICAL();

    if (zero_output)
    {
        output_debug_dm_sent = (uint8_t)(Dm_Send_Zero() == HAL_OK);
        output_debug_dji_sent = (uint8_t)(Dji_All_Stop() == HAL_OK);
        return;
    }
    output_debug_dm_sent = (uint8_t)(Dm_Send_Torque(applied.dm) == HAL_OK);
    output_debug_dji_sent = (uint8_t)(Dji_Send_Wheel_Torque( applied.dji[DJI_MOTOR_WHEEL_LFT], applied.dji[DJI_MOTOR_WHEEL_RGT]) == HAL_OK);
//     Dm_Send_Zero();
//     Dji_All_Stop();
}

/* 本拍补偿预计算 */
static void Control_Compensation_Update(void)
{
    uint8_t enabled = 0u;
    ctrl_strategy_t strategy = control_frame.rc.mode.strategy;

    if (control_frame.drive && control_frame.rc.mode.rc_enable && control_frame.rc.mode.engage)
    {
        if (strategy == CTRL_STRATEGY_LQR)
        {
            enabled = (uint8_t)((control_frame.normal ? LEG_COMP_GRAVITY : 0u) | (standup_control.enabled ? LEG_COMP_GRAVITY_WORLD : 0u));
            if (GAS_SPRING_COMP_ENABLE || (air_param.enabled && lqr_running && control_frame.normal && (!standup_control.enabled || standup_control.phase == STANDUP_DONE)))
            {
                enabled |= LEG_COMP_SPRING;
            }
        }
        else if (strategy == CTRL_STRATEGY_COMPENSATION && control_frame.normal)
        {
            enabled = LEG_COMP_SPRING | LEG_COMP_GRAVITY_WORLD;
        }
    }
    /* 公共观察器需要实体模型，不等待LQR投入或自起完成；命令补偿接入保持原样。 */
    if (air_param.enabled && control_frame.imu.online && control_frame.left.output.valid && control_frame.right.output.valid && control_frame.joint_ready)
    {
        enabled |= LEG_COMP_SPRING | LEG_COMP_GRAVITY;
    }
    Leg_Compensation_Prepare(&control_frame.compensation, &control_frame.left, &control_frame.right, &control_frame.imu, enabled);
}

/* 组装观测、速度补偿、位置积分 */
/*
 * 公共检测已在主链更新；此处构建LQR状态、接入原AIR响应和速度融合。
 * AIR停止轮速修正，仍用IMU预测；检测退出AIR的同拍恢复轮速修正，需与轮输出时序一起判读。
 */
static void Control_State_Update(uint8_t reset_velocity)
{
    if (!LQR_State_Update(&lqr_state, &control_frame.imu, &control_frame.left, &control_frame.right, control_frame.wheel_vel))
    {
        Slip_Reset(&slip_control);
        lqr_state.air_control = 0u;
        lqr_state.air_release = 0u;
        air_detection.applied = 0u;
        control_condition.valid = 0u;
        return;
    }
    Control_Air_Response_Update(&control_frame);
    if (reset_velocity || !slip_control.initialized)
    {
        Slip_Init(&slip_control, lqr_state.ds_raw, lqr_state.a_fwd);
    }
    else
    {
        (void)Slip_Update_Contact(&slip_control, lqr_state.ds_raw,
            lqr_state.a_fwd, MACHINE_LQR_DT,
            (uint8_t)(!lqr_state.air_control || !air_detection.flight));
    }
    if (!slip_control.valid)
    {
        lqr_state.valid = 0u;
        Slip_Reset(&slip_control);
        return;
    }
    LQR_Velocity_Apply(&lqr_state, slip_control.velocity, MACHINE_LQR_DT);
}

/* LQR 投入锁存 */
static uint8_t lqr_engage_update(void)
{
    uint8_t ready;
    lqr_debug_t saved_debug;

    ready = (uint8_t)(lqr_state.valid && (control_condition.condition == CONTROL_FALLEN ? control_frame.normal : (control_frame.drive && control_condition.valid)));
    if (!ready)
    {
        lqr_running = 0u;
    }
    else if (!lqr_running)
    {
        if (standup_control.recovered)
        {
            saved_debug = lqr_debug;
            LQR_Init(&lqr_state);
            lqr_debug = saved_debug;
            Control_State_Update(1u);
            standup_control.recovered = 0u;
        }
        /* 使能沿: 锁腿长目标 (不查实测腿长, 同 Leg2) */
        lqr_running = LQR_Enable_Latch(&lqr_state, &control_frame.rc);
        if (lqr_running)
        {
            Leg_Balance_Reset(&leg_balance);
        }
    }
    return lqr_running;
}

/* LQR退出只清私有历史；公共检测的历史由检测模块维护。 */
static void lqr_idle(void)
{
    lqr_running = 0u;
    Leg_Balance_Reset(&leg_balance);
    lqr_state.air_control = 0u;
    lqr_state.air_release = 0u;
    air_detection.applied = 0u;
}

/* ================= 2 求解层: 只写 torque, 不下发 ================= */
/* 仅弹簧与重力 */
static void solve_compensation(torque_output_t *torque)
{
    const float base_dm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (!control_frame.normal)
    {
        return;
    }
    torque->valid = Leg_Compensation_Apply(&control_frame.compensation.spring, &control_frame.compensation.gravity_world, base_dm, machine->dm_trq_clamp, torque->dm, &pending_compensation);
}

/* LQR 平衡: 目标 → 状态反馈 → 腿部力控 */
/*
 * 腿长直接读取公共目标，AIR屏蔽速度/转向，再运行目标、LQR和腿部力控。
 * 只求解torque并保存补偿诊断；不绕过最后输出口的许可检查。
 */
static void solve_lqr(torque_output_t *torque)
{
    rc_command_t command;

    command = control_frame.rc;
    if (lqr_state.air_control)
    {
        command.vel = 0.0f;
        command.yaw = 0.0f;
    }
    if (!LQR_Target_Update(&lqr_state, &command))
    {
        return;
    }
    if (!lqr_state.valid)
    {
        return;
    }
    LQR_Control_Update(&lqr_state);
    if (!lqr_state.gain_valid)
    {
        return;
    }
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &control_frame.left, &control_frame.right, &control_frame.compensation, &lqr_debug, MACHINE_LQR_DT, torque);
    if (torque->valid)
    {
        pending_compensation = leg_balance.compensation;
    }
}

/* 独立自起选路 */
/*
 * 自起返回BALANCE/PREPARE/STOP路由，执行层按路由决定LQR是否继续或退出。
 * PREPARE使用自起独立力矩，DONE交给LQR；无效结果清零并标记失败，不能同时叠加两套控制。
 */
static void solve_lqr_standup(torque_output_t *torque)
{
    uint8_t permit;
    uint8_t route;
    uint8_t was_done;

    permit = control_frame.recovery;
    was_done = (uint8_t)(standup_control.phase == STANDUP_DONE);
    if (was_done && permit && lqr_engage_update())
    {
        solve_lqr(torque);
    }
    route = Standup_Update(&standup_control, &control_frame.imu, &control_frame.left, &control_frame.right, &control_frame.compensation, permit, 1u, MACHINE_LQR_DT, torque);
    if (route == STANDUP_ROUTE_BALANCE)
    {
        /* 动作到位条件不变；先判断接回NORMAL还是FLIGHT，未知时等待而不重启动作。 */
        Control_Condition_Recovery_Ready(&control_condition, &air_detection);
        if (!control_condition.valid)
        {
            Torque_Output_Clear(torque);
            return;
        }
        Control_Air_Response_Update(&control_frame);
        if (!was_done)
        {
            if (lqr_engage_update())
            {
                solve_lqr(torque);
            }
            else
            {
                Standup_Fail(&standup_control, STANDUP_GATED);
            }
        }
        if (!torque->valid)
        {
            Standup_Fail(&standup_control, STANDUP_BAD_INPUT);
            Torque_Output_Clear(torque);
            lqr_idle();
            control_condition.condition = CONTROL_FALLEN;
            control_condition.normal_ready = 0u;
        }
    }
    else
    {
        lqr_idle();
        if (route == STANDUP_ROUTE_PREPARE && torque->valid)
        {
            pending_compensation = standup_control.compensation;
        }
    }
}

/* ================= 主体: 估计 → 求解 → 分发 ================= */
/*
 * 单周期主链：反馈快照 → 模型/公共检测/三态 → 原LQR状态及融合 → 唯一求解 → 诊断 → 唯一输出口。
 * torque初始为无效零值；只有求解成功的分支能置valid。大机RL间隔拍可保留已发输出，退出/故障仍复查。
 * RTOS节拍由freertos入口等待，本函数不创建任务或自行等待发送完成。
 */
void output_task_body(void)
{
    float wheel_vel[2];
    ctrl_strategy_t strategy;
    torque_output_t torque;
    uint8_t dispatch;
#if CONTROL_TIME_VOFA_ENABLE
    static uint32_t previous_start_us;
    static uint32_t previous_output_us;
    uint32_t start_us;
    uint32_t period_us;
    uint32_t run_us;

    start_us = (uint32_t)(Mono_Ns_Get() / 1000u);
    period_us = control_time_debug.sequence != 0u ? start_us - previous_start_us : 0u;
    previous_start_us = start_us;
#endif

    Control_Frame_Read();
    Control_RL_Recovery_Update();
    wheel_vel[0] = control_frame.wheel_vel[0];
    wheel_vel[1] = control_frame.wheel_vel[1];
    Control_Compensation_Update();

    /* 1 一次公共检测 → 三态选路 → 原LQR状态/速度融合 */
    if (Control_Detection_Update(&control_frame))
    {
        Standup_Reset(&standup_control);
    }
    Control_State_Update(0u);

    /* 2 求解: torque 默认全零 valid=0, 只有走通的分支才置 valid */
    Torque_Output_Clear(&torque);
    memset(&pending_compensation, 0, sizeof(pending_compensation));
    strategy = control_frame.rc.mode.rc_enable ? control_frame.rc.mode.strategy : CTRL_STRATEGY_DISABLE;
    ctrl_strategy = strategy;
    if (!standup_control.enabled || strategy == CTRL_STRATEGY_COMPENSATION || (control_frame.rc.online && !(control_frame.rc.mode.strategy == CTRL_STRATEGY_LQR && control_frame.rc.mode.engage)))
    {
        Standup_Reset(&standup_control);
    }
    else if (strategy != CTRL_STRATEGY_LQR && standup_control.phase != STANDUP_IDLE)
    {
        Standup_Fail(&standup_control, STANDUP_GATED);
    }
    rl_engaged = 0u;
    dispatch = 1u;
    if (strategy != CTRL_STRATEGY_RL)
    {
        rl_wait_ticks = 0u;
    }

    switch (strategy)
    {
        case CTRL_STRATEGY_COMPENSATION:
            lqr_idle();
            solve_compensation(&torque);
            break;

        case CTRL_STRATEGY_LQR:
        {
            if (!control_frame.drive || !control_frame.rc.mode.engage)
            {
                if (control_condition.condition == CONTROL_FALLEN && standup_control.enabled && standup_control.phase != STANDUP_IDLE && control_frame.rc.mode.engage)
                {
                    solve_lqr_standup(&torque);
                }
                else
                {
                    lqr_idle();
                }
            }
            else if (!control_condition.valid)
            {
                /* 初判/重确认期间保持默认无效零候选，不能清公共观察器计时。 */
            }
            else if (control_condition.condition == CONTROL_FALLEN)
            {
                if (standup_control.enabled)
                {
                    solve_lqr_standup(&torque);
                }
            }
            else if (control_condition.condition == CONTROL_FLIGHT && !air_param.control_enabled)
            {
                /* 关闭AIR处理不能隐式改选NORMAL；保持默认无效零候选。 */
            }
            else if (lqr_engage_update())
            {
                solve_lqr(&torque);
            }
            else
            {
                lqr_idle();
            }
            control_condition.normal_ready = (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_NORMAL && torque.valid);
            break;
        }

        case CTRL_STRATEGY_RL:
        {
            /* 基础任务仍1kHz；选中RL后每2拍采样/PD，策略每20拍更新一次原始action。 */
            uint8_t tick = (uint8_t)(rl_wait_ticks == 0u);
            lqr_running = 0u;
            rl_wait_ticks = tick ? MACHINE_RL_CTRL_DIV - 1u : rl_wait_ticks - 1u;
            if (tick)
            {
                (void)RL_Motor_Map(&machine->rl, control_frame.motor_pos, control_frame.motor_vel, wheel_vel, control_frame.motor_valid, &control_frame.rl_motor);
                RL_Observation_Sample(&rl_control.observation, control_frame.imu.gyro_rad_s, control_frame.imu.quat, &control_frame.rl_motor, control_frame.imu.online);
            }
            rl_engaged = (uint8_t)(control_frame.drive && control_frame.rc.mode.engage && rl_control.recovery.phase != RL_RECOVERY_IDLE && rl_control.recovery.phase != RL_RECOVERY_FAILED);
            if (rl_engaged && rl_control.recovery.phase != RL_RECOVERY_HOLD && action_state.rl_ready && control_frame.rl_motor.valid)
            {
                /* 正常间隔拍保持上次已发命令；故障/退出不走保持分支，仍每基础拍清零。 */
                if (tick) { (void)RL_Torque_Compute(&control_frame.rl_motor, &machine->rl, action_state.a, &rl_control.torque_state, &torque); }
                else { dispatch = 0u; }
            }
            break;
        }

        case CTRL_STRATEGY_DISABLE:
        default:
            lqr_idle();
            break;
    }

    /* 3 分发: 唯一出口 */
    taskENTER_CRITICAL();
    air_debug = air_detection;
    taskEXIT_CRITICAL();
    if (dispatch)
    {
        output_dispatch(&torque);
#if CONTROL_TIME_VOFA_ENABLE
        control_time_debug.output_period_us = control_time_debug.output_sequence != 0u ? start_us - previous_output_us : 0u;
        previous_output_us = start_us;
        control_time_debug.output_sequence++;
#endif
    }
#if CONTROL_TIME_VOFA_ENABLE
    run_us = (uint32_t)(Mono_Ns_Get() / 1000u) - start_us;
    if (control_time_debug.sequence != 0u)
    {
        if (control_time_debug.min_period_us == 0u || period_us < control_time_debug.min_period_us)
        {
            control_time_debug.min_period_us = period_us;
        }
        if (period_us > control_time_debug.max_period_us)
        {
            control_time_debug.max_period_us = period_us;
        }
    }
    control_time_debug.period_us = period_us;
    control_time_debug.run_us = run_us;
    control_time_debug.sequence++;
#endif
}
