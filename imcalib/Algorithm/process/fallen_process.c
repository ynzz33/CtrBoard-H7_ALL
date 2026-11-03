/*
 * 根据倒地姿态规划普通前/后侧自起或倒置翻身，推进阶段并生成腿长/摆角目标。
 * 独立长度/角度控制器输出准备力矩；返回PREPARE/BALANCE/STOP供执行层决定输出。
 */

#include "fallen_process.h"
#include "fallen_detection.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>


fallen_process_ctx_t fallen_process_control;


/* 将有限角度按2pi折回[-pi,pi]；边界与本模块连续角累加配套。 */
static float Fallen_Process_Wrap(float angle)
{
    angle = fmodf(angle, LEG_2PI);
    if (angle > LEG_PI)
    {
        angle -= LEG_2PI;
    }
    if (angle < -LEG_PI)
    {
        angle += LEG_2PI;
    }
    return angle;
}

/* 切换动作阶段并清阶段时间、到位连续时间和触发时间，不清整段轨迹。 */
static void Fallen_Process_Enter(fallen_process_ctx_t *st, uint8_t phase)
{
    st->lifecycle.phase = phase;
    st->lifecycle.elapsed = 0.0f;
    st->lifecycle.stable_elapsed = 0.0f;
    st->lifecycle.trigger_elapsed = 0.0f;
}

/* 初始化自起PD */
static void Fallen_Process_PID_Init(fallen_process_ctx_t *st)
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        PID_struct_init(&st->actuator.length_pid[i], POSITION_PID, machine->lqr.pd.leg_len[i].max_output, 0.0f, machine->lqr.pd.leg_len[i].kp, 0.0f, machine->lqr.pd.leg_len[i].kd, 0.0f, 0.0f);
        PID_struct_init(&st->actuator.angle_pos_pid[i], POSITION_PID, fallen_process_param.angle_speed_max, 0.0f, fallen_process_param.angle_pos_kp, 0.0f, fallen_process_param.angle_pos_kd, 0.0f, 0.0f);
        st->actuator.angle_pos_pid[i].angle_wrap = 1u;
        PID_struct_init(&st->actuator.angle_speed_pid[i], POSITION_PID, machine->hip.dm_trq_clamp, 0.0f, fallen_process_param.angle_speed_kp, 0.0f, fallen_process_param.angle_speed_kd, 0.0f, 0.0f);
    }
}

/* 首次清状态、默认开启自起/翻身，并初始化独立长度及摆角串级PID。 */
void Fallen_Process_Init(fallen_process_ctx_t *st)
{
    memset(st, 0, sizeof(*st));
    st->lifecycle.enabled = 1u;
    st->lifecycle.recovery_enabled = 1u;
    Fallen_Process_PID_Init(st);
}

/* 清本轮动作、轨迹和控制器历史，保留普通自起/倒置翻身两个功能开关。 */
void Fallen_Process_Reset(fallen_process_ctx_t *st)
{
    uint8_t enabled;
    uint8_t recovery_enabled;

    enabled = st->lifecycle.enabled;
    recovery_enabled = st->lifecycle.recovery_enabled;
    memset(st, 0, sizeof(*st));
    st->lifecycle.enabled = enabled;
    st->lifecycle.recovery_enabled = recovery_enabled;
}

/* 首次失败时锁存FAILED和原因并开始本模块等待计时，避免每拍重置等待起点。 */
void Fallen_Process_Fail(fallen_process_ctx_t *st, uint8_t fault)
{
    if (st->lifecycle.phase != FALLEN_PROCESS_FAILED)
    {
        st->lifecycle.phase = FALLEN_PROCESS_FAILED;
        st->lifecycle.fault = fault;
        st->lifecycle.retry_elapsed = 0.0f;
    }
}

/* 公共反馈标志 */
static uint8_t Fallen_Process_Input_Valid(const imu_state_t *imu,
                                  const leg_state_t *const leg[2])
{
    if (!imu->online || !imu->pitch_world_valid)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!leg[i]->output.valid || !leg[i]->output.force_valid)
        {
            return 0u;
        }
    }
    return 1u;
}

/* 接收已完成准备位的动作；目标/历史重建由本模块负责，任务层不伪造FALLEN。 */
uint8_t Fallen_Process_Begin_Retract(fallen_process_ctx_t *st, const imu_state_t *imu, const leg_state_t *left, const leg_state_t *right)
{
    const leg_state_t *leg[2] = {left, right};
    if (!Fallen_Process_Input_Valid(imu, leg)) { return 0u; }
    Fallen_Process_Init(st);
    st->lifecycle.recovery_enabled = 0u;
    st->normal.planned = 1u;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        st->normal.path[i] = 2u;
        st->normal.prepare_angle[i] = fallen_process_param.rear_angle;
        st->actuator.angle_target[i] = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        st->actuator.length_target[i] = leg[i]->output.virtual_leg_length;
    }
    Fallen_Process_Enter(st, FALLEN_PROCESS_RETRACT);
    return 1u;
}

/* 伸腿目标 */
static float Fallen_Process_Extend_Length(void)
{
    return clampf(fallen_process_param.extend_len, machine->leg.leg_len_min, machine->leg.leg_len_max);
}


/* 翻身扫腿前先确认两腿已形成要求的力臂。 */
static uint8_t Fallen_Process_Preloaded(const leg_state_t *const leg[2])
{
    float minimum;

    minimum = clampf(fallen_process_param.recovery_len, machine->leg.leg_len_min, machine->leg.leg_len_max) - fallen_process_param.length_tol;
    return (uint8_t)(leg[0]->output.virtual_leg_length >= minimum && leg[1]->output.virtual_leg_length >= minimum);
}

/* 将每拍包角变化累加成连续角，跨±pi时仍保持准备位路径方向。 */
static void Fallen_Process_Rear_Path_Update(fallen_process_ctx_t *st,
                                     const leg_state_t *const leg[2])
{
    float angle;

    for (uint8_t i = 0u; i < 2u; i++)
    {
        angle = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        st->normal.rear_position[i] += Fallen_Process_Wrap(angle - st->normal.rear_last[i]);
        st->normal.rear_last[i] = angle;
    }
}

/* 按当前阶段检查双腿实际位置及指令是否到位，输出ready_block位用于定位未满足项。 */
static uint8_t Fallen_Process_Ready(fallen_process_ctx_t *st,
                            const leg_state_t *const leg[2])
{
    float length_goal;
    float angle_error;
    float command_error;
    uint16_t ready_block = 0u;

    if (st->lifecycle.phase == FALLEN_PROCESS_RETRACT)
    {
        length_goal = fallen_process_param.retract_len + fallen_process_param.length_tol;
        for (uint8_t i = 0u; i < 2u; i++)
        {
            if (leg[i]->output.virtual_leg_length > length_goal)
            {
                ready_block |= (uint16_t)(1u << i);
            }
        }
        return (uint8_t)(ready_block == 0u);
    }
    if (st->lifecycle.phase == FALLEN_PROCESS_REAR)
    {
        if (!st->normal.rear_path_ready)
        {
            return 0u;
        }
        for (uint8_t i = 0u; i < 2u; i++)
        {
            if (fabsf(st->normal.rear_angle_target[i] - st->normal.rear_position[i]) > fallen_process_param.angle_tol)
            {
                ready_block |= (uint16_t)(1u << (4u + i));
            }
            if (fabsf(st->normal.rear_angle_target[i] - st->actuator.angle_target[i]) > fallen_process_param.angle_tol)
            {
                ready_block |= (uint16_t)(1u << (6u + i));
            }
        }
        return (uint8_t)(ready_block == 0u);
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (st->lifecycle.phase == FALLEN_PROCESS_SWING)
        {
            angle_error = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle - machine->lqr.reference.leg_trim[i]);
            command_error = Fallen_Process_Wrap(st->actuator.angle_target[i] - machine->lqr.reference.leg_trim[i]);
            if (fabsf(angle_error) > fallen_process_param.angle_tol)
            {
                ready_block |= (uint16_t)(1u << (4u + i));
            }
            if (fabsf(command_error) > fallen_process_param.angle_tol)
            {
                ready_block |= (uint16_t)(1u << (6u + i));
            }
        }
    }
    return (uint8_t)(ready_block == 0u);
}

/* 按当前测量和目标预置PID误差历史，避免开始新段时沿用上段差分。 */
static void Fallen_Process_Prime(pid_t *pid, float measured, float target)
{
    float error;

    error = target - measured;
    if (pid->angle_wrap)
    {
        error = Fallen_Process_Wrap(error);
    }
    PID_History_Set(pid, measured, target, error);
}

/* 检查当前是否在倒置稳定/伸腿/前点对齐/翻身阶段，用来保持翻身路径归属。 */
static uint8_t Fallen_Process_Recovering(const fallen_process_ctx_t *st)
{
    return (uint8_t)((st->lifecycle.phase >= FALLEN_PROCESS_SETTLE && st->lifecycle.phase <= FALLEN_PROCESS_FLIP) || st->lifecycle.phase == FALLEN_PROCESS_ALIGN_FRONT);
}

/* 进入指定倒置恢复阶段，按当前腿位初始化该阶段连续角与控制历史。 */
static void Fallen_Process_Recovery_Enter(fallen_process_ctx_t *st, uint8_t phase,
                                    const leg_state_t *const leg[2])
{
    float angle;

    Fallen_Process_Enter(st, phase);
    st->actuator.len_history_ready = 0u;
    st->actuator.angle_history_ready = 0u;
    st->actuator.support = 0.0f;
    st->self_right.stall_elapsed = 0.0f;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        angle = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        st->self_right.position[i] = angle;
        st->self_right.last[i] = angle;
        st->actuator.angle_target[i] = angle;
        st->actuator.length_target[i] = leg[i]->output.virtual_leg_length;
        st->actuator.angle_pos_pid[i].angle_wrap = 0u;
        if (phase == FALLEN_PROCESS_FLIP)
        {
            /* 双腿已确认到前点；统一指令起点，不保留各自的实测角差。 */
            st->actuator.angle_target[i] = fallen_process_param.recovery_front_angle;
        }
    }
    if (phase == FALLEN_PROCESS_FLIP)
    {
        st->self_right.sweep = 0.0f;
    }
}

/* 检查世界俯仰是否落入给定回正窗口rad；持续确认时间由调用方累计。 */
static uint8_t Fallen_Process_Recovery_Ready(const imu_state_t *imu, float pitch_limit)
{
    return (uint8_t)(fallen_detection.upright > 0.0f && fabsf(imu->pitch_world) <= pitch_limit);
}

/* 记录动作超时并返回停止路由，任务层随后执行撤力等待和重新初判。 */
static uint8_t Fallen_Process_Timeout(fallen_process_ctx_t *st)
{
    /* 本轮耗尽只报告超时；任务层撤力等待并重新判断，不直接重做预压。 */
    Fallen_Process_Fail(st, FALLEN_PROCESS_TIMEOUT);
    return FALLEN_PROCESS_ROUTE_STOP;
}

static void Fallen_Process_Normal_Plan(fallen_process_ctx_t *st, const imu_state_t *imu,
                                       const leg_state_t *const leg[2]);

/* 推进倒置SETTLE→PRELOAD→ALIGN_FRONT→FLIP，检查腿长、对齐、回正和卡滞。
 * 翻身回正后调用Normal_Plan重新规划普通自起，不直接跳到平衡输出。 */
static uint8_t Fallen_Process_Recovery_Stage(fallen_process_ctx_t *st, const imu_state_t *imu,
                                     const leg_state_t *const leg[2], float dt)
{
    float angle;
    float error;
    float speed;
    float start;
    float goal;
    uint8_t ready;

    for (uint8_t i = 0u; i < 2u; i++)
    {
        angle = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        st->self_right.position[i] += Fallen_Process_Wrap(angle - st->self_right.last[i]);
        st->self_right.last[i] = angle;
    }
    st->lifecycle.elapsed += dt;
    switch (st->lifecycle.phase)
    {
    case FALLEN_PROCESS_SETTLE:
        if (st->lifecycle.elapsed >= fallen_process_param.recovery_settle_time)
        {
            Fallen_Process_Recovery_Enter(st, FALLEN_PROCESS_PRELOAD, leg);
        }
        break;
    case FALLEN_PROCESS_PRELOAD:
        if (st->lifecycle.elapsed > fallen_process_param.length_timeout)
        {
            return Fallen_Process_Timeout(st);
        }
        if (Fallen_Process_Preloaded(leg))
        {
            Fallen_Process_Recovery_Enter(st, FALLEN_PROCESS_ALIGN_FRONT, leg);
            /* 复用原世界角0~2pi上绕通道，目标只在进入对齐时锁定。 */
            for (uint8_t i = 0u; i < 2u; i++)
            {
                start = Fallen_Process_Wrap(st->self_right.position[i] - imu->pitch_world);
                goal = Fallen_Process_Wrap(fallen_process_param.recovery_front_angle - imu->pitch_world);
                if (start < 0.0f) { start += LEG_2PI; }
                if (goal < 0.0f) { goal += LEG_2PI; }
                st->self_right.front_goal[i] = st->self_right.position[i] + goal - start;
            }
        }
        break;
    case FALLEN_PROCESS_ALIGN_FRONT:
        if (st->lifecycle.elapsed > fallen_process_param.angle_timeout)
        {
            return Fallen_Process_Timeout(st);
        }
        ready = Fallen_Process_Preloaded(leg);
        for (uint8_t i = 0u; i < 2u; i++)
        {
            if (fabsf(st->self_right.front_goal[i] - st->self_right.position[i]) > fallen_process_param.angle_tol || fabsf(st->self_right.front_goal[i] - st->actuator.angle_target[i]) > fallen_process_param.angle_tol)
            {
                ready = 0u;
            }
        }
        st->lifecycle.stable_elapsed = ready ? st->lifecycle.stable_elapsed + dt : 0.0f;
        if (st->lifecycle.stable_elapsed >= fallen_process_param.stable_time)
        {
            Fallen_Process_Recovery_Enter(st, FALLEN_PROCESS_FLIP, leg);
        }
        break;
    case FALLEN_PROCESS_FLIP:
        if (Fallen_Process_Recovery_Ready(imu, fallen_process_param.recovery_pitch_tol))
        {
            st->lifecycle.stable_elapsed += dt;
            if (st->lifecycle.stable_elapsed >= fallen_process_param.stable_time)
            {
                st->lifecycle.recovered = 1u;
                st->actuator.len_history_ready = 0u;
                st->actuator.angle_history_ready = 0u;
                st->actuator.support = 0.0f;
                for (uint8_t i = 0u; i < 2u; i++)
                {
                    st->actuator.angle_pos_pid[i].angle_wrap = 1u;
                    st->actuator.length_target[i] = leg[i]->output.virtual_leg_length;
                }
                /* 翻身回正后按当前腿位重新选路，复用新版普通自起。 */
                Fallen_Process_Normal_Plan(st, imu, leg);
                break;
            }
        }
        else
        {
            st->lifecycle.stable_elapsed = 0.0f;
        }
        error = fmaxf(fabsf(st->actuator.angle_target[0] - st->self_right.position[0]), fabsf(st->actuator.angle_target[1] - st->self_right.position[1]));
        speed = fmaxf(fabsf(leg[0]->output.d_virtual_leg_angle), fabsf(leg[1]->output.d_virtual_leg_angle));
        if (error > 0.5f && speed < 0.1f)
        {
            if (st->self_right.stall_elapsed == 0.0f)
            {
                st->self_right.stall_upright = fallen_detection.upright;
            }
            st->self_right.stall_elapsed += dt;
            if (fallen_detection.upright > st->self_right.stall_upright + 0.05f)
            {
                st->self_right.stall_elapsed = 0.0f;
            }
        }
        else
        {
            st->self_right.stall_elapsed = 0.0f;
        }
        if (st->self_right.stall_elapsed >= fallen_process_param.recovery_stall_time || st->self_right.sweep >= LEG_2PI)
        {
            return Fallen_Process_Timeout(st);
        }
        break;
    default:
        Fallen_Process_Fail(st, FALLEN_PROCESS_BAD_CONFIG);
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    return FALLEN_PROCESS_ROUTE_PREPARE;
}

/* 按倒置阶段生成腿长与连续摆角指令，按recovery_angle_rate*dt推进前点对齐或翻身扫角。 */
static void Fallen_Process_Recovery_Target(fallen_process_ctx_t *st,
                                     const leg_state_t *const leg[2], float dt)
{
    float target;
    float step;

    target = fallen_process_param.recovery_len;
    target = clampf(target, machine->leg.leg_len_min, machine->leg.leg_len_max);
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (st->lifecycle.phase == FALLEN_PROCESS_SETTLE)
        {
            st->actuator.length_target[i] = leg[i]->output.virtual_leg_length;
            st->actuator.angle_target[i] = st->self_right.position[i];
        }
        else
        {
            st->actuator.length_target[i] = target;
        }
        if (st->lifecycle.phase == FALLEN_PROCESS_ALIGN_FRONT)
        {
            step = fallen_process_param.recovery_angle_rate * dt;
            st->actuator.angle_target[i] = clampf(st->self_right.front_goal[i], st->actuator.angle_target[i] - step, st->actuator.angle_target[i] + step);
        }
        else if (st->lifecycle.phase == FALLEN_PROCESS_FLIP && st->lifecycle.stable_elapsed == 0.0f)
        {
            st->actuator.angle_target[i] += fallen_process_param.recovery_angle_rate * dt;
        }

    }
    if (st->lifecycle.phase == FALLEN_PROCESS_FLIP && st->lifecycle.stable_elapsed == 0.0f)
    {
        st->self_right.sweep += fallen_process_param.recovery_angle_rate * dt;
    }

}

/* 检查普通摆正的实测腿角及指令角是否均处于angle_tol窗口，不额外要求机身平衡。 */
static uint8_t Fallen_Process_Normal_Pose_Ready(const fallen_process_ctx_t *st,
                                                const leg_state_t *const leg[2])
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (fabsf(Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle - st->normal.prepare_angle[i])) > fallen_process_param.angle_tol)
        {
            return 0u;
        }
    }
    return 1u;
}

/* 检查普通规划路径当前长度目标是否到位，为伸腿/收腿阶段切换提供条件。 */
static uint8_t Fallen_Process_Normal_Length_Ready(const fallen_process_ctx_t *st,
                                                  const leg_state_t *const leg[2])
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (fabsf(leg[i]->output.virtual_leg_length - st->actuator.length_target[i]) > fallen_process_param.length_tol)
        {
            return 0u;
        }
    }
    return 1u;
}

/* 按当前腿相对机身/世界的姿态锁定每腿前侧、后侧或身下路径，并建立连续角准备目标。 */
static void Fallen_Process_Normal_Plan(fallen_process_ctx_t *st, const imu_state_t *imu,
                                       const leg_state_t *const leg[2])
{
    float angle;
    float start;
    float front;
    float rear;
    float goal;
    uint8_t aligned;

    st->normal.planned = 1u;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        angle = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        st->actuator.angle_target[i] = angle;
        st->actuator.length_target[i] = leg[i]->output.virtual_leg_length;
        // if (fabsf(Fallen_Process_Wrap(angle - imu->pitch_world - machine->lqr.reference.leg_trim[i])) <= fallen_process_param.normal_swing_window)
        // {
        //     st->normal.path[i] = 0u;
        //     st->normal.prepare_angle[i] = angle;
        //     continue;
        // }
        /* 沿用原上绕分支的0~2pi表示比较路径，不用跨机身零位的最短角。 */
        start = Fallen_Process_Wrap(angle - imu->pitch_world);
        front = Fallen_Process_Wrap(fallen_process_param.front_angle - imu->pitch_world);
        rear = Fallen_Process_Wrap(fallen_process_param.rear_angle - imu->pitch_world);
        if (start < 0.0f) { start += LEG_2PI; }
        if (front < 0.0f) { front += LEG_2PI; }
        if (rear < 0.0f) { rear += LEG_2PI; }
        st->normal.path[i] = fabsf(front - start) < fabsf(rear - start) ? 1u : 2u;
        goal = st->normal.path[i] == 1u ? fallen_process_param.front_angle : fallen_process_param.rear_angle;
        st->normal.prepare_angle[i] = goal;
        /* 已在准备位或长腿足够时保持，不为了准备动作先缩短再伸长。 */
        if (fabsf(Fallen_Process_Wrap(angle - goal)) > fallen_process_param.angle_tol &&
            leg[i]->output.virtual_leg_length < Fallen_Process_Extend_Length() - fallen_process_param.length_tol)
        {
            st->actuator.length_target[i] = Fallen_Process_Extend_Length();
        }
    }
    aligned = Fallen_Process_Normal_Pose_Ready(st, leg);
    if (aligned)
    {
        Fallen_Process_Enter(st, (leg[0]->output.virtual_leg_length <= fallen_process_param.retract_len + fallen_process_param.length_tol &&
                                 leg[1]->output.virtual_leg_length <= fallen_process_param.retract_len + fallen_process_param.length_tol) ? FALLEN_PROCESS_SWING : FALLEN_PROCESS_RETRACT);
    }
    else
    {
        Fallen_Process_Enter(st, Fallen_Process_Normal_Length_Ready(st, leg) ? FALLEN_PROCESS_REAR : FALLEN_PROCESS_EXTEND);
    }
}


/* 按已锁定普通路径推进准备位、收腿和摆正目标；不在每拍重新选择前后路径。 */
static void Fallen_Process_Normal_Target(fallen_process_ctx_t *st, const imu_state_t *imu,
                                          const leg_state_t *const leg[2], float angle_rate, float dt)
{
    float angle;
    float start;
    float goal;
    float step;

    if (st->lifecycle.phase == FALLEN_PROCESS_REAR && !st->normal.rear_path_ready)
    {
        for (uint8_t i = 0u; i < 2u; i++)
        {
            angle = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
            start = Fallen_Process_Wrap(angle - imu->pitch_world);
            goal = Fallen_Process_Wrap(st->normal.prepare_angle[i] - imu->pitch_world);
            if (start < 0.0f) { start += LEG_2PI; }
            if (goal < 0.0f) { goal += LEG_2PI; }
            st->normal.rear_position[i] = angle;
            st->normal.rear_last[i] = angle;
            st->normal.rear_angle_target[i] = angle + goal - start;
            st->actuator.angle_target[i] = angle;
            st->actuator.angle_pos_pid[i].angle_wrap = 0u;
        }
        st->normal.rear_path_ready = 1u;
    }
    if (!st->actuator.len_history_ready && st->lifecycle.phase != FALLEN_PROCESS_REAR)
    {
        for (uint8_t i = 0u; i < 2u; i++)
        {
            st->actuator.angle_target[i] = Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        }
    }
    if (st->lifecycle.phase == FALLEN_PROCESS_SWING)
    {
        st->actuator.support = clampf(st->actuator.support + dt / fallen_process_param.support_time, 0.0f, 1.0f);
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (st->lifecycle.phase == FALLEN_PROCESS_REAR)
        {
            step = angle_rate * dt;
            st->actuator.angle_target[i] = clampf(st->normal.rear_angle_target[i], st->actuator.angle_target[i] - step, st->actuator.angle_target[i] + step);
        }
        else if (st->lifecycle.phase == FALLEN_PROCESS_RETRACT || st->lifecycle.phase == FALLEN_PROCESS_SWING)
        {
            st->actuator.length_target[i] = fallen_process_param.retract_len;
            if (st->lifecycle.phase == FALLEN_PROCESS_SWING)
            {
                /* 首拍从实测角起步，沿原包角误差方向限速；沿用angle_target保存轨迹。 */
                ramp_t angle_ramp = {
                    .out = st->actuator.angle_history_ready ? st->actuator.angle_target[i] : Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle),
                    .rate = angle_rate,
                };
                goal = angle_ramp.out + Fallen_Process_Wrap(machine->lqr.reference.leg_trim[i] - angle_ramp.out);
                st->actuator.angle_target[i] = Fallen_Process_Wrap(Ramp_Update(&angle_ramp, goal, dt));
            }
        }
    }
}

/* 根据许可、反馈、倒置状态和各阶段到位/超时条件推进动作，返回准备/平衡/停止路由。 */
static uint8_t Fallen_Process_Stage_Update(fallen_process_ctx_t *st,
                                   const imu_state_t *imu,
                                   const leg_state_t *const leg[2],
                                   uint8_t permit,
                                   float dt)
{
    float timeout;
    uint8_t retry;

    if (st->lifecycle.phase == FALLEN_PROCESS_FAILED)
    {
        if (!permit)
        {
            st->lifecycle.retry_elapsed = 0.0f;
            return FALLEN_PROCESS_ROUTE_STOP;
        }
        if (st->lifecycle.fault == FALLEN_PROCESS_TIMEOUT || st->lifecycle.fault == FALLEN_PROCESS_BAD_CONFIG || st->lifecycle.retry >= fallen_process_param.recovery_retry_max)
        {
            return FALLEN_PROCESS_ROUTE_STOP;
        }
        st->lifecycle.retry_elapsed += dt;
        if (st->lifecycle.retry_elapsed < fallen_process_param.retry_wait)
        {
            return FALLEN_PROCESS_ROUTE_STOP;
        }
        retry = st->lifecycle.retry + 1u;
        Fallen_Process_Reset(st);
        st->lifecycle.retry = retry;
    }
    if (!permit)
    {
        if (st->lifecycle.phase != FALLEN_PROCESS_IDLE)
        {
            Fallen_Process_Fail(st, FALLEN_PROCESS_GATED);
        }
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    if (st->lifecycle.recovery_enabled)
    {
        if (fallen_detection.inverted && !Fallen_Process_Recovering(st))
        {
            retry = st->lifecycle.retry;
            Fallen_Process_Reset(st);
            st->lifecycle.retry = retry;
            Fallen_Process_PID_Init(st);
            Fallen_Process_Recovery_Enter(st, FALLEN_PROCESS_SETTLE, leg);
        }
    }
    if (Fallen_Process_Recovering(st))
    {
        return Fallen_Process_Recovery_Stage(st, imu, leg, dt);
    }
    if (st->lifecycle.phase == FALLEN_PROCESS_DONE)
    {
        if (fallen_detection.abnormal)
        {
            st->lifecycle.trigger_elapsed += dt;
        }
        else
        {
            st->lifecycle.trigger_elapsed = 0.0f;
            st->lifecycle.retry = 0u;
        }
        if (st->lifecycle.trigger_elapsed < fallen_detection_param.trigger_time)
        {
            return FALLEN_PROCESS_ROUTE_BALANCE;
        }
        retry = st->lifecycle.retry;
        Fallen_Process_Reset(st);
        st->lifecycle.retry = retry;
    }
    if (st->lifecycle.phase == FALLEN_PROCESS_IDLE)
    {
        /* 自动翻身关闭时，倒置只撤力等待，不回退旧固定自起。 */
        if (fallen_detection.inverted)
        {
            return FALLEN_PROCESS_ROUTE_STOP;
        }
        retry = st->lifecycle.retry;
        Fallen_Process_Reset(st);
        st->lifecycle.retry = retry;
        Fallen_Process_PID_Init(st);
        Fallen_Process_Normal_Plan(st, imu, leg);
    }
    if (st->lifecycle.phase != FALLEN_PROCESS_RETRACT && st->lifecycle.phase != FALLEN_PROCESS_SWING && st->lifecycle.phase != FALLEN_PROCESS_REAR && st->lifecycle.phase != FALLEN_PROCESS_EXTEND)
    {
        Fallen_Process_Fail(st, FALLEN_PROCESS_BAD_CONFIG);
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    if (st->lifecycle.phase == FALLEN_PROCESS_REAR && st->normal.rear_path_ready)
    {
        Fallen_Process_Rear_Path_Update(st, leg);
    }
    if (fabsf(imu->pitch_world) > fallen_detection_param.pitch_max)
    {
        Fallen_Process_Fail(st, FALLEN_PROCESS_BAD_POSE);
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    st->lifecycle.elapsed += dt;
    timeout = (st->lifecycle.phase == FALLEN_PROCESS_EXTEND || st->lifecycle.phase == FALLEN_PROCESS_RETRACT) ? fallen_process_param.length_timeout : fallen_process_param.angle_timeout;
    if (st->lifecycle.elapsed > timeout)
    {
        Fallen_Process_Fail(st, FALLEN_PROCESS_TIMEOUT);
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    if (st->lifecycle.phase == FALLEN_PROCESS_EXTEND ?
        Fallen_Process_Normal_Length_Ready(st, leg) : Fallen_Process_Ready(st, leg))
    {
        if (st->lifecycle.phase == FALLEN_PROCESS_RETRACT)
        {
            Fallen_Process_Enter(st, FALLEN_PROCESS_SWING);
            return FALLEN_PROCESS_ROUTE_PREPARE;
        }
        st->lifecycle.stable_elapsed += dt;
    }
    else
    {
        st->lifecycle.stable_elapsed = 0.0f;
    }
    if (st->lifecycle.stable_elapsed >= fallen_process_param.stable_time)
    {
        if (st->lifecycle.phase == FALLEN_PROCESS_REAR || st->lifecycle.phase == FALLEN_PROCESS_EXTEND)
        {
            st->actuator.len_history_ready = 0u;
            st->actuator.angle_history_ready = 0u;
            for (uint8_t i = 0u; i < 2u; i++)
            {
                st->actuator.angle_pos_pid[i].angle_wrap = 1u;
            }
            if (st->lifecycle.phase == FALLEN_PROCESS_EXTEND)
            {
                Fallen_Process_Enter(st, FALLEN_PROCESS_REAR);
            }
            else if (leg[0]->output.virtual_leg_length <= fallen_process_param.retract_len + fallen_process_param.length_tol && leg[1]->output.virtual_leg_length <= fallen_process_param.retract_len + fallen_process_param.length_tol)
            {
                Fallen_Process_Enter(st, FALLEN_PROCESS_SWING);
            }
            else
            {
                Fallen_Process_Enter(st, FALLEN_PROCESS_RETRACT);
            }
        }
        else
        {
            Fallen_Process_Enter(st, FALLEN_PROCESS_DONE);
            return FALLEN_PROCESS_ROUTE_BALANCE;
        }
    }
    return FALLEN_PROCESS_ROUTE_PREPARE;
}

/* 根据当前普通或倒置阶段生成本拍腿长/摆角目标，随后独立控制器跟踪这些目标。 */
static void Fallen_Process_Target_Update(fallen_process_ctx_t *st,
                                  const imu_state_t *imu,
                                  const leg_state_t *const leg[2],
                                  float angle_rate, float dt)
{
    if (Fallen_Process_Recovering(st))
    {
        Fallen_Process_Recovery_Target(st, leg, dt);
    }
    else
    {
        Fallen_Process_Normal_Target(st, imu, leg, angle_rate, dt);
    }
}

/* 由长度和串级摆角控制器求轴向力/摆矩，映射四髋后加补偿并限矩；准备阶段双轮为零。 */
static uint8_t Fallen_Process_Compute_Output(fallen_process_ctx_t *st,
                                      const leg_state_t *const leg[2],
                                      const leg_compensation_values_t *values,
                                      float dt,
                                      torque_output_t *torque)
{
    float angle;
    float rate;
    float angle_speed_target;
    float support_weight;
    float length_target;
    float force[2];
    float tp[2];
    float base[DM_MOTOR_NUM];
    torque_output_t prepared;

    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (Fallen_Process_Recovering(st))
        {
            angle = st->self_right.position[i];
        }
        else
        {
            angle = st->lifecycle.phase == FALLEN_PROCESS_REAR ? st->normal.rear_position[i] : Fallen_Process_Wrap(leg[i]->output.virtual_leg_angle);
        }
        rate = leg[i]->output.d_virtual_leg_angle;
        if (!isfinite(angle))
        {
            return 0u;
        }
        length_target = Ramp_Target_Update(&st->actuator.length_ramp[i], leg[i]->output.virtual_leg_length, st->actuator.length_target[i], fallen_process_param.length_rate, dt);
        if (!isfinite(length_target))
        {
            return 0u;
        }
        if (!st->actuator.len_history_ready)
        {
            Fallen_Process_Prime(&st->actuator.length_pid[i], leg[i]->output.virtual_leg_length, length_target);
        }
        force[i] = pid_calc(&st->actuator.length_pid[i], leg[i]->output.virtual_leg_length, length_target, dt);
        tp[i] = 0.0f;
        if (st->lifecycle.phase != FALLEN_PROCESS_RETRACT)
        {
            if (!st->actuator.angle_history_ready)
            {
                Fallen_Process_Prime(&st->actuator.angle_pos_pid[i], angle, st->actuator.angle_target[i]);
            }
            angle_speed_target = pid_calc(&st->actuator.angle_pos_pid[i], angle, st->actuator.angle_target[i], dt);
            if (!st->actuator.angle_history_ready)
            {
                Fallen_Process_Prime(&st->actuator.angle_speed_pid[i], rate, angle_speed_target);
            }
            tp[i] = pid_calc(&st->actuator.angle_speed_pid[i], rate, angle_speed_target, dt);
        }
        support_weight = clampf(1.0f - fabsf(Fallen_Process_Wrap(angle - machine->lqr.reference.leg_trim[i])) / machine->recovery.fallen_support_angle_range, 0.0f, 1.0f);
        force[i] += st->actuator.support * support_weight * machine->lqr.roll_height.support_force[i];
        if (!isfinite(force[i]) || !isfinite(tp[i]))
        {
            return 0u;
        }
    }
    st->actuator.len_history_ready = 1u;
    if (st->lifecycle.phase != FALLEN_PROCESS_RETRACT)
    {
        st->actuator.angle_history_ready = 1u;
    }
    Torque_Output_Clear(&prepared);
    memset(&st->actuator.compensation, 0, sizeof(st->actuator.compensation));
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!Leg_Force_Map_Forward(leg[i], force[i], tp[i], &base[2u * i]))
        {
            return 0u;
        }
    }
    if (!Leg_Compensation_Apply(&values->spring, &values->gravity, base, machine->hip.dm_trq_clamp, prepared.dm, &st->actuator.compensation))
    {
        return 0u;
    }
    prepared.valid = 1u;
    *torque = prepared;
    return 1u;
}

/* 一拍自起入口：检查输入/许可→推进阶段→更新目标→计算准备力矩，返回输出路由。
 * DONE路径可消费执行层预先生成的平衡候选；返回STOP时不能延用旧力矩。 */
uint8_t Fallen_Process_Update(fallen_process_ctx_t *st, const imu_state_t *imu,
                       const leg_state_t *left, const leg_state_t *right,
                       const leg_compensation_values_t *values,
                       uint8_t permit, float angle_rate, float dt,
                       torque_output_t *torque)
{
    const leg_state_t *leg[2] = {left, right};
    uint8_t previous_phase;
    uint8_t route;

    if (!st->lifecycle.enabled)
    {
        return FALLEN_PROCESS_ROUTE_BALANCE;
    }
    if (!Fallen_Process_Input_Valid(imu, leg))
    {
        Fallen_Process_Fail(st, FALLEN_PROCESS_BAD_INPUT);
        st->lifecycle.retry_elapsed = 0.0f;
        Torque_Output_Clear(torque);
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    previous_phase = st->lifecycle.phase;
    route = Fallen_Process_Stage_Update(st, imu, leg, permit, dt);
    if (route == FALLEN_PROCESS_ROUTE_STOP)
    {
        Torque_Output_Clear(torque);
        return route;
    }
    if (route == FALLEN_PROCESS_ROUTE_BALANCE)
    {
        if (previous_phase == FALLEN_PROCESS_DONE && !torque->valid)
        {
            Fallen_Process_Fail(st, FALLEN_PROCESS_BAD_INPUT);
            Torque_Output_Clear(torque);
            return FALLEN_PROCESS_ROUTE_STOP;
        }
        return route;
    }

    Fallen_Process_Target_Update(st, imu, leg, angle_rate, dt);
    if (!Fallen_Process_Compute_Output(st, leg, values, dt, torque))
    {
        Fallen_Process_Fail(st, FALLEN_PROCESS_BAD_INPUT);
        Torque_Output_Clear(torque);
        return FALLEN_PROCESS_ROUTE_STOP;
    }
    return route;
}




