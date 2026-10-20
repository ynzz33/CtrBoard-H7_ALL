/*
 * 文件职责：离地/落地状态估计（air_detection.c）。
 * 输入/来源：执行任务传入的双腿几何、四髋反馈力矩与IMU。
 * 输出/去向：发布支持力、initialized/valid/flight及压缩诊断，执行层决定是否切换控制。
 * 关键边界：按固定执行周期处理缓存反馈；无效观测保留flight。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "air_detection.h"
#include "machine_config.h"
#include <math.h>
#include <string.h>

air_param_t air_param = {
    .enabled = 1u, .control_enabled = 1u,
    .landing_enabled = 1u, .compression_enabled = 1u,
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    .wheel_mass = 0.65f, .force_off = 5.0f, .force_on = 20.0f,
    .hip_max = 10.0f,
    .time_off = 0.1f, .time_on = 0.05f,
    .compression_on = 0.03f, .compression_time = 0.1f,
#else
    .wheel_mass = 0.13463f, .force_off = 6.0f, .force_on = 18.0f,
    .hip_max = 8.0f,
    .time_off = 0.01f, .time_on = 0.02f,
    .compression_on = 0.01f, .compression_time = 0.05f,
#endif
    .derivative_tau = 0.01f, .force_tau = 0.01f,
};
air_detection_t air_detection;
volatile air_debug_t air_debug;

/* 重新等待样本 */
/*
 * 清空本次检测所有历史，包括initialized、flight和悬空峰值。
 * 初始化/关闭检测调用；LQR退出不清公共观测，清零不能当作真实落地事件。
 */
void Air_Detection_Reset(air_detection_t *state)
{
    memset(state, 0, sizeof(*state));
}

/* 清条件计时 */
static void Air_Timer_Reset(air_detection_t *state)
{
    state->elapsed = 0.0f;
    state->initial_unload_elapsed = 0.0f;
    state->landing[0].elapsed = 0.0f;
    state->landing[1].elapsed = 0.0f;
}

/* 失效不算接地 */
/*
 * 观测失效时清连续条件计时并保留flight和悬空峰值。
 * fault记录本拍失败；恢复时重新播种差分，失败拍不累计连续条件时间。
 */
static void Air_Invalid(air_detection_t *state)
{
    state->valid = 0u;
    state->initialized = 0u;
    Air_Timer_Reset(state);
    state->fault = 1u;
}

/* 单腿支持力 */
/*
 * 由同侧前后髋反馈反解电机轴向力/摆矩，读取本拍实体弹簧力与原姿态引用的自重摆矩。
 * 将力投影到世界竖直方向，再用机体/腿投影加速度估计轮重动态项；滤波得到本腿FN。
 * 首帧没有可靠差分，必须先播种；这里计算的是支持力估计，不是长度PID给定或发送力矩。
 */
static uint8_t Air_Estimate(air_leg_t *next, const air_leg_t *previous,const air_input_t *input, uint8_t side, uint8_t seeded, float az)
{
    const leg_state_t *leg;
    float theta;
    float omega;
    float length;
    const float dt = MACHINE_LQR_DT;
    float alpha;
    float force;
    float torque;

    leg = input->leg[side];
    if (!Leg_Force_Map_Inverse(leg, input->torque[side], &next->motor_force, &next->motor_torque))
    {
        return 0u;
    }
    length = leg->output.virtual_leg_length;
    theta = remainderf(leg->output.virtual_leg_angle - leg->config.offset_phi0 - input->imu->euler_rad[ATTITUDE_ROLL], LEG_2PI);
    omega = leg->output.d_virtual_leg_angle - input->imu->gyro_rad_s[1];
    if (length <= 1.0e-3f || !isfinite(theta) || !isfinite(omega))
    {
        return 0u;
    }
    next->spring_force = input->compensation->spring.force[side];
    next->gravity_torque = 0.0f;
    if (machine->gravity != NULL)
    {
        next->gravity_torque = input->compensation->gravity.leg[side].torque;
    }
    force = next->motor_force + next->spring_force;
    torque = next->motor_torque - next->gravity_torque;
    next->pressure = force * cosf(theta) - torque * sinf(theta) / length;
    next->height_velocity = leg->output.d_virtual_leg_length * cosf(theta) - length * omega * sinf(theta);
    next->height_acceleration = 0.0f;
    if (seeded)
    {
        alpha = dt / (air_param.derivative_tau + dt);
        next->height_acceleration = previous->height_acceleration + alpha * ((next->height_velocity - previous->height_velocity) / dt - previous->height_acceleration);
    }
    next->wheel_acceleration = az - next->height_acceleration;
    next->force_raw = next->pressure + air_param.wheel_mass * (9.81f + next->wheel_acceleration);
    alpha = seeded ? dt / (air_param.force_tau + dt) : 1.0f;
    next->force = previous->force + alpha * (next->force_raw - previous->force);
    return (uint8_t)(isfinite(next->spring_force) && isfinite(next->pressure) && isfinite(next->height_velocity) && isfinite(next->height_acceleration) && isfinite(next->force_raw) && isfinite(next->force));
}

/* 均值迟滞计时 */
/*
 * 首次高/低支持力分别连续确认，空中上电不要求已有支撑历史。
 * AIR每拍更新各腿峰值；压缩模式使用每腿独立计时，任一腿达时即可恢复整机接地。
 * landing_enabled关闭时只观察峰值/压缩，不退出AIR；非压缩模式才使用高支持力落地。
 */
static void Air_State_Update(air_detection_t *state, const air_input_t *input)
{
    const float dt = MACHINE_LQR_DT;
    float duration;
    float length;
    uint8_t condition;

    if (!state->initialized)
    {
        state->elapsed = state->mean_force > air_param.force_on ? state->elapsed + dt : 0.0f;
        state->initial_unload_elapsed = state->mean_force < air_param.force_off ? state->initial_unload_elapsed + dt : 0.0f;
        if (state->elapsed + 1.0e-6f >= air_param.time_on)
        {
            state->initialized = 1u;
            state->elapsed = 0.0f;
            state->initial_unload_elapsed = 0.0f;
        }
        else if (state->initial_unload_elapsed + 1.0e-6f >= air_param.time_off)
        {
            state->initialized = 1u;
            state->elapsed = 0.0f;
            state->initial_unload_elapsed = 0.0f;
            /* 已知AIR的失效重确认保留原峰值；只为新AIR播种。 */
            if (!state->flight)
            {
                state->flight = 1u;
                memset(state->landing, 0, sizeof(state->landing));
                for (uint8_t side = 0u; side < 2u; side++)
                {
                    state->landing[side].peak_length = input->leg[side]->output.virtual_leg_length;
                }
            }
        }
        return;
    }
    if (state->flight)
    {
        for (uint8_t side = 0u; side < 2u; side++)
        {
            length = input->leg[side]->output.virtual_leg_length;
            state->landing[side].peak_length = fmaxf(state->landing[side].peak_length, length);
            state->landing[side].compression = state->landing[side].peak_length - length;
        }
        if (!air_param.landing_enabled)
        {
            Air_Timer_Reset(state);
            return;
        }
        if (air_param.compression_enabled)
        {
            for (uint8_t side = 0u; side < 2u; side++)
            {
                state->landing[side].elapsed = state->landing[side].compression >= air_param.compression_on ? state->landing[side].elapsed + dt : 0.0f;
            }
            state->elapsed = fmaxf(state->landing[0].elapsed, state->landing[1].elapsed);
            if (state->elapsed + 1.0e-6f >= air_param.compression_time)
            {
                state->flight = 0u;
                state->elapsed = 0.0f;
            }
            return;
        }
        state->landing[0].elapsed = 0.0f;
        state->landing[1].elapsed = 0.0f;
    }
    condition = state->flight ? (uint8_t)(state->mean_force > air_param.force_on) : (uint8_t)(state->mean_force < air_param.force_off);
    duration = state->flight ? air_param.time_on : air_param.time_off;
    state->elapsed = condition ? state->elapsed + dt : 0.0f;
    if (state->elapsed + 1.0e-6f >= duration)
    {
        state->flight = (uint8_t)!state->flight;
        state->elapsed = 0.0f;
        if (state->flight)
        {
            memset(state->landing, 0, sizeof(state->landing));
            for (uint8_t side = 0u; side < 2u; side++)
            {
                state->landing[side].peak_length = input->leg[side]->output.virtual_leg_length;
            }
        }
    }
}

/* 固定执行周期 */
/*
 * 公共反馈/四髋可用 → 双腿估力 → 状态机；缓存未更新也按MACHINE_LQR_DT处理。
 * 首次或失败恢复先播种差分，连续条件计时每个有效执行拍推进。
 * 算法只更新state，执行层负责真实控制切换。
 */
void Air_Detection_Update(air_detection_t *state, const air_input_t *input)
{
    air_leg_t air_leg_temp[2];

    if (!air_param.enabled)
    {
        Air_Detection_Reset(state);
        return;
    }
    if (!input->imu->online || !input->imu->pitch_world_valid || !input->leg[0]->output.valid || !input->leg[1]->output.valid || !input->joint_ready || !input->compensation->spring.valid || (machine->gravity != NULL && !input->compensation->gravity.valid))
    {
        Air_Invalid(state);
        return;
    }
    memset(air_leg_temp, 0, sizeof(air_leg_temp));
    for (uint8_t side = 0u; side < 2u; side++)
    {
        if (!Air_Estimate(&air_leg_temp[side], &state->leg[side], input, side, state->valid, input->imu->accel_vertical))
        {
            Air_Invalid(state);
            return;
        }
    }
    memcpy(state->leg, air_leg_temp, sizeof(air_leg_temp));
    state->mean_force = air_leg_temp[0].force * 0.5f + air_leg_temp[1].force * 0.5f;
    state->valid = 1u;
    state->fault = 0u;
    Air_State_Update(state, input);
}
