/*
 * 文件职责：离地/落地状态估计（air_detection.c）。
 * 输入/来源：执行任务传入的双腿几何、四髋反馈力矩/接收时刻与IMU。
 * 输出/去向：发布支持力、armed/valid/flight及压缩诊断，执行层决定是否切换控制。
 * 关键边界：新反馈才推进计时；输入无效保留flight并清连续计时；不直接改腿长目标或电机输出。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "air_detection.h"
#include "Attitude_Algorithm.h"
#include "gas_spring.h"
#include "gravity_comp.h"
#include "machine_config.h"
#include <math.h>
#include <string.h>

air_param_t air_param = {
    .enabled = 1u, .control_enabled = 1u, .vofa_page = 4u,
    .landing_enabled = 1u, .compression_enabled = 1u,
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    .wheel_mass = 0.65f, .force_off = 20.0f, .force_on = 35.0f,
    .hip_max = 10.0f,
#else
    .wheel_mass = 0.13463f, .force_off = 6.0f, .force_on = 18.0f,
    .hip_max = 8.0f,
#endif
    .time_off = 0.01f, .time_on = 0.02f,
    .sample_timeout = 0.05f, .sample_skew = 0.005f,
    .derivative_tau = 0.01f, .force_tau = 0.01f,
    .compression_on = 0.01f, .compression_time = 0.05f,
};
air_detection_t air_detection;
volatile air_detection_t air_debug;

/* 重新等待样本 */
/*
 * 清空本次检测所有历史，包括armed、flight、样本时间和悬空峰值。
 * 许可/模式退出调用此接口；再次投入必须重新确认初次支撑，不能把清零当作真实落地事件。
 */
void Air_Detection_Reset(air_detection_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

/*
 * 按enabled/control_enabled/flight决定是否选择空中控制。
 * valid表示观测是否新鲜，不等同于接地；短暂无效期间仍可保留上次AIR状态。
 */
uint8_t Air_Detection_Active(const air_detection_t *state)
{
    return (uint8_t)(state != NULL && air_param.enabled && air_param.control_enabled
        && state->flight);
}

/* 清条件计时 */
static void Air_Timer_Reset(air_detection_t *state)
{
    state->elapsed = 0.0f;
    state->landing[0].elapsed = 0.0f;
    state->landing[1].elapsed = 0.0f;
}

/* 失效不算接地 */
/*
 * 观测失效时清连续条件计时并保留flight和悬空峰值。
 * valid_ns用于判断持续失效是否报fault；恢复时重新播种差分，不能借用失效时间确认接地。
 */
static void Air_Invalid(air_detection_t *state, uint64_t now)
{
    state->valid = 0u;
    Air_Timer_Reset(state);
    if (state->valid_ns == 0u)
    {
        state->valid_ns = now;
    }
    if (now < state->valid_ns
        || (double)(now - state->valid_ns) * 1.0e-9 > air_param.sample_timeout)
    {
        state->fault = 1u;
    }
}

/* 参数有限且有序 */
static uint8_t Air_Params_Valid(void)
{
    return (uint8_t)(isfinite(air_param.wheel_mass) && air_param.wheel_mass > 0.0f
        && isfinite(air_param.force_off) && air_param.force_off >= 0.0f
        && isfinite(air_param.force_on) && air_param.force_on > air_param.force_off
        && isfinite(air_param.time_off) && air_param.time_off > 0.0f
        && isfinite(air_param.time_on) && air_param.time_on > 0.0f
        && isfinite(air_param.sample_timeout) && air_param.sample_timeout > 0.0f
        && isfinite(air_param.sample_skew) && air_param.sample_skew >= 0.0f
        && isfinite(air_param.derivative_tau) && air_param.derivative_tau >= 0.0f
        && isfinite(air_param.force_tau) && air_param.force_tau >= 0.0f
        && isfinite(air_param.hip_max) && air_param.hip_max > 0.0f
        && (!air_param.compression_enabled
            || (isfinite(air_param.compression_on) && air_param.compression_on > 0.0f
                && isfinite(air_param.compression_time) && air_param.compression_time > 0.0f)));
}

/* 单腿支持力 */
/*
 * 由同侧前后髋反馈反解电机轴向力/摆矩，计回实体弹簧并扣除自重摆矩。
 * 将力投影到世界竖直方向，再用机体/腿投影加速度估计轮重动态项；滤波得到本腿FN。
 * 首帧没有可靠差分，必须先播种；这里计算的是支持力估计，不是长度PID给定或发送力矩。
 */
static uint8_t Air_Estimate(air_leg_t *next, const air_leg_t *previous,
    const air_input_t *input, uint8_t side, uint8_t seeded, float az)
{
    const leg_state_t *leg;
    gravity_leg_result_t gravity;
    float theta;
    float omega;
    float length;
    float dt;
    float alpha;
    float force;
    float torque;

    leg = input->leg[side];
    if (leg == NULL || !leg->output.valid || !leg->output.force_valid
        || !Leg_Force_Map_Inverse(leg, input->torque[side], &next->motor_force, &next->motor_torque))
    {
        return 0u;
    }
    length = leg->output.virtual_leg_length;
    theta = remainderf(leg->output.virtual_leg_angle - leg->config.offset_phi0
        - input->imu->euler_rad[ATTITUDE_ROLL], LEG_2PI);
    omega = leg->output.d_virtual_leg_angle - input->imu->gyro_rad_s[1];
    if (!isfinite(length) || length <= 1.0e-3f || !isfinite(theta) || !isfinite(omega))
    {
        return 0u;
    }
    next->spring_force = Gas_Spring_Force(length);
    next->gravity_torque = 0.0f;
    if (machine->gravity != NULL)
    {
        if (!Gravity_Comp_Compute(machine->gravity, length,
            leg->output.virtual_leg_angle - input->imu->euler_rad[ATTITUDE_ROLL], &gravity))
        {
            return 0u;
        }
        next->gravity_torque = gravity.torque;
    }
    force = next->motor_force + next->spring_force;
    torque = next->motor_torque - next->gravity_torque;
    next->pressure = force * cosf(theta) - torque * sinf(theta) / length;
    next->height_velocity = leg->output.d_virtual_leg_length * cosf(theta)
        - length * omega * sinf(theta);
    next->height_acceleration = 0.0f;
    dt = seeded ? (float)((double)(next->sample_ns - previous->sample_ns) * 1.0e-9) : 0.0f;
    if (seeded)
    {
        if (!(dt > 0.0f) || dt > air_param.sample_timeout)
        {
            return 0u;
        }
        alpha = dt / (air_param.derivative_tau + dt);
        next->height_acceleration = previous->height_acceleration + alpha
            * ((next->height_velocity - previous->height_velocity) / dt - previous->height_acceleration);
    }
    next->wheel_acceleration = az - next->height_acceleration;
    next->force_raw = next->pressure + air_param.wheel_mass * (9.81f + next->wheel_acceleration);
    alpha = seeded ? dt / (air_param.force_tau + dt) : 1.0f;
    next->force = previous->force + alpha * (next->force_raw - previous->force);
    return (uint8_t)(isfinite(next->spring_force) && isfinite(next->pressure)
        && isfinite(next->height_velocity) && isfinite(next->height_acceleration)
        && isfinite(next->force_raw) && isfinite(next->force));
}

/* 均值迟滞计时 */
/*
 * 先确认初次支撑armed，再按低支持力的连续时间确认离地。
 * AIR有效新样本更新各腿峰值；压缩模式使用每腿独立计时，任一腿达时即可恢复整机接地。
 * landing_enabled关闭时只观察峰值/压缩，不退出AIR；非压缩模式才使用高支持力落地。
 */
static void Air_State_Update(air_detection_t *state, const air_input_t *input,
    float dt, const float leg_dt[2])
{
    float duration;
    float length;
    uint8_t condition;
    uint8_t side;

    if (!state->armed)
    {
        state->elapsed = state->mean_force > air_param.force_on ? state->elapsed + dt : 0.0f;
        if (state->elapsed + 1.0e-6f >= air_param.time_on)
        {
            state->armed = 1u;
            state->elapsed = 0.0f;
        }
        return;
    }
    if (state->flight)
    {
        for (side = 0u; side < 2u; side++)
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
            for (side = 0u; side < 2u; side++)
            {
                state->landing[side].elapsed = state->landing[side].compression >= air_param.compression_on
                    ? state->landing[side].elapsed + leg_dt[side] : 0.0f;
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
    condition = state->flight ? (uint8_t)(state->mean_force > air_param.force_on)
        : (uint8_t)(state->mean_force < air_param.force_off);
    duration = state->flight ? air_param.time_on : air_param.time_off;
    state->elapsed = condition ? state->elapsed + dt : 0.0f;
    if (state->elapsed + 1.0e-6f >= duration)
    {
        state->flight = (uint8_t)!state->flight;
        state->elapsed = 0.0f;
        if (state->flight)
        {
            memset(state->landing, 0, sizeof(state->landing));
            for (side = 0u; side < 2u; side++)
            {
                state->landing[side].peak_length = input->leg[side]->output.virtual_leg_length;
            }
        }
    }
}

/* 新反馈才推进 */
/*
 * 完整入口：许可/参数/IMU检查 → 四髋时间龄和偏差检查 → 双腿新样本检查 → 估力 → 状态机。
 * 以反馈接收时间计算dt，不以任务被调用次数累计；双腿的最早髋样本都要更新才推进。
 * 旧帧直接返回；无效恢复先播种且dt=0。算法只更新state，执行层负责真实控制切换。
 */
void Air_Detection_Update(air_detection_t *state, const air_input_t *input)
{
    air_leg_t next[2];
    uint64_t oldest;
    uint64_t newest;
    uint64_t minimum;
    uint64_t maximum;
    uint8_t side;
    uint8_t fresh;
    uint8_t seeded;
    float dt;
    float leg_dt[2];
    float az;

    if (state == NULL || input == NULL)
    {
        return;
    }
    if (!air_param.enabled || !input->permit)
    {
        Air_Detection_Reset(state);
        return;
    }
    if (!Air_Params_Valid())
    {
        state->valid = 0u;
        state->fault = 1u;
        Air_Timer_Reset(state);
        return;
    }
    if (input->imu == NULL || !input->imu->online
        || !Attitude_Accel_Vertical(input->imu, &az))
    {
        Air_Invalid(state, input->now_ns);
        return;
    }
    if (state->imu_seen_ns == 0u || state->imu_sequence != input->imu->last_timestamp_ms)
    {
        state->imu_sequence = input->imu->last_timestamp_ms;
        state->imu_seen_ns = input->now_ns;
    }
    if (input->now_ns < state->imu_seen_ns
        || (double)(input->now_ns - state->imu_seen_ns) * 1.0e-9 > air_param.sample_timeout)
    {
        Air_Invalid(state, input->now_ns);
        return;
    }
    oldest = input->now_ns;
    newest = 0u;
    fresh = 1u;
    memset(next, 0, sizeof(next));
    for (side = 0u; side < 2u; side++)
    {
        minimum = input->rx_ns[side][0] < input->rx_ns[side][1]
            ? input->rx_ns[side][0] : input->rx_ns[side][1];
        maximum = input->rx_ns[side][0] > input->rx_ns[side][1]
            ? input->rx_ns[side][0] : input->rx_ns[side][1];
        next[side].sample_ns = minimum;
        if (minimum < oldest) { oldest = minimum; }
        if (maximum > newest) { newest = maximum; }
        fresh &= (uint8_t)(minimum > state->leg[side].sample_ns);
    }
    if (oldest == 0u || newest > input->now_ns
        || (double)(input->now_ns - oldest) * 1.0e-9 > air_param.sample_timeout
        || (double)(newest - oldest) * 1.0e-9 > air_param.sample_skew)
    {
        Air_Invalid(state, input->now_ns);
        return;
    }
    if (!fresh)
    {
        return;
    }
    seeded = state->valid;
    for (side = 0u; side < 2u; side++)
    {
        leg_dt[side] = seeded
            ? (float)((double)(next[side].sample_ns - state->leg[side].sample_ns) * 1.0e-9) : 0.0f;
        if (!Air_Estimate(&next[side], &state->leg[side], input, side, seeded, az))
        {
            Air_Invalid(state, input->now_ns);
            return;
        }
    }
    minimum = state->leg[0].sample_ns < state->leg[1].sample_ns
        ? state->leg[0].sample_ns : state->leg[1].sample_ns;
    maximum = next[0].sample_ns < next[1].sample_ns ? next[0].sample_ns : next[1].sample_ns;
    dt = seeded && maximum > minimum ? (float)((double)(maximum - minimum) * 1.0e-9) : 0.0f;
    memcpy(state->leg, next, sizeof(next));
    state->mean_force = next[0].force * 0.5f + next[1].force * 0.5f;
    state->valid = 1u;
    state->fault = 0u;
    state->valid_ns = input->now_ns;
    Air_State_Update(state, input, dt, leg_dt);
}
