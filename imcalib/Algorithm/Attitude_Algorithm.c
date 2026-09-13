#include "Attitude_Algorithm.h"
#include <math.h>
#include <string.h>

/* 限制数值 */
static float Attitude_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* 检查有限值 */
static bool Attitude_Is_Finite3(const float value[3])
{
    return !isnan(value[0]) && !isnan(value[1]) && !isnan(value[2])
        && !isinf(value[0]) && !isinf(value[1]) && !isinf(value[2]);
}

/* 检查四元数 */
static bool Attitude_Is_Finite4(const float value[4])
{
    return !isnan(value[0]) && !isnan(value[1]) && !isnan(value[2])
        && !isnan(value[3]) && !isinf(value[0]) && !isinf(value[1])
        && !isinf(value[2]) && !isinf(value[3]);
}

/* 更新姿态输出 */
static void Attitude_Update_Output(imu_state_t *state)
{
    const float q0 = state->output.quat[0];
    const float q1 = state->output.quat[1];
    const float q2 = state->output.quat[2];
    const float q3 = state->output.quat[3];
    const float rad_to_deg = 57.2957795131f;

    state->output.rotation_t[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    state->output.rotation_t[0][1] = 2.0f * (q1 * q2 + q0 * q3);
    state->output.rotation_t[0][2] = 2.0f * (q1 * q3 - q0 * q2);
    state->output.rotation_t[1][0] = 2.0f * (q1 * q2 - q0 * q3);
    state->output.rotation_t[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    state->output.rotation_t[1][2] = 2.0f * (q2 * q3 + q0 * q1);
    state->output.rotation_t[2][0] = 2.0f * (q1 * q3 + q0 * q2);
    state->output.rotation_t[2][1] = 2.0f * (q2 * q3 - q0 * q1);
    state->output.rotation_t[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    state->output.euler_deg[ATTITUDE_PITCH] = asinf(Attitude_Clamp(
        2.0f * (q2 * q3 + q0 * q1), -1.0f, 1.0f)) * rad_to_deg;
    state->output.euler_deg[ATTITUDE_ROLL] = atan2f(2.0f * (q0 * q2 - q1 * q3),
        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * rad_to_deg;
    state->output.euler_deg[ATTITUDE_YAW] = atan2f(2.0f * (q1 * q2 - q0 * q3),
        1.0f - 2.0f * (q1 * q1 + q3 * q3)) * rad_to_deg;
    state->output.euler_rad[ATTITUDE_PITCH] = state->output.euler_deg[ATTITUDE_PITCH]
        * 0.01745329251994f;
    state->output.euler_rad[ATTITUDE_ROLL] = state->output.euler_deg[ATTITUDE_ROLL]
        * 0.01745329251994f;
    state->output.euler_rad[ATTITUDE_YAW] = state->output.euler_deg[ATTITUDE_YAW]
        * 0.01745329251994f;
}

/* 初始化姿态 */
void Attitude_Init(imu_state_t *state)
{
    uint8_t axis;

    if (state == NULL) return;

    memset(state, 0, sizeof(*state));
    state->output.quat[0] = 1.0f;
    for (axis = 0u; axis < 3u; axis++)
        PID_struct_init(&state->mahony_pid[axis], POSITION_PID,
                        30.0f, 5.0f, ATTITUDE_MAHONY_KP,
                        ATTITUDE_MAHONY_KI, 0.0f, 0.0f, 0.0f);
    state->initialized = 1u;
    Attitude_Update_Output(state);
}

/* 转换单位 */
void IMU_State_Convert_Unit(imu_state_t *state)
{
    uint8_t axis;

    if (state == NULL) return;
    for (axis = 0u; axis < 3u; axis++)
    {
        state->input.gyro_rad_s[axis] = state->input.gyro_dps[axis]
            * 0.01745329251994f;
        state->reference.euler_rad[axis] = state->reference.euler_deg[axis]
            * 0.01745329251994f;
    }
}

#if 0
/* Mahony 解算 */
bool Attitude_Update(imu_state_t *state)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float gx;
    float gy;
    float gz;
    float q_norm;
    float qa;
    float qb;
    float qc;
    float qd;

    if (state == NULL || !Attitude_Is_Finite3(state->input.gyro_rad_s)
        || state->input.dt_s < ATTITUDE_MIN_DT_S
        || state->input.dt_s > ATTITUDE_MAX_DT_S)
        return false;

    q0 = state->output.quat[0];
    q1 = state->output.quat[1];
    q2 = state->output.quat[2];
    q3 = state->output.quat[3];
    gx = state->input.gyro_rad_s[0];
    gy = state->input.gyro_rad_s[1];
    gz = state->input.gyro_rad_s[2];
    state->output.accel_trust = 0.0f;
    memset(state->output.accel_normed, 0, sizeof(state->output.accel_normed));
    memset(state->output.mahony_error, 0, sizeof(state->output.mahony_error));

    /* 加速度仅在接近重力时参与校正。 */
    if (Attitude_Is_Finite3(state->input.accel_g))
    {
        const float acc_norm = sqrtf(state->input.accel_g[0] * state->input.accel_g[0]
            + state->input.accel_g[1] * state->input.accel_g[1]
            + state->input.accel_g[2] * state->input.accel_g[2]);

        if (acc_norm > ATTITUDE_MIN_ACC_NORM_G)
        {
            const float acc_dev = fabsf(acc_norm - 1.0f);
            const float tx = 2.0f * (q1 * q3 - q0 * q2);
            const float ty = 2.0f * (q2 * q3 + q0 * q1);
            const float tz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

            state->output.accel_normed[0] = state->input.accel_g[0] / acc_norm;
            state->output.accel_normed[1] = state->input.accel_g[1] / acc_norm;
            state->output.accel_normed[2] = state->input.accel_g[2] / acc_norm;
            /* 线加速度越大，越少相信加速度方向。 */
            if (acc_dev <= ATTITUDE_ACC_TRUST_FULL)
                state->output.accel_trust = 1.0f;
            else if (acc_dev < ATTITUDE_ACC_TRUST_ZERO)
                state->output.accel_trust = (ATTITUDE_ACC_TRUST_ZERO - acc_dev)
                    / (ATTITUDE_ACC_TRUST_ZERO - ATTITUDE_ACC_TRUST_FULL);

            state->output.mahony_error[0] = state->output.accel_trust
                * (state->output.accel_normed[2] * ty - state->output.accel_normed[1] * tz);
            state->output.mahony_error[1] = state->output.accel_trust
                * (state->output.accel_normed[0] * tz - state->output.accel_normed[2] * tx);
            state->output.mahony_error[2] = state->output.accel_trust
                * (state->output.accel_normed[1] * tx - state->output.accel_normed[0] * ty);
        }
    }

    /* PID 将重力方向误差变为陀螺补偿。 */
    state->output.mahony_output[0] = pid_calc(&state->mahony_pid[0],
        state->output.mahony_error[0], 0.0f, state->input.dt_s);
    state->output.mahony_output[1] = pid_calc(&state->mahony_pid[1],
        state->output.mahony_error[1], 0.0f, state->input.dt_s);
    state->output.mahony_output[2] = pid_calc(&state->mahony_pid[2],
        state->output.mahony_error[2], 0.0f, state->input.dt_s);
    gx += state->output.mahony_output[0];
    gy += state->output.mahony_output[1];
    gz += state->output.mahony_output[2];

    /* 使用补偿后的角速度积分四元数。 */
    qa = q0;
    qb = q1;
    qc = q2;
    qd = q3;
    q0 += 0.5f * state->input.dt_s * (-qb * gx - qc * gy - qd * gz);
    q1 += 0.5f * state->input.dt_s * ( qa * gx + qc * gz - qd * gy);
    q2 += 0.5f * state->input.dt_s * ( qa * gy - qb * gz + qd * gx);
    q3 += 0.5f * state->input.dt_s * ( qa * gz + qb * gy - qc * gx);

    q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (isnan(q_norm) || isinf(q_norm) || q_norm < 0.001f)
    {
        Attitude_Init(state);
        return false;
    }

    state->output.quat[0] = q0 / q_norm;
    state->output.quat[1] = q1 / q_norm;
    state->output.quat[2] = q2 / q_norm;
    state->output.quat[3] = q3 / q_norm;
    /* 归一化后同步更新欧拉角和旋转矩阵。 */
    Attitude_Update_Output(state);
    return true;
}
#endif

/* HI229 姿态核心 */
bool Attitude_Update_From_HI229(imu_state_t *state)
{
    float q_norm;
    float acc_norm;
    uint8_t axis;

    if (state == NULL || !Attitude_Is_Finite3(state->input.gyro_dps)
        || !Attitude_Is_Finite3(state->input.accel_g)
        || !Attitude_Is_Finite4(state->reference.quat)
        || !Attitude_Is_Finite3(state->reference.euler_deg))
        return false;

    q_norm = sqrtf(state->reference.quat[0] * state->reference.quat[0]
        + state->reference.quat[1] * state->reference.quat[1]
        + state->reference.quat[2] * state->reference.quat[2]
        + state->reference.quat[3] * state->reference.quat[3]);
    if (!isfinite(q_norm) || q_norm < 0.001f)
        return false;

    for (axis = 0u; axis < 4u; axis++)
        state->output.quat[axis] = state->reference.quat[axis] / q_norm;

    state->output.euler_deg[ATTITUDE_ROLL] = state->reference.euler_deg[0];
    state->output.euler_deg[ATTITUDE_PITCH] = state->reference.euler_deg[1];
    state->output.euler_deg[ATTITUDE_YAW] = state->reference.euler_deg[2];
    for (axis = 0u; axis < 3u; axis++)
        state->output.euler_rad[axis] = state->output.euler_deg[axis]
            * 0.01745329251994f;

    acc_norm = sqrtf(state->input.accel_g[0] * state->input.accel_g[0]
        + state->input.accel_g[1] * state->input.accel_g[1]
        + state->input.accel_g[2] * state->input.accel_g[2]);
    if (!isfinite(acc_norm) || acc_norm < ATTITUDE_MIN_ACC_NORM_G)
        return false;

    for (axis = 0u; axis < 3u; axis++)
        state->output.accel_normed[axis] = state->input.accel_g[axis] / acc_norm;
    state->output.accel_trust = 1.0f;
    memset(state->output.mahony_error, 0, sizeof(state->output.mahony_error));
    memset(state->output.mahony_output, 0, sizeof(state->output.mahony_output));
    Attitude_Update_Output(state);
    state->output.euler_deg[ATTITUDE_ROLL] = state->reference.euler_deg[0];
    state->output.euler_deg[ATTITUDE_PITCH] = state->reference.euler_deg[1];
    state->output.euler_deg[ATTITUDE_YAW] = state->reference.euler_deg[2];
    for (axis = 0u; axis < 3u; axis++)
        state->output.euler_rad[axis] = state->output.euler_deg[axis]
            * 0.01745329251994f;
    return true;
}

/* 兼容旧接口 */
bool Attitude_Update(imu_state_t *state)
{
    (void)state;
    return false;
}
