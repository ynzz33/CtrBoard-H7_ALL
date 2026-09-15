#include "rl_torque.h"

#include <math.h>
#include <string.h>

#define RL_TQ_ACTION_CLIP      100.0f
#define RL_TQ_POS_SCALE        0.5f
#define RL_TQ_VEL_SCALE        10.0f
#define RL_TQ_SERIAL_LIMIT     1000.0f
#define RL_TQ_LEG_LIMIT        35.0f
#define RL_TQ_WHEEL_LIMIT      5.0f
#define RL_TQ_JUMP_WHEEL_LIMIT 4.0f
#define RL_TQ_TWO_PI           6.283185307179586f
#define RL_TQ_VSHANK_MIN       2.277f
#define RL_TQ_VSHANK_MAX       3.133f

/* 限幅 */
static float RL_Torque_Clip(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 将环绕角目标放到与当前反馈最近的 2pi 分支，供普通 PID 使用。 */
static float RL_Torque_Nearest_Angle_Target(float target, float current)
{
    float error = target - current;
    while (error > 3.141592653589793f) error -= RL_TQ_TWO_PI;
    while (error <= -3.141592653589793f) error += RL_TQ_TWO_PI;
    return current + error;
}

/* 将虚拟小腿目标归一到正工作支后限幅。 */
static float RL_Torque_Clamp_VShank_Target(float target)
{
    while (target < 0.0f) target += RL_TQ_TWO_PI;
    while (target >= RL_TQ_TWO_PI) target -= RL_TQ_TWO_PI;
    if (target < RL_TQ_VSHANK_MIN) target = RL_TQ_VSHANK_MIN;
    if (target > RL_TQ_VSHANK_MAX) target = RL_TQ_VSHANK_MAX;
    return target;
}

/* 检查数组 */
static uint8_t RL_Torque_Array_Finite(const float *data, uint32_t count)
{
    uint32_t i;
    if (data == NULL) return 0u;
    for (i = 0u; i < count; i++)
    {
        if (!isfinite(data[i])) return 0u;
    }
    return 1u;
}

/* 同步 PID 参数 */
static void RL_Torque_PID_Sync(rl_torque_state_t *state,
                               const rl_torque_param_t *param)
{
    uint32_t i;
    if (state->param_ref == param)
    {
        return;
    }
    memset(state->controller, 0, sizeof(state->controller));
    for (i = 0u; i < RL_ACTION_SIZE; i++)
    {
        if (i == 2u || i == 5u)
        {
            PID_struct_init(&state->controller[i], POSITION_PID,
                RL_TQ_SERIAL_LIMIT, 0.0f, param->wheel_kp[i == 5u],
                0.0f, 0.0f, 0.0f, 0.0f);
        }
        else
        {
            PID_struct_init(&state->controller[i], POSITION_PID,
                RL_TQ_SERIAL_LIMIT, 0.0f, param->p_gains[i],
                0.0f, param->d_gains[i], 0.0f, 0.0f);
        }
    }
    state->param_ref = param;
}

/* 初始化参数 */
void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model)
{
    if (param == NULL) return;
    memset(param, 0, sizeof(*param));

    if (model == RL_MODEL_JUMP)
    {
        const float dof_pos[6] = { 0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f};
        const float p_gains[6] = { 3.0f, 3.0f, 0.0f, 3.0f, 3.0f, 0.0f};
        const float d_gains[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        memcpy(param->dof_pos, dof_pos, sizeof(dof_pos));
        memcpy(param->p_gains, p_gains, sizeof(p_gains));
        memcpy(param->d_gains, d_gains, sizeof(d_gains));
        param->wheel_kp[0] = 0.1f;
        param->wheel_kp[1] = 0.1f;
        param->jump_mode = 1u;
    }
    else if (model == RL_MODEL_PIN)
    {
        const float dof_pos[6] = {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
        const float p_gains[6] = {3.0f, 3.0f, 0.0f, 3.0f, 3.0f, 0.0f};
        const float d_gains[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        memcpy(param->dof_pos, dof_pos, sizeof(dof_pos));
        memcpy(param->p_gains, p_gains, sizeof(p_gains));
        memcpy(param->d_gains, d_gains, sizeof(d_gains));
        param->wheel_kp[0] = 0.1f;
        param->wheel_kp[1] = 0.1f;
        param->spin_mode = 1u;
    }
    else
    {
        const float dof_pos[6] = {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
        const float p_gains[6] = {0.5f, 0.5f, 0.0f, 0.5f, 0.5f, 0.0f};
        const float d_gains[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        memcpy(param->dof_pos, dof_pos, sizeof(dof_pos));
        memcpy(param->p_gains, p_gains, sizeof(p_gains));
        memcpy(param->d_gains, d_gains, sizeof(d_gains));
        param->wheel_kp[0] = 0.1f;
        param->wheel_kp[1] = 0.1f;
    }
}

/* 初始化状态 */
void RL_Torque_State_Init(rl_torque_state_t *state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
}

/* 限制输出 */
void RL_Torque_Clamp_Output(float torque[RL_TQ_NUM], float leg_limit,
                            float wheel_limit)
{
    uint32_t i;

    if (torque == NULL)
    {
        return;
    }
    for (i = RL_TQ_L_THIGH; i <= RL_TQ_R_SHANK; i++)
    {
        torque[i] = RL_Torque_Clip(torque[i], leg_limit);
    }
    torque[RL_TQ_L_WHEEL] = RL_Torque_Clip(torque[RL_TQ_L_WHEEL],
        wheel_limit);
    torque[RL_TQ_R_WHEEL] = RL_Torque_Clip(torque[RL_TQ_R_WHEEL],
        wheel_limit);
}

/* 力域转换 + 气弹簧补偿 */
static void RL_Torque_Gas_Spring(const leg_output_t *out, float tau_f, float tau_b,
                                 float gas, int8_t mirror,
                                 float *tau_f_out, float *tau_b_out)
{
    float force;
    float torque;
    float det;

    *tau_f_out = tau_f;
    *tau_b_out = tau_b;
    if (!out->force_valid || gas == 0.0f) return;

    det = out->force_det;
    force = (out->force_map[1][1] * tau_f - out->force_map[0][1] * tau_b) / det;
    torque = (-out->force_map[1][0] * tau_f + out->force_map[0][0] * tau_b) / det;
    force -= (float)mirror * gas * out->l0;
    *tau_f_out = out->force_map[0][0] * force + out->force_map[0][1] * torque;
    *tau_b_out = out->force_map[1][0] * force + out->force_map[1][1] * torque;
}

/* 计算力矩 */
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          float torque[RL_TQ_NUM])
{
    float act[RL_ACTION_SIZE];
    float pos_ref[RL_ACTION_SIZE];
    float vel_ref[RL_ACTION_SIZE];
    float q[RL_ACTION_SIZE];
    float qd[RL_ACTION_SIZE];
    float tau_v[RL_ACTION_SIZE];
    float tau_f[2];
    float tau_b[2];
    float wheel_limit;
    float position_target;
    uint32_t i;

    for (i = 0u; i < RL_TQ_NUM; i++) torque[i] = 0.0f;
    if (leg_l == NULL || leg_r == NULL || param == NULL || state == NULL
        || wheel_vel == NULL || action == NULL)
        return 0u;
    if (!leg_l->output.valid || !leg_r->output.valid) return 0u;
    if (!RL_Torque_Array_Finite(action, RL_ACTION_SIZE)
        || !RL_Torque_Array_Finite(wheel_vel, 2u))
        return 0u;
    RL_Torque_PID_Sync(state, param);

    /* 动作限幅 */
    for (i = 0u; i < RL_ACTION_SIZE; i++)
        act[i] = RL_Torque_Clip(action[i], RL_TQ_ACTION_CLIP);

    /* 交错布局 */
    q[0] = leg_l->input.hip_f;
    q[1] = leg_l->output.virtual_shank;
    q[2] = 0.0f;
    q[3] = leg_r->input.hip_f;
    q[4] = leg_r->output.virtual_shank;
    q[5] = 0.0f;
    qd[0] = leg_l->input.d_hip_f;
    qd[1] = leg_l->output.d_virtual_shank;
    qd[2] = wheel_vel[0];
    qd[3] = leg_r->input.d_hip_f;
    qd[4] = leg_r->output.d_virtual_shank;
    qd[5] = wheel_vel[1];

    /* 位置/速度目标 */
    for (i = 0u; i < RL_ACTION_SIZE; i++)
    {
        pos_ref[i] = 0.0f;
        vel_ref[i] = 0.0f;
    }
    pos_ref[0] = act[0] * RL_TQ_POS_SCALE;
    pos_ref[1] = act[1] * RL_TQ_POS_SCALE;
    pos_ref[3] = act[3] * RL_TQ_POS_SCALE;
    pos_ref[4] = act[4] * RL_TQ_POS_SCALE;
    vel_ref[2] = act[2] * RL_TQ_VEL_SCALE;
    vel_ref[5] = act[5] * RL_TQ_VEL_SCALE;

    /* 虚拟关节 PID */
    for (i = 0u; i < RL_ACTION_SIZE; i++)
    {
        if (i == 2u || i == 5u)
        {
            tau_v[i] = pid_calc(&state->controller[i], qd[i], vel_ref[i],
                0.002f);
        }
        else
        {
            position_target = pos_ref[i] + param->dof_pos[i];
            if (i == 1u || i == 4u)
            {
                position_target = RL_Torque_Clamp_VShank_Target(
                    position_target);
            }
            if (i == 0u || i == 1u || i == 3u || i == 4u)
            {
                position_target = RL_Torque_Nearest_Angle_Target(
                    position_target, q[i]);
            }
            tau_v[i] = pid_calc(&state->controller[i], q[i],
                position_target, 0.002f);
        }
    }
    memcpy(state->virtual_torque, tau_v, sizeof(state->virtual_torque));

    /* 大腿 PID 分量由前后髋链条共同驱动；virtual_shank 分量保持原雅可比映射。 */
    tau_f[0] = -tau_v[0] + tau_v[1] * leg_l->output.vshank_jac[1];
    tau_b[0] = -tau_v[0] + tau_v[1] * leg_l->output.vshank_jac[0];
    tau_f[1] = -tau_v[3] + tau_v[4] * leg_r->output.vshank_jac[1];
    tau_b[1] = -tau_v[3] + tau_v[4] * leg_r->output.vshank_jac[0];

    /* 力域转换 + 气弹簧 */
    RL_Torque_Gas_Spring(&leg_l->output, tau_f[0], tau_b[0],
                         param->gas_spring[0], leg_l->config.mirror,
                         &tau_f[0], &tau_b[0]);
    RL_Torque_Gas_Spring(&leg_r->output, tau_f[1], tau_b[1],
                         param->gas_spring[1], leg_r->config.mirror,
                         &tau_f[1], &tau_b[1]);

    /* 物理通道输出 */
    torque[RL_TQ_L_THIGH] = RL_Torque_Clip(tau_f[0], RL_TQ_LEG_LIMIT);
    torque[RL_TQ_L_SHANK] = RL_Torque_Clip(tau_b[0], RL_TQ_LEG_LIMIT);
    torque[RL_TQ_R_THIGH] = RL_Torque_Clip(tau_f[1], RL_TQ_LEG_LIMIT);
    torque[RL_TQ_R_SHANK] = RL_Torque_Clip(tau_b[1], RL_TQ_LEG_LIMIT);
    wheel_limit = param->jump_mode ? RL_TQ_JUMP_WHEEL_LIMIT : RL_TQ_WHEEL_LIMIT;
    torque[RL_TQ_L_WHEEL] = RL_Torque_Clip(-tau_v[2], wheel_limit);
    torque[RL_TQ_R_WHEEL] = RL_Torque_Clip(-tau_v[5], wheel_limit);

    /* 斜率限制, 0=关 */
    if (state->valid)
    {
        for (i = 0u; i < RL_TQ_NUM; i++)
        {
            float step = param->max_step[i];
            float diff;

            if (step <= 0.0f) continue;
            diff = torque[i] - state->last_torque[i];
            if (diff > step) diff = step;
            if (diff < -step) diff = -step;
            torque[i] = state->last_torque[i] + diff;
        }
    }
    memcpy(state->last_torque, torque, sizeof(state->last_torque));
    state->valid = 1u;
    return 1u;
}
