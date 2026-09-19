#include "rl_torque.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

#define RL_TQ_POS_SCALE        0.5f
#define RL_TQ_WHEEL_VEL_SCALE  20.0f
#define RL_TQ_WHEEL_VEL_MAX    62.0f    /* 轮侧满速 rad/s (转子满速 ÷ 总减速比) */
#define RL_TQ_VSHANK_MIN       2.277f
#define RL_TQ_VSHANK_MAX       3.133f

/* 虚拟关节索引 (仅限本文件内部) */
enum {
    VJ_L_THIGH = 0,
    VJ_L_SHANK = 1,
    VJ_L_WHEEL = 2,
    VJ_R_THIGH = 3,
    VJ_R_SHANK = 4,
    VJ_R_WHEEL = 5,
    VJ_NUM     = 6,
};

/* 检查数组 */
static uint8_t RL_Torque_Array_Finite(const float *data, uint32_t count)
{
    if (data == NULL)
    {
        return 0u;
    }
    for (uint32_t i = 0u; i < count; i++)
    {
        if (!isfinite(data[i]))
        {
            return 0u;
        }
    }
    return 1u;
}

/* 初始化参数 */
void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model)
{
    if (param == NULL)
    {
        return;
    }
    memset(param, 0, sizeof(*param));

    if (model == RL_MODEL_JUMP)
    {
        const float dof_pos[6] = {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
        const float p_gains[6] = {2.0f, 2.0f, 0.0f, 2.0f, 2.0f, 0.0f};
        const float d_gains[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        memcpy(param->dof_pos, dof_pos, sizeof(dof_pos));
        memcpy(param->p_gains, p_gains, sizeof(p_gains));
        memcpy(param->d_gains, d_gains, sizeof(d_gains));
        param->wheel_pid[0][0] = 5.0f; param->wheel_pid[0][1] = 0.0f; param->wheel_pid[0][2] = 0.0f;
        param->wheel_pid[1][0] = 5.0f; param->wheel_pid[1][1] = 0.0f; param->wheel_pid[1][2] = 0.0f;
        param->jump_mode = 1u;
    }
    else if (model == RL_MODEL_PIN)
    {
        const float dof_pos[6] = {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
        const float p_gains[6] = {2.0f, 2.0f, 0.0f, 2.0f, 2.0f, 0.0f};
        const float d_gains[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        memcpy(param->dof_pos, dof_pos, sizeof(dof_pos));
        memcpy(param->p_gains, p_gains, sizeof(p_gains));
        memcpy(param->d_gains, d_gains, sizeof(d_gains));
        param->wheel_pid[0][0] = 5.0f; param->wheel_pid[0][1] = 0.0f; param->wheel_pid[0][2] = 0.0f;
        param->wheel_pid[1][0] = 5.0f; param->wheel_pid[1][1] = 0.0f; param->wheel_pid[1][2] = 0.0f;
        param->spin_mode = 1u;
    }
    else
    {
        const float dof_pos[6] = {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f};
        const float p_gains[6] = {9.5f,9.5f, 0.0f, 9.5f, 9.5f, 0.0f};
        const float d_gains[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        memcpy(param->dof_pos, dof_pos, sizeof(dof_pos));
        memcpy(param->p_gains, p_gains, sizeof(p_gains));
        memcpy(param->d_gains, d_gains, sizeof(d_gains));
        param->wheel_pid[0][0] = 0.35f; param->wheel_pid[0][1] = 0.0f; param->wheel_pid[0][2] = 0.000f;
        param->wheel_pid[1][0] = 0.35f; param->wheel_pid[1][1] = 0.0f; param->wheel_pid[1][2] = 0.000f;
    }
}

/* 初始化 PID */
void RL_Torque_State_Init(rl_torque_state_t *state,
                          const rl_torque_param_t *param)
{
    if (state == NULL || param == NULL)
    {
        return;
    }
    memset(state, 0, sizeof(*state));
    PID_struct_init(&state->controller[VJ_L_THIGH], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_L_THIGH], 0.0f, param->d_gains[VJ_L_THIGH], 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_L_SHANK], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_L_SHANK], 0.0f, param->d_gains[VJ_L_SHANK], 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_L_WHEEL], POSITION_PID, 1000.0f,
        0.0f, param->wheel_pid[0][0], param->wheel_pid[0][1], param->wheel_pid[0][2], 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_R_THIGH], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_R_THIGH], 0.0f, param->d_gains[VJ_R_THIGH], 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_R_SHANK], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_R_SHANK], 0.0f, param->d_gains[VJ_R_SHANK], 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_R_WHEEL], POSITION_PID, 1000.0f,
        0.0f, param->wheel_pid[1][0], param->wheel_pid[1][1], param->wheel_pid[1][2], 0.0f, 0.0f);
    /* 关节 PID 开启角度环绕 */
    state->controller[VJ_L_THIGH].angle_wrap = 1u;
    state->controller[VJ_L_SHANK].angle_wrap = 1u;
    state->controller[VJ_R_THIGH].angle_wrap = 1u;
    state->controller[VJ_R_SHANK].angle_wrap = 1u;

    /* pos_target 初始化为静息位 */
    state->pos_target[VJ_L_THIGH] = param->dof_pos[VJ_L_THIGH];
    state->pos_target[VJ_L_SHANK] = param->dof_pos[VJ_L_SHANK];
    state->pos_target[VJ_R_THIGH] = param->dof_pos[VJ_R_THIGH];
    state->pos_target[VJ_R_SHANK] = param->dof_pos[VJ_R_SHANK];
}

/* 计算力矩 */
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          torque_output_t *torque)
{
    float act[VJ_NUM];
    float pos_ref[VJ_NUM];
    float vel_ref[VJ_NUM];
    float q[VJ_NUM];
    float qd[VJ_NUM];
    float tau_v[VJ_NUM];
    float tau_f[2];
    float tau_b[2];
    float leg_limit;
    float wheel_limit;

    torque->dm[DM_MOTOR_LEG_F_LFT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_B_LFT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_F_RGT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_B_RGT] = 0.0f;
    torque->dji[DJI_MOTOR_WHEEL_LFT] = 0.0f;
    torque->dji[DJI_MOTOR_WHEEL_RGT] = 0.0f;

    if (leg_l == NULL || leg_r == NULL || param == NULL || state == NULL
        || wheel_vel == NULL || action == NULL || torque == NULL)
    {
        return 0u;
    }
    if (!leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }
    if (!RL_Torque_Array_Finite(action, RL_ACTION_SIZE)
        || !RL_Torque_Array_Finite(wheel_vel, 2u))
    {
        return 0u;
    }

    /* 动作限幅 */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        act[i] = action[i];
    }

    /* 虚拟关节状态 */
    q[VJ_L_THIGH] = leg_l->output.thigh_angle;
    q[VJ_L_SHANK] = leg_l->output.virtual_shank_angle;
    q[VJ_L_WHEEL] = 0.0f;
    q[VJ_R_THIGH] = leg_r->output.thigh_angle;
    q[VJ_R_SHANK] = leg_r->output.virtual_shank_angle;
    q[VJ_R_WHEEL] = 0.0f;
    qd[VJ_L_THIGH] = leg_l->input.d_hip_f;
    qd[VJ_L_SHANK] = leg_l->output.d_virtual_shank_angle;
    qd[VJ_L_WHEEL] = wheel_vel[0];
    qd[VJ_R_THIGH] = leg_r->input.d_hip_f;
    qd[VJ_R_SHANK] = leg_r->output.d_virtual_shank_angle;
    qd[VJ_R_WHEEL] = wheel_vel[1];

    /* 位置/速度目标 */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        pos_ref[i] = 0.0f;
        vel_ref[i] = 0.0f;
    }
    pos_ref[VJ_L_THIGH] = act[VJ_L_THIGH] * RL_TQ_POS_SCALE;
    pos_ref[VJ_L_SHANK] = act[VJ_L_SHANK] * RL_TQ_POS_SCALE;
    pos_ref[VJ_R_THIGH] = act[VJ_R_THIGH] * RL_TQ_POS_SCALE;
    pos_ref[VJ_R_SHANK] = act[VJ_R_SHANK] * RL_TQ_POS_SCALE;
    vel_ref[VJ_L_WHEEL] = act[VJ_L_WHEEL] * RL_TQ_WHEEL_VEL_SCALE;
    vel_ref[VJ_R_WHEEL] = act[VJ_R_WHEEL] * RL_TQ_WHEEL_VEL_SCALE;

    /* 虚拟关节 PD */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        if (i == VJ_L_WHEEL || i == VJ_R_WHEEL)
        {
            /* 轮速目标限幅到物理满速 */
            vel_ref[i] = clampf(vel_ref[i], -RL_TQ_WHEEL_VEL_MAX,
                                RL_TQ_WHEEL_VEL_MAX);
            state->pos_target[i] = vel_ref[i];
            tau_v[i] = pid_calc(&state->controller[i], qd[i], vel_ref[i],
                0.002f);
        }
        else
        {
            float target = pos_ref[i] + param->dof_pos[i];
            state->pos_target[i] = target;
            tau_v[i] = pid_calc(&state->controller[i], q[i], target,
                0.002f);
        }
    }
    memcpy(state->virtual_torque, tau_v, sizeof(state->virtual_torque));

    /* 虚拟力矩映射: vshank_jac[0]→后髋, [1]→前髋 */
    tau_f[0] = tau_v[VJ_L_THIGH] + tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[1];
    tau_b[0] = tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[0];
    tau_f[1] = tau_v[VJ_R_THIGH] + tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[1];
    tau_b[1] = tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[0];

    /* DM 输出 (满限幅) */
    leg_limit = machine->dm_trq_clamp;
    torque->dm[DM_MOTOR_LEG_F_LFT] = clampf(tau_f[0], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_LFT] = clampf(tau_b[0], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_F_RGT] = clampf(tau_f[1], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_RGT] = clampf(tau_b[1], -leg_limit, leg_limit);

    /* DJI 输出 (满限幅) */
    wheel_limit = machine->dji_trq_clamp;
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(tau_v[VJ_L_WHEEL], -wheel_limit, wheel_limit);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(tau_v[VJ_R_WHEEL], -wheel_limit, wheel_limit);

    memcpy(&state->last_torque, torque, sizeof(state->last_torque));
    return 1u;
}
