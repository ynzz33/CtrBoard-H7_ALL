/*
 * 文件职责：RL动作到虚拟关节及实体电机力矩（rl_torque.c）。
 * 输入/来源：6维动作、双腿几何、实测关节/轮速度、机器RL映射与控制器参数。
 * 输出/去向：构造关节目标、计算控制力矩、映射髋电机并补弹簧/限幅，输出torque_output_t。
 * 关键边界：动作是训练角色量而非直接CAN值；轮组使用本路径自己的速度控制，不套用LQR输出。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "rl_torque.h"
#include "gas_spring.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

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

/* 从本机RL配置加载动作缩放与控制器参数。当前仅networkzn1，沿用单模型入口。
 * 固件默认角仍为zero + sign × 训练默认角；不改关节映射或速度阻尼含义。 */
void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model,
                          const machine_rl_cfg_t *config)
{
    (void)model;
    memset(param, 0, sizeof(*param));
    param->dof_pos[VJ_L_THIGH] = config->zero[0] + (float)config->sign[VJ_L_THIGH] * config->default_pos[0];
    param->dof_pos[VJ_L_SHANK] = config->zero[1] + (float)config->sign[VJ_L_SHANK] * config->default_pos[1];
    param->dof_pos[VJ_R_THIGH] = config->zero[2] + (float)config->sign[VJ_R_THIGH] * config->default_pos[2];
    param->dof_pos[VJ_R_SHANK] = config->zero[3] + (float)config->sign[VJ_R_SHANK] * config->default_pos[3];
    memcpy(param->p_gains, config->torque.p_gains, sizeof(param->p_gains));
    memcpy(param->d_gains, config->torque.d_gains, sizeof(param->d_gains));
    memcpy(param->wheel_pid, config->torque.wheel_pid, sizeof(param->wheel_pid));
    param->leg_pid_max_output = config->torque.leg_pid_max_output;
    param->pos_scale = config->torque.pos_scale;
    param->wheel_vel_scale = config->torque.wheel_vel_scale;
}

/* 初始化 PID */
void RL_Torque_State_Init(rl_torque_state_t *state,
                          const rl_torque_param_t *param)
{
    uint8_t side;
    uint8_t leg_joint;

    memset(state, 0, sizeof(*state));
    for (uint8_t joint = 0u; joint < VJ_NUM; joint++)
    {
        if (joint == VJ_L_WHEEL || joint == VJ_R_WHEEL)
        {
            side = joint == VJ_L_WHEEL ? 0u : 1u;
            PID_struct_init(&state->controller[joint], POSITION_PID, param->wheel_pid[side].max_output, param->wheel_pid[side].integral_limit, param->wheel_pid[side].kp, param->wheel_pid[side].ki, param->wheel_pid[side].kd, 0.0f, 0.0f);
        }
        else
        {
            /* 腿部仅P */
            leg_joint = joint < VJ_L_WHEEL ? joint : joint - 1u;
            PID_struct_init(&state->controller[joint], POSITION_PID, param->leg_pid_max_output, 0.0f, param->p_gains[leg_joint], 0.0f, 0.0f, 0.0f, 0.0f);
            state->controller[joint].angle_wrap = 1u;
            state->pos_target[joint] = param->dof_pos[joint];
        }
    }
}

/* 计算力矩 */
/*
 * 动作缩放成训练虚拟关节目标 → 关节控制 → 五连杆/髋映射 → 弹簧 → 实体限幅。
 * 腿目标P与速度阻尼按本路径参数组合；轮子是速度目标环，不能用LQR轮矩含义解释动作。
 */
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const gas_spring_result_t *spring,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          torque_output_t *torque)
{
    float pos_ref[VJ_NUM];
    float vel_ref[VJ_NUM];
    float q[VJ_NUM];
    float qd[VJ_NUM];
    float tau_v[VJ_NUM];
    float base_dm[4];
    float raw_dm[4];
    float leg_limit;
    float wheel_limit;
    uint32_t leg_joint;

    torque->dm[DM_MOTOR_LEG_F_LFT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_B_LFT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_F_RGT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_B_RGT] = 0.0f;
    torque->dji[DJI_MOTOR_WHEEL_LFT] = 0.0f;
    torque->dji[DJI_MOTOR_WHEEL_RGT] = 0.0f;

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
    pos_ref[VJ_L_THIGH] = action[VJ_L_THIGH] * param->pos_scale;
    pos_ref[VJ_L_SHANK] = action[VJ_L_SHANK] * param->pos_scale;
    pos_ref[VJ_R_THIGH] = action[VJ_R_THIGH] * param->pos_scale;
    pos_ref[VJ_R_SHANK] = action[VJ_R_SHANK] * param->pos_scale;
    vel_ref[VJ_L_WHEEL] = action[VJ_L_WHEEL] * param->wheel_vel_scale;
    vel_ref[VJ_R_WHEEL] = action[VJ_R_WHEEL] * param->wheel_vel_scale;

    /* 虚拟关节 PD: 腿 Kp·wrap(目标 − q) − Kd·q̇, 轮 Kp·(v_ref − v) */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        if (i == VJ_L_WHEEL || i == VJ_R_WHEEL)
        {
            state->pos_target[i] = vel_ref[i];
            tau_v[i] = pid_calc(&state->controller[i], qd[i], vel_ref[i], MACHINE_RL_CTRL_DT);
        }
        else
        {
            float target = pos_ref[i] + param->dof_pos[i];
            leg_joint = i < VJ_L_WHEEL ? i : i - 1u;
            state->pos_target[i] = target;
            tau_v[i] = pid_calc(&state->controller[i], q[i], target, MACHINE_RL_CTRL_DT) - param->d_gains[leg_joint] * qd[i];
        }
    }
    /* 机器力矩上限 */
    leg_limit = machine->dm_trq_clamp;
    wheel_limit = machine->dji_trq_clamp;
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        if (i == VJ_L_WHEEL || i == VJ_R_WHEEL)
        {
            tau_v[i] = clampf(tau_v[i], -wheel_limit, wheel_limit);
        }
        else
        {
            tau_v[i] = clampf(tau_v[i], -leg_limit, leg_limit);
        }
    }
    memcpy(state->virtual_torque, tau_v, sizeof(state->virtual_torque));

    /* 补偿前力矩 */
    base_dm[0] = tau_v[VJ_L_THIGH] + tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[1];
    base_dm[1] = tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[0];
    base_dm[2] = tau_v[VJ_R_THIGH] + tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[1];
    base_dm[3] = tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[0];

    /* 独立补偿叠加 */
    if (!Gas_Spring_Apply(spring, base_dm, raw_dm))
    {
        return 0u;
    }

    /* DM 输出 (满限幅) */
    torque->dm[DM_MOTOR_LEG_F_LFT] = clampf(raw_dm[0], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_LFT] = clampf(raw_dm[1], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_F_RGT] = clampf(raw_dm[2], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_RGT] = clampf(raw_dm[3], -leg_limit, leg_limit);

    /* DJI 输出 (满限幅) */
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(tau_v[VJ_L_WHEEL], -wheel_limit, wheel_limit);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(tau_v[VJ_R_WHEEL], -wheel_limit, wheel_limit);

    memcpy(&state->last_torque, torque, sizeof(state->last_torque));
    return 1u;
}
