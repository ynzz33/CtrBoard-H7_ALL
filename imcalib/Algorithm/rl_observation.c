/*
 * 文件职责：训练侧观测构建和历史队列（rl_observation.c）。
 * 输入/来源：机体角速度/四元数、训练角色关节状态、遥控指令和上次动作。
 * 输出/去向：构建25维观测、5帧125维历史；完成实体角色与训练角色互逆转换。
 * 关键边界：缩放、裁剪、默认角和历史顺序需与训练一致；这里不推理、不选策略、不输出电机。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "math_utils.h"
#include "rl_observation.h"

#include <math.h>
#include <string.h>

/* 左右交换，腿反号；同一变换也是逆变换 */
static void RL_Observation_Transform_Roles(const float input[RL_ACTION_SIZE],
                                           float output[RL_ACTION_SIZE])
{
    static const uint8_t source[RL_ACTION_SIZE] = {3u, 4u, 5u, 0u, 1u, 2u};
    static const float sign[RL_ACTION_SIZE] = {-1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f};
    float snapshot[RL_ACTION_SIZE];

    memcpy(snapshot, input, sizeof(snapshot));
    for (uint32_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        output[i] = sign[i] * snapshot[source[i]];
    }
}

/* 实体参考量 → 训练角色 */
void RL_Observation_Roles_To_Training(const float reference[RL_ACTION_SIZE],
                                      float training[RL_ACTION_SIZE])
{
    RL_Observation_Transform_Roles(reference, training);
}

/* 训练角色 → 实体参考量 */
void RL_Observation_Roles_From_Training(const float training[RL_ACTION_SIZE],
                                        float reference[RL_ACTION_SIZE])
{
    RL_Observation_Transform_Roles(training, reference);
}

/* 初始化观测 */
void RL_Observation_Init(rl_observation_state_t *state)
{
    RL_Observation_Reset(state);
}

/* 从本机RL配置加载训练缩放、默认角和裁剪，运行状态另行初始化。 */
void RL_Observation_Param_Init(rl_observation_param_t *param,
                               const machine_rl_cfg_t *config)
{
    memset(param, 0, sizeof(*param));
    memcpy(param->obs_dof_pos, config->default_pos, sizeof(param->obs_dof_pos));
    memcpy(param->command_scale, config->observation.command_scale, sizeof(param->command_scale));
    memcpy(param->gyro_scale, config->observation.gyro_scale, sizeof(param->gyro_scale));
    memcpy(param->joint_vel_scale, config->observation.joint_vel_scale, sizeof(param->joint_vel_scale));
    param->obs_clip = config->observation.obs_clip;
    param->configured = config->configured;
}

/* 清空观测 */
void RL_Observation_Reset(rl_observation_state_t *state)
{
    memset(state, 0, sizeof(*state));
}

/* 计算重力 */
static void RL_Observation_Project_Gravity_Unit(const float quat[4], float gravity[3])
{
    float qx, qy, qz;  /* 四元虚部 */
    float qw;          /* 四元实部 */
    float tx, ty, tz;  /* 首次叉乘 */
    float cx, cy, cz;  /* 二次叉乘 */

    /* 对齐策略 */
    qx = -quat[1];
    qy = -quat[2];
    qz = -quat[3];
    qw = quat[0];

    /* 旋转重力 */
    tx = -2.0f * qy;
    ty =  2.0f * qx;
    tz =  0.0f;
    cx = qy * tz - qz * ty;
    cy = qz * tx - qx * tz;
    cz = qx * ty - qy * tx;

    gravity[0] = qw * tx + cx;
    gravity[1] = qw * ty + cy;
    gravity[2] = -1.0f + qw * tz + cz;
}

/* 投影重力接口 */
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3])
{
    float norm;
    float quat_unit[4];

    norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    for (uint32_t i = 0u; i < 4u; i++)
    {
        quat_unit[i] = quat[i] / norm;
    }
    RL_Observation_Project_Gravity_Unit(quat_unit, gravity);
    return Float_Array_Finite(gravity, 3u);
}

/* 构建观测 */
/*
 * 按训练契约把角速度、投影重力、指令、关节角/速和上次动作组织为25维观测。
 * 原训练缩放/角色顺序/裁剪保持；输入由反馈/映射端验证，source_valid为可用标志。
 */
uint8_t RL_Observation_Build(rl_observation_state_t *state,
                             const rl_observation_param_t *param,
                             const float gyro_rad_s[3],
                             const float quat[4],
                             const float command[3],
                             const float joint_pos[4],
                             const float joint_vel[6],
                             uint8_t source_valid)
{
    float gravity[3];    /* 投影重力 */

    if (!source_valid || !param->configured)
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    if (!RL_Observation_Project_Gravity(quat, gravity))
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    /* imu_state 已由 machine.imu 完成轴映射与极性校正；策略按训练 XYZ 直接读取。 */
    for (uint32_t i = 0u; i < 3u; i++)
    {
        state->obs[RL_OBS_GYRO_X + i] = gyro_rad_s[i] * param->gyro_scale[i];
    }
    for (uint32_t i = 0u; i < 3u; i++)
    {
        state->obs[RL_OBS_GRAV_X + i] = gravity[i];
    }
    for (uint32_t i = 0u; i < 3u; i++)
    {
        state->obs[RL_OBS_CMD_VX + i] = command[i] * param->command_scale[i];
    }

    state->obs[RL_OBS_L_THIGH] = joint_pos[0] - param->obs_dof_pos[0];
    state->obs[RL_OBS_L_SHANK] = joint_pos[1] - param->obs_dof_pos[1];
    state->obs[RL_OBS_R_THIGH] = joint_pos[2] - param->obs_dof_pos[2];
    state->obs[RL_OBS_R_SHANK] = joint_pos[3] - param->obs_dof_pos[3];

    for (uint32_t i = 0u; i < 6u; i++)
    {
        state->obs[RL_OBS_L_THIGH_VEL + i] = joint_vel[i] * param->joint_vel_scale[i];
    }
    for (uint32_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        state->obs[RL_OBS_LAST_ACTION + i] = state->last_action[i];
    }

    /* 整体裁剪 */
    for (uint32_t i = 0u; i < RL_OBS_SIZE; i++)
    {
        if (state->obs[i] > param->obs_clip)
        {
            state->obs[i] = param->obs_clip;
        }
        else if (state->obs[i] < -param->obs_clip)
        {
            state->obs[i] = -param->obs_clip;
        }
    }

    if (!Float_Array_Finite(state->obs, RL_OBS_SIZE))
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    state->valid = 1u;
    return 1u;
}

/* 更新历史 */
/*
 * 按训练顺序移动历史并追加当前帧；首帧按既有规则填满5帧。
 * history_ready与当前obs有效性共同决定推理输入是否可用。
 */
void RL_Observation_Update_History(rl_observation_state_t *state)
{
    if (!state->valid)
    {
        RL_Observation_Reset(state);
        return;
    }

    if (!state->history_ready)
    {
        for (uint32_t i = 0u; i < RL_OBS_HISTORY_FRAMES; i++)
        {
            memcpy(&state->history[i * RL_OBS_SIZE], state->obs, RL_OBS_SIZE * sizeof(float));
        }
        state->history_ready = 1u;
        return;
    }

    memmove(&state->history[0], &state->history[RL_OBS_SIZE], (RL_OBS_HISTORY_SIZE - RL_OBS_SIZE) * sizeof(float));
    memcpy(&state->history[RL_OBS_HISTORY_SIZE - RL_OBS_SIZE], state->obs, RL_OBS_SIZE * sizeof(float));
}

/* 保存动作 */
void RL_Observation_Set_Last_Action(rl_observation_state_t *state,
                                    const float action[RL_ACTION_SIZE])
{
    memcpy(state->last_action, action, RL_ACTION_SIZE * sizeof(float));
}
