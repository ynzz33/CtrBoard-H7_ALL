/*
 * model3023的RL39输入：将已标定电机反馈转换到模型原生坐标，再按训练顺序构建观测。
 * 位置/速度使用rad、rad/s；先裁剪再缩放。机器配置直接取固定表，不扫描配置合法性。
 * 执行任务保存最新反馈，策略任务复制后构建；不添加训练噪声或延迟，不调用网络或发送电机。
 */
#include "rl_observation.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>
/* 固件→模型：位置去额外零偏后乘sign，速度只乘sign；输出力矩在rl_torque中配对还原。 */
uint8_t RL_Motor_Map(const machine_rl_cfg_t *config, const float position[4], const float velocity[4], const float wheel_velocity[2], uint8_t source_valid, rl_motor_state_t *out)
{
    out->valid = 0u;
    if (!source_valid || !config->configured) { return 0u; }
    for (uint8_t i = 0u; i < RL_LEG_MOTOR_NUM; i++)
    {
        out->pos[i] = (float)config->motor_sign[i] * (position[i] - config->motor_zero[i]);
        out->vel[i] = (float)config->motor_sign[i] * velocity[i];
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        out->vel[RL_MOTOR_L_WHEEL + i] = (float)config->motor_sign[RL_MOTOR_L_WHEEL + i] * wheel_velocity[i];
    }
    out->valid = (uint8_t)(Float_Array_Finite(out->pos, RL_LEG_MOTOR_NUM) && Float_Array_Finite(out->vel, RL_ACTION_SIZE));
    return out->valid;
}
/* 四元数顺序w/x/y/z；归一化后将世界单位重力[0,0,-1]逆旋转到机体系。 */
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3])
{
    float norm, w, x, y, z;
    norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    w = quat[0] / norm;
    x = quat[1] / norm;
    y = quat[2] / norm;
    z = quat[3] / norm;
    gravity[0] = 2.0f * (w * y - x * z);
    gravity[1] = -2.0f * (w * x + y * z);
    gravity[2] = 2.0f * (x * x + y * y) - 1.0f;
    return Float_Array_Finite(gravity, 3u);
}
/* 退出RL或初始化时清观测、原始动作历史及最新样本。 */
void RL_Observation_Reset(rl_observation_state_t *state)
{
    memset(state, 0, sizeof(*state));
}
/* 每个RL控制子步保存未缩放反馈；源头online/valid保证原始量，新投影在这里检查。 */
void RL_Observation_Sample(rl_observation_state_t *state, const float gyro[3], const float quat[4], const rl_motor_state_t *motor, uint8_t source_valid)
{
    rl_observation_sample_t sample;
    if (!source_valid || !motor->valid || !RL_Observation_Project_Gravity(quat, sample.gravity))
    {
        state->sample_valid = 0u; /* 坏反馈不能继续借用上一帧。 */
        return;
    }
    memcpy(sample.gyro, gyro, sizeof(sample.gyro));
    memcpy(sample.pos, motor->pos, sizeof(sample.pos));
    memcpy(sample.vel, motor->vel, sizeof(sample.vel));
    state->sample = sample;
    state->sample_valid = 1u;
}
/* 调用方在短临界区复制最新有效样本，采样与推理之间仅保留实际任务相位。 */
uint8_t RL_Observation_Read_Sample(const rl_observation_state_t *state, rl_observation_sample_t *sample)
{
    if (!state->sample_valid) { return 0u; }
    *sample = state->sample;
    return 1u;
}
/* actor固定39维；首轮范围为NORMAL静站，不把SDK就绪/实时接地当成模型状态标志。 */
uint8_t RL_Observation_Build(rl_observation_state_t *state, const machine_rl_cfg_t *config, const rl_observation_sample_t *sample, uint8_t source_valid)
{
    memset(state->obs, 0, sizeof(state->obs));
    state->valid = 0u;
    if (!source_valid) { return 0u; }
    /* 0～3：三路速度指令保持0，高度取模型固定命令，与公共RC腿长分开。 */
    state->obs[RL_OBS_CMD_HEIGHT] = clampf(config->height, 0.0f, 1.0f) * config->observation.height_scale;
    /* 4～9：机体角速度与单位重力；角速度先限幅，再乘训练缩放。 */
    for (uint8_t i = 0u; i < 3u; i++)
    {
        state->obs[RL_OBS_GYRO_X + i] = clip(sample->gyro[i], config->observation.obs_clip) * config->observation.gyro_scale;
        state->obs[RL_OBS_GRAV_X + i] = clip(sample->gravity[i], config->observation.obs_clip);
    }
    /* 10～15：四实体电机角减名义角，末两路轮位置槽保持0。 */
    for (uint8_t i = 0u; i < RL_LEG_MOTOR_NUM; i++)
    {
        state->obs[RL_OBS_MOTOR_POS + i] = clip(sample->pos[i] - config->nominal_pos[i], config->observation.obs_clip);
    }
    /* 16～27：六路速度与上一原始请求；观测裁剪不改写保存的action。 */
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        state->obs[RL_OBS_MOTOR_VEL + i] = clip(sample->vel[i], config->observation.velocity_clip) * config->observation.joint_vel_scale;
        state->obs[RL_OBS_LAST_ACTION + i] = clip(state->last_action[RL_Action_Index(i)], config->observation.obs_clip);
    }
    state->obs[RL_OBS_CONTROL_MODE] = 1.0f; /* 28～34固定[1,0,0,0,0,0,0]，与拨杆状态无关。 */
    state->obs[RL_OBS_POLICY_ACTIVE] = 1.0f; /* 35～38固定[0,1,0,0]：普通初态、策略阶段激活。 */
    state->valid = Float_Array_Finite(state->obs, RL_OBS_SIZE);
    return state->valid;
}
/* 保存网络请求本身；不能用PD目标、延迟后的目标或最终力矩替代。 */
void RL_Observation_Set_Last_Action(rl_observation_state_t *state, const float action[RL_ACTION_SIZE])
{
    memcpy(state->last_action, action, sizeof(state->last_action));
}
