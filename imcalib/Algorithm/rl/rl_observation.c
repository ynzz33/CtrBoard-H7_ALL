/*
 * 将执行任务采样的物理反馈转成模型坐标、投影重力并组装当前网络观测。
 * 同时维护RL倾倒失败计时；last_action保存裁剪后网络动作，不保存最终力矩。
 */
#include "rl_observation.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>
/* 8000 v2：行走模型仅保留倾倒终止，不实现趴地恢复。 */
#define RL_RECOVERY_TICK_MS ((uint16_t)(MACHINE_TICK_DT * 1000.0f + 0.5f))
void RL_Recovery_Reset(rl_recovery_state_t *state)
{
    memset(state, 0, sizeof(*state));
}
/* 仿真gravity_z>-0.1连续超过100个策略步终止；硬件以基础拍累计同一1010ms期限。 */
void RL_Recovery_Update(rl_recovery_state_t *state, uint8_t request, uint8_t feedback_valid, float gravity_z)
{
    if (!request) { RL_Recovery_Reset(state); return; }
    if (state->phase == RL_RECOVERY_FAILED) { return; }
    if (!feedback_valid || !isfinite(gravity_z)) { RL_Recovery_Reset(state); return; }
    state->phase = RL_RECOVERY_NORMAL;
    state->fallen_ms = gravity_z > machine->rl.recovery.terminal_gravity_z ? state->fallen_ms + RL_RECOVERY_TICK_MS : 0u;
    if (state->fallen_ms >= machine->rl.recovery.terminal_ms) { state->phase = RL_RECOVERY_FAILED; }
}
/* 公共拨轮仍表示腿长；RL入口先换算直立机高，再裁至8000训练范围。 */
float RL_Command_Height(float leg_length_command)
{
    return clampf(leg_length_command + MACHINE_RL_LEG_TO_HEIGHT, 0.22f, 0.30f);
}
/* 固件→模型：位置去额外零偏后乘sign，速度只乘sign；输出力矩在rl_torque中配对还原。 */
uint8_t RL_Motor_Map(const machine_rl_cfg_t *config, float position_period, const float position[4], const float velocity[4], const float wheel_velocity[2], uint8_t source_valid, rl_motor_map_state_t *state, rl_motor_state_t *out)
{
    out->valid = 0u;
    if (!source_valid) { return 0u; }
    for (uint8_t i = 0u; i < RL_LEG_MOTOR_NUM; i++)
    {
        out->pos[i] = (float)config->motor_sign[i] * (position[i] - config->motor_zero[i]);
        if (!state->initialized)
        {
            state->position_offset[i] = position_period * floorf((out->pos[i] - config->position_limit_center[i]) / position_period + 0.5f);
        }
        out->pos[i] -= state->position_offset[i];
        out->vel[i] = (float)config->motor_sign[i] * velocity[i];
    }
    for (uint8_t i = 0u; i < 2u; i++)
    {
        out->vel[RL_MOTOR_L_WHEEL + i] = (float)config->motor_sign[RL_MOTOR_L_WHEEL + i] * wheel_velocity[i];
    }
    out->valid = (uint8_t)(Float_Array_Finite(out->pos, RL_LEG_MOTOR_NUM) && Float_Array_Finite(out->vel, RL_ACTION_SIZE));
    if (out->valid) { state->initialized = 1u; }
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
/* 8000 v2：先缩放拼帧，历史保存未裁剪帧，当前输入最后整体裁剪。
 * 首帧重复填5次；任务先执行一个零动作周期，下一帧FIFO形成4份复位帧+1份新帧。
 * 不在硬件添加仿真噪声，也不把后续帧覆写进旧历史。 */
uint8_t RL_Observation_Build(rl_observation_state_t *state, const machine_rl_cfg_t *config, const rl_observation_sample_t *sample, const rc_command_t *command, const rl_recovery_state_t *recovery, uint8_t source_valid)
{
    float raw[RL_OBS_SIZE] = {0.0f};
    state->valid = 0u;
    memset(state->obs, 0, sizeof(state->obs));
    if (!source_valid || recovery->phase == RL_RECOVERY_FAILED) { return 0u; }
    for (uint8_t i = 0u; i < 3u; i++)
    {
        raw[RL_OBS_GYRO_X + i] = sample->gyro[i] * config->observation.gyro_scale;
        raw[RL_OBS_GRAV_X + i] = sample->gravity[i];
    }
    raw[RL_OBS_CMD_VX] = clip(command->velocity_target, 2.5f) * config->observation.velocity_command_scale;
    raw[RL_OBS_CMD_YAW_RATE] = clip(command->yaw_rate_target, 2.0f) * config->observation.gyro_scale;
    raw[RL_OBS_CMD_HEIGHT] = RL_Command_Height(command->leg_length_target) * config->observation.height_scale;
    for (uint8_t i = 0u; i < RL_LEG_MOTOR_NUM; i++)
    {
        raw[RL_OBS_MOTOR_POS + i] = sample->pos[i] - config->nominal_pos[i];
    }
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        raw[RL_OBS_MOTOR_VEL + RL_Action_Index(i)] = sample->vel[i] * config->observation.joint_vel_scale;
        raw[RL_OBS_LAST_ACTION + i] = state->last_action[i];
    }
    if (!Float_Array_Finite(raw, RL_OBS_SIZE)) { return 0u; }
    if (!state->history_initialized)
    {
        for (uint8_t i = 0u; i < RL_HISTORY_FRAMES; i++) { memcpy(&state->history[i * RL_OBS_SIZE], raw, sizeof(raw)); }
        state->history_initialized = 1u;
    }
    else
    {
        memmove(state->history, &state->history[RL_OBS_SIZE], (RL_HISTORY_SIZE - RL_OBS_SIZE) * sizeof(float));
        memcpy(&state->history[RL_HISTORY_SIZE - RL_OBS_SIZE], raw, sizeof(raw));
    }
    for (uint8_t i = 0u; i < RL_OBS_SIZE; i++) { state->obs[i] = clip(raw[i], config->observation.obs_clip); }
    state->valid = Float_Array_Finite(state->history, RL_HISTORY_SIZE);
    return state->valid;
}
/* 与环境process_actions一致：last_action保存裁剪后的动作，保持网络顺序。 */
void RL_Observation_Set_Last_Action(rl_observation_state_t *state, const float action[RL_ACTION_SIZE])
{
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++) { state->last_action[i] = clip(action[i], RL_ACTION_CLIP); }
}
