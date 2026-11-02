/*
 * 将执行任务采样的物理反馈转成模型坐标、投影重力并组装当前网络观测。
 * 同时维护RL启动保持/连续异常失败计时；last_action保存网络原始动作，不保存最终力矩。
 */
#include "rl_observation.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>
/* 1999连续触地/倾倒20个50Hz步终止；作者实机试验指定代理机高≤0.13m。 */
#define RL_RECOVERY_TICK_MS ((uint16_t)(MACHINE_TICK_DT * 1000.0f + 0.5f))
void RL_Recovery_Reset(rl_recovery_state_t *state)
{
    memset(state, 0, sizeof(*state));
}
/* 每基础拍按固定dt累计RL启动保持和连续down时间；down由倾角或低高度构成。
 * 首次down进入HOLD，保持结束转NORMAL；连续异常达超时转FAILED，退出请求才解除失败。
 * 当前未设置RECOVER阶段；stable_ms仍累计，不能把枚举保留值当成已实现的恢复动作。 */
void RL_Recovery_Update(rl_recovery_state_t *state, uint8_t request, uint8_t feedback_valid, uint8_t fallen, float height)
{
    uint8_t down;
    if (!request) { RL_Recovery_Reset(state); return; }
    if (state->phase == RL_RECOVERY_FAILED) { return; }
    if (!feedback_valid) { RL_Recovery_Reset(state); return; }
    down = (uint8_t)(fallen || height <= machine->rl.recovery.recovery_height);
    if (state->phase == RL_RECOVERY_IDLE)
    {
        state->ground_start = down;
        state->phase = down ? RL_RECOVERY_HOLD : RL_RECOVERY_NORMAL;
        return; /* 投入当拍为t=0，不提前推进启动保持。 */
    }
    if (state->hold_ms < machine->rl.recovery.recovery_hold_ms) { state->hold_ms += RL_RECOVERY_TICK_MS; }
    if (down)
    {
        state->stable_ms = 0u;
        state->fallen_ms += RL_RECOVERY_TICK_MS;
        if (state->phase != RL_RECOVERY_HOLD) { state->phase = RL_RECOVERY_NORMAL; }
    }
    else
    {
        state->fallen_ms = 0u;
        if (state->stable_ms < machine->rl.recovery.recovery_stable_ms) { state->stable_ms += RL_RECOVERY_TICK_MS; }
        if (state->stable_ms >= machine->rl.recovery.recovery_stable_ms) { state->phase = RL_RECOVERY_NORMAL; }
    }
    if (state->phase == RL_RECOVERY_HOLD && state->hold_ms >= machine->rl.recovery.recovery_hold_ms) { state->phase = RL_RECOVERY_NORMAL; }
    if (state->fallen_ms >= machine->rl.recovery.recovery_timeout_ms) { state->phase = RL_RECOVERY_FAILED; }
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
/* 按RL_OBS索引填命令、角速度、投影重力、电机位置/速度及上次原始action。
 * 高度请求独立限幅/缩放，当前控制模式槽固定1；结果全部有限才置valid。 */
uint8_t RL_Observation_Build(rl_observation_state_t *state, const machine_rl_cfg_t *config, const rl_observation_sample_t *sample, const rc_command_t *command, const rl_recovery_state_t *recovery, uint8_t source_valid)
{
    memset(state->obs, 0, sizeof(state->obs));
    state->valid = 0u;
    if (!source_valid || recovery->phase == RL_RECOVERY_FAILED) { return 0u; }
    /* 作者选择直接复用rc.len作高度命令，接受低段拨轮被下限截住；不回写公共RC。 */
    state->obs[RL_OBS_CMD_VX] = clip(command->velocity_target, 2.7f);
    state->obs[RL_OBS_CMD_YAW_RATE] = clip(command->yaw_rate_target, 6.283185307f);
    state->obs[RL_OBS_CMD_HEIGHT] = clampf(command->leg_length_target, MACHINE_RL_HEIGHT_CMD_MIN, MACHINE_RL_HEIGHT_CMD_MAX) * config->observation.height_scale;
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
    state->obs[RL_OBS_CONTROL_MODE] = 1.0f; /* 1999动态模式关闭，始终普通模式。 */
    state->valid = Float_Array_Finite(state->obs, RL_OBS_SIZE);
    return state->valid;
}
/* 保存网络请求本身；不能用PD目标、延迟后的目标或最终力矩替代。 */
void RL_Observation_Set_Last_Action(rl_observation_state_t *state, const float action[RL_ACTION_SIZE])
{
    memcpy(state->last_action, action, sizeof(state->last_action));
}
