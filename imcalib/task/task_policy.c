#include "robot_control.h"
#include "robot_state.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include "machine_config.h"
#include "pid.h"

#include <string.h>

/* 选中 RL 后预热，再运行 networkzn1 推理；观测无效时发布零动作。 */
static uint16_t warmup_cnt;     /* 预热计数 */
enum { RL_OBSERVATION_PREVIEW, RL_OBSERVATION_HISTORY };



/* 检查电机 */
static uint8_t RL_Motors_Online(const motor_state_t *motor)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor->dm.online[i])
        {
            return 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor->dji.online[i])
        {
            return 0u;
        }
    }
    return 1u;
}

/* 机器表校准后交换角色: 训练 LF 对应实体右腿，RF 对应实体左腿 */
uint8_t RL_Joint_Map(const robot_feedback_t *feedback, float joint_pos[4], float joint_vel[6])
{
    const rl_map_t *map = &machine->rl;
    float q[RL_ACTION_SIZE] = {0.0f};
    float qd[6];

    if (!map->configured)
    {
        return 0u;
    }
    q[0] = (float)map->sign[0] * Angle_Wrap_180(feedback->left.output.thigh_angle - map->zero[0]);
    q[1] = (float)map->sign[1] * Angle_Wrap_180(feedback->left.output.virtual_shank_angle - map->zero[1]);
    q[3] = (float)map->sign[3] * Angle_Wrap_180(feedback->right.output.thigh_angle - map->zero[2]);
    q[4] = (float)map->sign[4] * Angle_Wrap_180(feedback->right.output.virtual_shank_angle - map->zero[3]);
    RL_Observation_Roles_To_Training(q, q);
    joint_pos[0] = q[0];
    joint_pos[1] = q[1];
    joint_pos[2] = q[3];
    joint_pos[3] = q[4];
    qd[0] = feedback->left.input.d_hip_f;
    qd[1] = feedback->left.output.d_virtual_shank_angle;
    qd[2] = feedback->motor.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    qd[3] = feedback->right.input.d_hip_f;
    qd[4] = feedback->right.output.d_virtual_shank_angle;
    qd[5] = feedback->motor.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    for (uint8_t i = 0u; i < 6u; i++)
    {
        qd[i] *= (float)map->sign[i];
    }
    RL_Observation_Roles_To_Training(qd, joint_vel);
    return 1u;
}

/* 采集与构建 */
static uint8_t RL_Control_Update_Observation(const robot_feedback_t *feedback, const float command[3], uint8_t mode)
{
    const imu_state_t *imu = &feedback->imu;
    float joint_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float joint_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t source_valid;

    source_valid = (uint8_t)(imu->online && feedback->left.output.valid
        && feedback->right.output.valid && (mode == RL_OBSERVATION_PREVIEW || RL_Motors_Online(&feedback->motor))
        && RL_Joint_Map(feedback, joint_pos, joint_vel));
    if (mode == RL_OBSERVATION_PREVIEW)
    {
        RL_Observation_Reset(&rl_control.observation);
    }
    if (!RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu->gyro_rad_s, imu->quat, command,
        joint_pos, joint_vel, source_valid))
    {
        return 0u;
    }
    if (mode == RL_OBSERVATION_HISTORY)
    {
        RL_Observation_Update_History(&rl_control.observation);
        return rl_control.observation.history_ready;
    }
    return rl_control.observation.valid;
}

/* 发布动作 (固件关节空间) */
static void RL_Action_Publish(const float action[RL_ACTION_SIZE], uint8_t rl_ready)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(action_state.a, action, sizeof(action_state.a));
    action_state.updated = 1u;
    action_state.last_ok_tick = HAL_GetTick();
    action_state.rl_ready = rl_ready;
    __set_PRIMASK(primask);
}

/* 遥控 → 策略指令 */
static void RL_Command_From_Rc(const rc_command_t *rc, float command[3])
{
    float vx_max = RL_CMD_VX_MAX;
    float yaw_max = RL_CMD_YAW_MAX;
    float height_min = RL_CMD_HEIGHT_MIN;
    float height_max = RL_CMD_HEIGHT_MAX;
    float height = input_command.height_cmd;
    float policy_dt = MACHINE_POLICY_DT;

    if (height < height_min || height > height_max)
    {
        height = RL_CMD_HEIGHT_INIT;
    }
    if (output_task_rl_engaged() && rc->online)
    {
        height += rc->len * RL_CMD_HEIGHT_RATE * policy_dt;
        height = clampf(height, height_min, height_max);
    }
    else
    {
        height = RL_CMD_HEIGHT_INIT;
    }

    /* 保留训练范围 */
    command[0] = machine->lqr.vel_max > 0.0f
        ? rc->vel / machine->lqr.vel_max * vx_max : 0.0f;
    command[1] = machine->lqr.yaw_max > 0.0f
        ? -rc->yaw / machine->lqr.yaw_max * yaw_max : 0.0f;
    command[2] = height;
    input_command.vx_cmd = command[0];
    input_command.yaw_cmd = command[1];
    input_command.height_cmd = command[2];
}

/* 动作裁剪 (RL_ACTION_CLIP = 0 不裁) */
static void RL_Action_Clip(float action[RL_ACTION_SIZE])
{
    float limit = RL_ACTION_CLIP;

    if (limit <= 0.0f)
    {
        return;
    }
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = clampf(action[i], -limit, limit);
    }
}

/* 推理路径单周期 */
static void RL_Infer_Body(const robot_feedback_t *feedback)
{
    const rl_map_t *map = &machine->rl;
    float command[3];
    float action_t[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   /* 训练空间 */
    float action_ref[RL_ACTION_SIZE] = {0.0f};  /* 实体参考 */
    float action[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};     /* 固件空间 */
    uint8_t engaged;

    RL_Command_From_Rc(&feedback->rc, command);
    engaged = output_task_rl_engaged();

    /* 未投入: 清历史 (预览观测照算供 VOFA), 发零动作保持新鲜 (LQR 挡也走这里) */
    if (!engaged)
    {
        warmup_cnt = 0u;
        (void)RL_Control_Update_Observation(feedback, command, RL_OBSERVATION_PREVIEW);
        RL_Action_Publish(action, 0u);
        return;
    }

    if (!RL_Control_Update_Observation(feedback, command, RL_OBSERVATION_HISTORY))
    {
        /* 观测无效: 重新预热, 零力矩 */
        warmup_cnt = 0u;
        RL_Action_Publish(action, 0u);
        return;
    }

    if (warmup_cnt < RL_WARMUP_STEPS)
    {
        warmup_cnt++;
    }
    else
    {
        if (!RL_Policy_Run(&rl_control.policy, &rl_control.observation, action_t))
        {
            RL_Observation_Set_Last_Action(&rl_control.observation, action_t);   /* 失败: 记录零动作 */
            RL_Action_Publish(action, 0u);   /* 推理失败 */
            return;
        }
        RL_Action_Clip(action_t);
    }

    RL_Observation_Set_Last_Action(&rl_control.observation, action_t);
    RL_Observation_Roles_From_Training(action_t, action_ref);
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = (float)map->sign[i] * action_ref[i];   /* 参考 → 固件 */
    }
    RL_Action_Publish(action, 1u);
}

/* 策略初始化 */
void ctrl_task_init(void)
{
    if (machine->rl.configured)
    {
        (void)RL_Policy_Init(&rl_control.policy);
    }
}

/* 策略单周期 */
void ctrl_task_body(void)
{
    static robot_feedback_t feedback;

    if (machine->rl.configured)
    {
        (void)Robot_Feedback_Read(&feedback);
        RL_Infer_Body(&feedback);
    }
}
