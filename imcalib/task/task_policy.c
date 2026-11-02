/*
 * 每个策略节拍复制最新反馈、遥控、RL执行状态和上一原始动作，组装网络输入并推理。
 * 发布六路原始action前重查模式和reset_count，防止旧控制段的推理覆盖新动作。
 */
#include "robot_control.h"
#include "robot_state.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include <string.h>
/* 初始化已编译CubeAI网络并检查观测/action尺寸；不在这里开启电机出力。 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}
/* 复制反馈和last_action→构建观测→推理→发布原始action。
 * reset_count前后不一致或模式已退出则丢弃结果；遥控选择RL可推理，出力另由执行层许可。 */
void ctrl_task_body(void)
{
    rl_observation_state_t observation;
    rl_observation_sample_t sample;
    rl_recovery_state_t recovery;
    rc_command_t command;
    float action[RL_ACTION_SIZE] = {0.0f};
    uint32_t reset_count; /* 本次推理所属动作段编号；发布前不一致说明结果已过期。 */
    uint8_t selected;
    uint8_t sampled; /* 是否成功复制执行任务的有效反馈样本。 */
    uint8_t built; /* 本次网络观测是否成功构建且全部有限。 */
    /* 样本缓存由高优先级执行任务推进，先在临界区复制，避免读取半更新帧。 */
    taskENTER_CRITICAL();
    selected = (uint8_t)(rc_command.online && rc_command.mode.strategy == CTRL_STRATEGY_RL);
    reset_count = action_state.reset_count;
    if (selected)
    {
        recovery = rl_control.recovery;
        command = rc_command;
        sampled = RL_Observation_Read_Sample(&rl_control.observation, &sample);
        memcpy(observation.last_action, rl_control.observation.last_action, sizeof(observation.last_action));
    }
    else
    {
        sampled = 0u;
        RL_Observation_Reset(&rl_control.observation);
    }
    taskEXIT_CRITICAL();
    if (!selected)
    {
        Action_State_Clear();
        return;
    }
    built = RL_Observation_Build(&observation, &machine->rl, &sample, &command, &recovery, sampled);
    taskENTER_CRITICAL();
    if (reset_count != action_state.reset_count)
    {
        taskEXIT_CRITICAL();
        return;
    }
    memcpy(rl_control.observation.obs, observation.obs, sizeof(observation.obs));
    rl_control.observation.valid = built;
    taskEXIT_CRITICAL();
    if (!built || !RL_Policy_Run(&rl_control.policy, &observation, action))
    {
        taskENTER_CRITICAL();
        if (reset_count == action_state.reset_count)
        {
            memset(rl_control.observation.last_action, 0, sizeof(rl_control.observation.last_action));
            Action_State_Clear();
        }
        taskEXIT_CRITICAL();
        return;
    }
    /* 网络请求保持四髋两轮原始顺序，目标生成和力矩限幅在RL_Torque_Compute。
     * 推理期间可能退出RL，发布前重查模式，防止新动作覆盖退出时的清零。 */
    taskENTER_CRITICAL();
    if (reset_count == action_state.reset_count && rc_command.online && rc_command.mode.strategy == CTRL_STRATEGY_RL)
    {
        RL_Observation_Set_Last_Action(&rl_control.observation, action);
        memcpy(action_state.a, action, sizeof(action_state.a));
        action_state.last_ok_tick = HAL_GetTick();
        action_state.updated = 1u;
        action_state.rl_ready = 1u;
    }
    /* 不匹配时直接丢弃；不能用旧推理的失败路径清掉新控制段。 */
    taskEXIT_CRITICAL();
}
