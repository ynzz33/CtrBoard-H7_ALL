/*
 * 50Hz策略：复制500Hz最新反馈及原始动作历史→组装obs39→推理→发布原始action6。
 * 同帧复制公共遥控与恢复标志；趴地启动保持由执行任务控制，不以SDK就绪替代模型激活。
 * 共享样本/动作只在短临界区读写，观测构建和网络调用均放在临界区之外。
 */
#include "robot_control.h"
#include "robot_state.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include <string.h>
/* 策略任务启动时初始化已编译网络；加载/接口检查由RL_Policy_Init完成。 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}
/* 本地构造后完整发布；清零期间仍在途的推理不能覆盖新控制段的动作。 */
void ctrl_task_body(void)
{
    rl_observation_state_t observation;
    rl_observation_sample_t sample;
    rl_recovery_state_t recovery;
    rc_command_t command;
    float action[RL_ACTION_SIZE] = {0.0f};
    uint32_t reset_count;
    uint8_t selected;
    uint8_t sampled;
    uint8_t built;
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
    /* 请求动作不反号、不全局裁剪；观测裁剪与目标裁剪由各自生成端完成。
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
