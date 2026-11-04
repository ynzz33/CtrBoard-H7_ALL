/*
 * 初始化CubeAI网络并校验输入/输出尺寸，观测有效时运行一次网络，返回环境裁剪后的action。
 * ready表示网络接口就绪，不表示遥控投入或允许电机出力；推理耗时单独记录。
 */
/* 8000 v2：obs[1,25]+obs_history[1,125]→actions[1,6]。
 * 图内含Encoder和Actor；生成文件仍由作者使用同名networkzn1替换。
 * 旧单输入网络可编译，但初始化必须拒绝，不能把旧权重用于新观测。 */
#include "rl_policy.h"
#include "math_utils.h"
#include "main.h"
#include "mono_ns.h"
#include "networkzn1.h"
#include "networkzn1_data.h"
#include <string.h>
static rl_network_t rl_network;
/* 工作区尺寸跟随生成头；独立I/O缓存避免依赖导出时的activation分配选项。 */
AI_ALIGNED(4) static ai_u8 standup_activations[AI_NETWORKZN1_DATA_ACTIVATION_1_SIZE];
AI_ALIGNED(4) static ai_float standup_input[RL_OBS_SIZE];
AI_ALIGNED(4) static ai_float history_input[RL_HISTORY_SIZE];
AI_ALIGNED(4) static ai_float standup_output[RL_ACTION_SIZE];
/* 只创建一个网络；生成头与SDK实际接口均匹配后才发布ready。 */
static uint8_t RL_Policy_Init_Network(void)
{
    rl_network_t *network;
    ai_error error;
    ai_u16 input_count = 0u;
    ai_u16 output_count = 0u;
    const ai_handle activations[] = {AI_HANDLE_PTR(standup_activations)};
    /* 维度来自CubeAI生成头，替换同名完整生成文件即可接入新actor。 */
    if (AI_NETWORKZN1_IN_NUM != 2 || AI_NETWORKZN1_OUT_NUM != 1 || AI_NETWORKZN1_OUT_1_SIZE != RL_ACTION_SIZE) { return 0u; }
    network = &rl_network;
    if (network->ready) { return 1u; }
    error = ai_networkzn1_create_and_init(&network->network, activations, NULL);
    if (error.type == AI_ERROR_NONE)
    {
        network->inputs = ai_networkzn1_inputs_get(network->network, &input_count);
        network->outputs = ai_networkzn1_outputs_get(network->network, &output_count);
        network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL && input_count == 2u && output_count == 1u);
        /* 两路输入均须是batch1/float32；按实际元素数绑定，不猜生成器排序。 */
        if (network->ready)
        {
            uint8_t matched = 0u;
            for (uint8_t i = 0u; i < 2u; i++)
            {
                ai_buffer *input = &network->inputs[i];
                uint32_t size = AI_BUFFER_SIZE_UNPAD(input);
                if (AI_BUFFER_SHAPE_ELEM(input, AI_SHAPE_BATCH) != 1u || AI_BUFFER_FMT_GET_TYPE(input->format) != AI_BUFFER_FMT_TYPE_FLOAT || AI_BUFFER_FMT_GET_BITS(input->format) != 32u) { break; }
                if (size == RL_OBS_SIZE) { network->obs_index = i; matched |= 1u; }
                if (size == RL_HISTORY_SIZE) { network->history_index = i; matched |= 2u; }
            }
            network->ready = (uint8_t)(matched == 3u && AI_BUFFER_SIZE_UNPAD(&network->outputs[0]) == RL_ACTION_SIZE && AI_BUFFER_SHAPE_ELEM(&network->outputs[0], AI_SHAPE_BATCH) == 1u && AI_BUFFER_FMT_GET_TYPE(network->outputs[0].format) == AI_BUFFER_FMT_TYPE_FLOAT && AI_BUFFER_FMT_GET_BITS(network->outputs[0].format) == 32u);
        }
    }
    /* 初始化失败释放句柄，不能让部分初始化结果被策略任务使用。 */
    if (!network->ready)
    {
        if (network->network != AI_HANDLE_NULL) { (void)ai_networkzn1_destroy(network->network); }
        network->network = AI_HANDLE_NULL;
        network->inputs = NULL;
        network->outputs = NULL;
    }
    else
    {
        /* 独立I/O缓冲，不依赖导出时是否将输入/输出分配到activation。 */
        network->inputs[network->obs_index].data = AI_HANDLE_PTR(standup_input);
        network->inputs[network->history_index].data = AI_HANDLE_PTR(history_input);
        network->outputs[0].data = AI_HANDLE_PTR(standup_output);
    }
    return network->ready;
}
/* 清策略状态及计数；网络句柄由本模块管理，不在每次投入时重复创建。 */
void RL_Policy_Reset(rl_policy_t *policy)
{
    memset(policy, 0, sizeof(*policy));
}
/* 固定机器配置直接使用；运行前仅初始化网络及其所需CRC时钟。 */
uint8_t RL_Policy_Init(rl_policy_t *policy)
{
    policy->ready = 0u;
    __HAL_RCC_CRC_CLK_ENABLE();
    policy->ready = RL_Policy_Init_Network();
    return policy->ready;
}
/* 复制当前帧和历史→推理→环境±100裁剪；不推进历史，失败不发布动作。 */
uint8_t RL_Policy_Run(rl_policy_t *policy, const rl_observation_state_t *observation, float action[RL_ACTION_SIZE])
{
    rl_network_t *network;
    ai_float *obs_input;
    ai_float *history;
    ai_float *action_output;
    ai_i32 batches;
    uint64_t start_ns;
    memset(action, 0, RL_ACTION_SIZE * sizeof(float));
    if (!policy->ready || !observation->valid) { return 0u; }
    network = &rl_network;
    if (!network->ready) { return 0u; }
    obs_input = AI_BUFFER_DATA(&network->inputs[network->obs_index], ai_float);
    history = AI_BUFFER_DATA(&network->inputs[network->history_index], ai_float);
    action_output = AI_BUFFER_DATA(&network->outputs[0], ai_float);
    if (obs_input == NULL || history == NULL || action_output == NULL) { policy->run_fail++; return 0u; }
    memcpy(obs_input, observation->obs, RL_OBS_SIZE * sizeof(float));
    memcpy(history, observation->history, RL_HISTORY_SIZE * sizeof(float));
    start_ns = Mono_Ns_Get(); /* run_us只统计网络调用，不包含观测构建或CAN下发。 */
    batches = ai_networkzn1_run(network->network, network->inputs, network->outputs);
    policy->run_us = (uint32_t)((Mono_Ns_Get() - start_ns) / 1000u);
    if (batches != 1)
    {
        (void)ai_networkzn1_get_error(network->network);
        policy->run_fail++;
        return 0u;
    }
    memcpy(action, action_output, RL_ACTION_SIZE * sizeof(float));
    if (!Float_Array_Finite(action, RL_ACTION_SIZE))
    {
        memset(action, 0, RL_ACTION_SIZE * sizeof(float));
        policy->run_fail++;
        return 0u;
    }
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++) { action[i] = clip(action[i], RL_ACTION_CLIP); }
    policy->run_ok++;
    return 1u;
}
