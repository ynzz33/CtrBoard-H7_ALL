/*
 * model1999单actor：输入float32[1,35]，输出action[1,6]，按四腿两轮排列。
 * 管理网络工作区、SDK调用和推理耗时；观测由rl_observation提供，电机控制归rl_torque。
 * 旧25/125生成模型仍能编译，但尺寸不符时拒绝初始化；新模型须成套替换生成文件。
 */
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
    if (AI_NETWORKZN1_IN_NUM != 1 || AI_NETWORKZN1_OUT_NUM != 1 || AI_NETWORKZN1_IN_1_SIZE != RL_OBS_SIZE || AI_NETWORKZN1_OUT_1_SIZE != RL_ACTION_SIZE) { return 0u; }
    network = &rl_network;
    if (network->ready) { return 1u; }
    error = ai_networkzn1_create_and_init(&network->network, activations, NULL);
    if (error.type == AI_ERROR_NONE)
    {
        network->inputs = ai_networkzn1_inputs_get(network->network, &input_count);
        network->outputs = ai_networkzn1_outputs_get(network->network, &output_count);
        network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL && input_count == 1u && output_count == 1u);
        /* SDK逻辑元素数可能与底层对齐容量不同，按实际35/6、batch1及float32检查。 */
        if (network->ready)
        {
            network->ready = (uint8_t)(AI_BUFFER_SIZE_UNPAD(&network->inputs[0]) == RL_OBS_SIZE && AI_BUFFER_SIZE_UNPAD(&network->outputs[0]) == RL_ACTION_SIZE && AI_BUFFER_SHAPE_ELEM(&network->inputs[0], AI_SHAPE_BATCH) == 1u && AI_BUFFER_SHAPE_ELEM(&network->outputs[0], AI_SHAPE_BATCH) == 1u && AI_BUFFER_FMT_GET_TYPE(network->inputs[0].format) == AI_BUFFER_FMT_TYPE_FLOAT && AI_BUFFER_FMT_GET_TYPE(network->outputs[0].format) == AI_BUFFER_FMT_TYPE_FLOAT && AI_BUFFER_FMT_GET_BITS(network->inputs[0].format) == 32u && AI_BUFFER_FMT_GET_BITS(network->outputs[0].format) == 32u);
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
        network->inputs[0].data = AI_HANDLE_PTR(standup_input);
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
/* 复制有效obs35→推理→读取原始action6；失败返回零动作，是否出力由任务层决定。 */
uint8_t RL_Policy_Run(rl_policy_t *policy, const rl_observation_state_t *observation, float action[RL_ACTION_SIZE])
{
    rl_network_t *network;
    ai_float *obs_input;
    ai_float *action_output;
    ai_i32 batches;
    uint64_t start_ns;
    memset(action, 0, RL_ACTION_SIZE * sizeof(float));
    if (!policy->ready || !observation->valid) { return 0u; }
    network = &rl_network;
    if (!network->ready) { return 0u; }
    obs_input = AI_BUFFER_DATA(&network->inputs[0], ai_float);
    action_output = AI_BUFFER_DATA(&network->outputs[0], ai_float);
    if (obs_input == NULL || action_output == NULL) { policy->run_fail++; return 0u; }
    memcpy(obs_input, observation->obs, RL_OBS_SIZE * sizeof(float));
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
    policy->run_ok++;
    return 1u;
}
