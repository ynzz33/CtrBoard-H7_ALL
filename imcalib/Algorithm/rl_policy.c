#include "rl_policy.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "jump.h"
#include "jump_data.h"
#include "pin.h"
#include "pin_data.h"
#include "stable.h"
#include "stable_data.h"
#include "upstairs.h"
#include "upstairs_data.h"

#if (AI_STABLE_IN_1_SIZE != RL_OBS_SIZE) || (AI_STABLE_IN_2_SIZE != RL_OBS_HISTORY_SIZE) || (AI_STABLE_OUT_1_SIZE != RL_ACTION_SIZE)
#error "stable dimension mismatch"
#endif
#if (AI_UPSTAIRS_IN_1_SIZE != RL_OBS_SIZE) || (AI_UPSTAIRS_IN_2_SIZE != RL_OBS_HISTORY_SIZE) || (AI_UPSTAIRS_OUT_1_SIZE != RL_ACTION_SIZE)
#error "upstairs dimension mismatch"
#endif
#if (AI_PIN_IN_1_SIZE != RL_OBS_SIZE) || (AI_PIN_IN_2_SIZE != RL_OBS_HISTORY_SIZE) || (AI_PIN_OUT_1_SIZE != RL_ACTION_SIZE)
#error "pin dimension mismatch"
#endif
#if (AI_JUMP_IN_1_SIZE != RL_OBS_SIZE) || (AI_JUMP_IN_2_SIZE != RL_OBS_HISTORY_SIZE) || (AI_JUMP_OUT_1_SIZE != RL_ACTION_SIZE)
#error "jump dimension mismatch"
#endif

typedef struct {
    ai_handle network;   /* 网络句柄 */
    ai_buffer *inputs;   /* 输入缓存 */
    ai_buffer *outputs;  /* 输出缓存 */
    uint8_t ready;       /* 网络有效 */
} rl_network_t;

static rl_network_t rl_network[RL_MODEL_COUNT];

AI_ALIGNED(4) static ai_u8 stable_activations[AI_STABLE_DATA_ACTIVATION_1_SIZE];
AI_ALIGNED(4) static ai_u8 upstairs_activations[AI_UPSTAIRS_DATA_ACTIVATION_1_SIZE];
AI_ALIGNED(4) static ai_u8 pin_activations[AI_PIN_DATA_ACTIVATION_1_SIZE];
AI_ALIGNED(4) static ai_u8 jump_activations[AI_JUMP_DATA_ACTIVATION_1_SIZE];

/* 清空动作 */
static void RL_Policy_Clear_Action(float action[RL_ACTION_SIZE])
{
    if (action != NULL)
        memset(action, 0, RL_ACTION_SIZE * sizeof(float));
}

/* 检查动作 */
static uint8_t RL_Policy_Array_Valid(const float *data, uint32_t count)
{
    uint32_t i;

    if (data == NULL) return 0u;
    for (i = 0u; i < count; i++)
    {
        if (!isfinite(data[i])) return 0u;
    }
    return 1u;
}

/* 检查动作 */
static uint8_t RL_Policy_Action_Valid(const float action[RL_ACTION_SIZE])
{
    return RL_Policy_Array_Valid(action, RL_ACTION_SIZE);
}

/* 初始化网络 */
static uint8_t RL_Policy_Init_Network(rl_model_t model)
{
    rl_network_t *network;
    ai_error error;
    ai_u16 input_count;
    ai_u16 output_count;

    if (model >= RL_MODEL_COUNT) return 0u;
    network = &rl_network[model];
    if (network->ready) return 1u;

    input_count = 0u;
    output_count = 0u;
    if (model == RL_MODEL_STABLE)
    {
        const ai_handle activations[] = {AI_HANDLE_PTR(stable_activations)};
        error = ai_stable_create_and_init(&network->network, activations, NULL);
        if (error.type == AI_ERROR_NONE)
        {
            network->inputs = ai_stable_inputs_get(network->network, &input_count);
            network->outputs = ai_stable_outputs_get(network->network, &output_count);
            network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL
                && input_count == AI_STABLE_IN_NUM && output_count == AI_STABLE_OUT_NUM);
        }
    }
    else if (model == RL_MODEL_UPSTAIRS)
    {
        const ai_handle activations[] = {AI_HANDLE_PTR(upstairs_activations)};
        error = ai_upstairs_create_and_init(&network->network, activations, NULL);
        if (error.type == AI_ERROR_NONE)
        {
            network->inputs = ai_upstairs_inputs_get(network->network, &input_count);
            network->outputs = ai_upstairs_outputs_get(network->network, &output_count);
            network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL
                && input_count == AI_UPSTAIRS_IN_NUM && output_count == AI_UPSTAIRS_OUT_NUM);
        }
    }
    else if (model == RL_MODEL_PIN)
    {
        const ai_handle activations[] = {AI_HANDLE_PTR(pin_activations)};
        error = ai_pin_create_and_init(&network->network, activations, NULL);
        if (error.type == AI_ERROR_NONE)
        {
            network->inputs = ai_pin_inputs_get(network->network, &input_count);
            network->outputs = ai_pin_outputs_get(network->network, &output_count);
            network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL
                && input_count == AI_PIN_IN_NUM && output_count == AI_PIN_OUT_NUM);
        }
    }
    else
    {
        const ai_handle activations[] = {AI_HANDLE_PTR(jump_activations)};
        error = ai_jump_create_and_init(&network->network, activations, NULL);
        if (error.type == AI_ERROR_NONE)
        {
            network->inputs = ai_jump_inputs_get(network->network, &input_count);
            network->outputs = ai_jump_outputs_get(network->network, &output_count);
            network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL
                && input_count == AI_JUMP_IN_NUM && output_count == AI_JUMP_OUT_NUM);
        }
    }

    if (!network->ready)
    {
        network->network = AI_HANDLE_NULL;
        network->inputs = NULL;
        network->outputs = NULL;
    }
    return network->ready;
}

/* 运行网络 */
static ai_i32 RL_Policy_Run_Network(rl_model_t model, rl_network_t *network)
{
    if (model == RL_MODEL_STABLE)
        return ai_stable_run(network->network, network->inputs, network->outputs);
    if (model == RL_MODEL_UPSTAIRS)
        return ai_upstairs_run(network->network, network->inputs, network->outputs);
    if (model == RL_MODEL_PIN)
        return ai_pin_run(network->network, network->inputs, network->outputs);
    return ai_jump_run(network->network, network->inputs, network->outputs);
}

/* 读取错误 */
static void RL_Policy_Read_Error(rl_model_t model, const rl_network_t *network)
{
    if (model == RL_MODEL_STABLE)
        (void)ai_stable_get_error(network->network);
    else if (model == RL_MODEL_UPSTAIRS)
        (void)ai_upstairs_get_error(network->network);
    else if (model == RL_MODEL_PIN)
        (void)ai_pin_get_error(network->network);
    else
        (void)ai_jump_get_error(network->network);
}

/* 重置策略 */
void RL_Policy_Reset(rl_policy_t *policy)
{
    if (policy == NULL) return;
    memset(policy, 0, sizeof(*policy));
    policy->selected_model = RL_MODEL_STABLE;
}

/* 初始化策略 */
uint8_t RL_Policy_Init(rl_policy_t *policy)
{
    uint32_t i;

    if (policy == NULL) return 0u;
    __HAL_RCC_CRC_CLK_ENABLE();

    for (i = 0u; i < RL_MODEL_COUNT; i++)
        policy->model_ready[i] = RL_Policy_Init_Network((rl_model_t)i);

    policy->ready = 1u;
    for (i = 0u; i < RL_MODEL_COUNT; i++)
    {
        if (!policy->model_ready[i]) policy->ready = 0u;
    }
    return policy->ready;
}

/* 选择模型 */
uint8_t RL_Policy_Select(rl_policy_t *policy, rl_model_t model)
{
    if (policy == NULL || model >= RL_MODEL_COUNT) return 0u;
    policy->selected_model = model;
    return 1u;
}

/* 执行推理 */
uint8_t RL_Policy_Run(rl_policy_t *policy,
                      const rl_observation_state_t *observation,
                      float action[RL_ACTION_SIZE])
{
    rl_network_t *network;
    ai_float *obs_input;
    ai_float *history_input;
    ai_float *action_output;
    ai_i32 batches;

    RL_Policy_Clear_Action(action);
    if (policy == NULL || observation == NULL || !policy->ready
        || !observation->valid || !observation->history_ready
        || policy->selected_model >= RL_MODEL_COUNT
        || !RL_Policy_Array_Valid(observation->obs, RL_OBS_SIZE)
        || !RL_Policy_Array_Valid(observation->history, RL_OBS_HISTORY_SIZE))
        return 0u;

    network = &rl_network[policy->selected_model];
    if (!network->ready) return 0u;

    obs_input = AI_BUFFER_DATA(&network->inputs[0], ai_float);
    history_input = AI_BUFFER_DATA(&network->inputs[1], ai_float);
    action_output = AI_BUFFER_DATA(&network->outputs[0], ai_float);
    if (obs_input == NULL || history_input == NULL || action_output == NULL)
        return 0u;

    memcpy(obs_input, observation->obs, RL_OBS_SIZE * sizeof(float));
    memcpy(history_input, observation->history, RL_OBS_HISTORY_SIZE * sizeof(float));
    batches = RL_Policy_Run_Network(policy->selected_model, network);
    if (batches != 1)
    {
        RL_Policy_Read_Error(policy->selected_model, network);
        return 0u;
    }

    memcpy(action, action_output, RL_ACTION_SIZE * sizeof(float));
    if (!RL_Policy_Action_Valid(action))
    {
        RL_Policy_Clear_Action(action);
        return 0u;
    }
    return 1u;
}
