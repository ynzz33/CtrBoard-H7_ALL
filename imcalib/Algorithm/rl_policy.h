/* 单actor推理接口及SDK状态；网络仅输出请求动作，不包含PD、历史encoder或latent。 */
#ifndef RL_POLICY_H
#define RL_POLICY_H
#include "rl_observation.h"
#include "ai_platform.h"
typedef struct {
    uint8_t ready;          /* 网络接口匹配且初始化成功 */
    uint32_t run_ok;        /* 有效推理次数 */
    uint32_t run_fail;      /* SDK运行/缓冲或输出异常次数 */
    uint32_t run_us;        /* 最近一次网络调用耗时，us */
} rl_policy_t;
/* SDK工作对象由rl_policy.c私有持有，任务层只读取上面的策略状态。 */
typedef struct {
    ai_handle network;
    ai_buffer *inputs;
    ai_buffer *outputs;
    uint8_t ready;
} rl_network_t;
/* 初始化在策略任务启动时调用；Run保持网络原始左右两侧顺序，不重排或全局限幅。 */
void RL_Policy_Reset(rl_policy_t *policy);
uint8_t RL_Policy_Init(rl_policy_t *policy);
uint8_t RL_Policy_Run(rl_policy_t *policy, const rl_observation_state_t *observation, float action[RL_ACTION_SIZE]);
#endif
