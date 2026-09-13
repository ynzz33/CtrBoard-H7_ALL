#ifndef RL_POLICY_H
#define RL_POLICY_H

#include <stdint.h>

#include "rl_observation.h"

typedef enum {
    RL_MODEL_STABLE = 0,
    RL_MODEL_UPSTAIRS,
    RL_MODEL_PIN,
    RL_MODEL_JUMP,
    RL_MODEL_COUNT
} rl_model_t;

typedef struct {
    rl_model_t selected_model;      /* 当前模型 */
    uint8_t model_ready[RL_MODEL_COUNT]; /* 模型状态 */
    uint8_t ready;                  /* 全部有效 */
} rl_policy_t;

void RL_Policy_Reset(rl_policy_t *policy);
uint8_t RL_Policy_Init(rl_policy_t *policy);
uint8_t RL_Policy_Select(rl_policy_t *policy, rl_model_t model);
uint8_t RL_Policy_Run(rl_policy_t *policy,
                      const rl_observation_state_t *observation,
                      float action[RL_ACTION_SIZE]);

#endif
