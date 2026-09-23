#ifndef RL_POLICY_H
#define RL_POLICY_H

#include <stdint.h>

#include "rl_observation.h"
#include "ai_platform.h"

#define REMOTE_COMMAND_SCALE 3.0f   /* 手动遥操: obs 指令缩放 (旧路径) */

/* 推理路径: 遥控 → 策略指令的训练侧范围 (待训练侧给; 0 = 该路恒零) */
#define RL_CMD_VX_MAX       0.0f    /* m/s */
#define RL_CMD_YAW_MAX      0.0f    /* rad/s */
#define RL_CMD_HEIGHT_MIN   0.0f    /* m */
#define RL_CMD_HEIGHT_MAX   0.0f    /* m */

/* 推理路径: 投入后先零动作 N 步, 只跑 PD + 历史 (复现"接地前零动作"; 步数待训练侧) */
#define RL_WARMUP_STEPS     50u

/* 动作裁剪 (训练侧 clip_actions, 待训练侧; 0 = 不裁) */
#define RL_ACTION_CLIP      0.0f

#define RL_LATENT_SIZE      3u

/* 模型: networkzn1 = chuanliantui 起立策略 (model_6000, 2026-09-22) */
typedef enum {
    RL_MODEL_STANDUP = 0,
    RL_MODEL_COUNT
} rl_model_t;

typedef struct {
    rl_model_t selected_model;           /* 当前模型 */
    uint8_t model_ready[RL_MODEL_COUNT]; /* 模型状态 */
    uint8_t ready;                       /* 全部有效 */
    float latent[RL_LATENT_SIZE];        /* 编码输出 */
    uint32_t run_ok;                     /* 成功次数 */
    uint32_t run_fail;                   /* 失败次数 */
    uint32_t run_us;                     /* 上次耗时 */
} rl_policy_t;

/* CubeAI 网络 */
typedef struct {
    ai_handle network;
    ai_buffer *inputs;
    ai_buffer *outputs;
    uint8_t ready;
} rl_network_t;

void RL_Policy_Reset(rl_policy_t *policy);
uint8_t RL_Policy_Init(rl_policy_t *policy);
uint8_t RL_Policy_Select(rl_policy_t *policy, rl_model_t model);
uint8_t RL_Policy_Run(rl_policy_t *policy,
                      const rl_observation_state_t *observation,
                      float action[RL_ACTION_SIZE]);

#endif
