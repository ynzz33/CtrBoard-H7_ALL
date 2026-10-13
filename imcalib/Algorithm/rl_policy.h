/*
 * 文件职责：CubeAI networkzn1模型生命周期与推理（rl_policy.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：已准备的RL历史向量、模型选择和网络运行库。
 * 输出/去向：管理网络句柄/工作区，输出6维动作和运行有效性/耗时。
 * 关键边界：仅封装网络调用；遥控预热、动作发布与电机执行分别由任务层/rl_torque负责。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef RL_POLICY_H
#define RL_POLICY_H

#include <stdint.h>

#include "rl_observation.h"
#include "rl_types.h"
#include "ai_platform.h"

/* 模型: networkzn1 = chuanliantui 起立策略 (model_6000, 2026-09-22) */


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
