/*
 * 文件职责：RL模型编号、动作大小等公共定义（rl_types.h）。
 * 本头文件定义共享数据类型、常量或内联辅助，由引用方读取/调用，不单独创建任务或维护外设。
 * 输入/来源：训练模型与固件部署接口约定。
 * 输出/去向：供观测/推理/力矩模块共享动作维度和模型类型。
 * 关键边界：纯类型头，不包含网络运行库、任务共享状态或策略选择逻辑。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef RL_TYPES_H
#define RL_TYPES_H

#define RL_ACTION_SIZE 6u
#define RL_LATENT_SIZE 3u

typedef enum {
    RL_MODEL_STANDUP = 0,
    RL_MODEL_COUNT
} rl_model_t;

#endif
