/* 8000 v2：内部反馈按四腿两轮排列，网络动作按左右两组排列。 */
#ifndef RL_TYPES_H
#define RL_TYPES_H
#include <stdint.h>
#define RL_ACTION_SIZE 6u
#define RL_LEG_MOTOR_NUM 4u
#define RL_OBS_SIZE 25u
#define RL_HISTORY_FRAMES 5u
#define RL_HISTORY_SIZE (RL_OBS_SIZE * RL_HISTORY_FRAMES)
#define RL_ACTION_CLIP 100.0f

/* 固件内部顺序保持不变，避免修改实机电机配置及反馈索引。 */
typedef enum {
    RL_MOTOR_L_FRONT = 0, RL_MOTOR_L_REAR, RL_MOTOR_R_FRONT, RL_MOTOR_R_REAR, RL_MOTOR_L_WHEEL, RL_MOTOR_R_WHEEL
} rl_motor_index_t;

/* 内部电机索引→网络[lf0,lf00,lfwheel,rf0,rf00,rfwheel]索引。 */
static inline uint8_t RL_Action_Index(uint8_t motor_index)
{
    static const uint8_t order[RL_ACTION_SIZE] = {0u, 1u, 3u, 4u, 2u, 5u};
    return order[motor_index];
}
typedef struct {
    float pos[RL_LEG_MOTOR_NUM];     /* 首次分支锚定的模型连续角，rad；轮位置观测固定0 */
    float vel[RL_ACTION_SIZE];      /* 同顺序四电机与两轮速度，rad/s */
    uint8_t valid;                  /* 源反馈可用且本次模型映射结果有限 */
} rl_motor_state_t;
#endif
