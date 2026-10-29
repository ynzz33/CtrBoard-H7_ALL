/* 高度门控1999：反馈、网络action均为四腿两轮顺序。 */
#ifndef RL_TYPES_H
#define RL_TYPES_H
#include <stdint.h>
#define RL_ACTION_SIZE 6u
#define RL_LEG_MOTOR_NUM 4u
#define RL_OBS_SIZE 35u

/* 对应训练[lf0,lf00,rf0,rf00,lfwheel,rfwheel]；不再使用旧虚拟腿角色交换。 */
typedef enum {
    RL_MOTOR_L_FRONT = 0, RL_MOTOR_L_REAR, RL_MOTOR_R_FRONT, RL_MOTOR_R_REAR, RL_MOTOR_L_WHEEL, RL_MOTOR_R_WHEEL
} rl_motor_index_t;

/* 1999网络[lf0,lf00,rf0,rf00,lfwheel,rfwheel]与内部顺序一致。 */
static inline uint8_t RL_Action_Index(uint8_t motor_index)
{
    static const uint8_t order[RL_ACTION_SIZE] = {0u, 1u, 2u, 3u, 4u, 5u};
    return order[motor_index];
}
typedef struct {
    float pos[RL_LEG_MOTOR_NUM];     /* 首次分支锚定的模型连续角，rad；轮位置观测固定0 */
    float vel[RL_ACTION_SIZE];      /* 同顺序四电机与两轮速度，rad/s */
    uint8_t valid;                  /* 源反馈可用且本次模型映射结果有限 */
} rl_motor_state_t;
#endif
