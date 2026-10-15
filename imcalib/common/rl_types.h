/* RL39实体电机顺序；网络原始action另按左右两侧排列。 */
#ifndef RL_TYPES_H
#define RL_TYPES_H
#include <stdint.h>
#define RL_ACTION_SIZE 6u
#define RL_LEG_MOTOR_NUM 4u
#define RL_OBS_SIZE 39u

/* 对应训练[lf0,lf00,rf0,rf00,lfwheel,rfwheel]；不再使用旧虚拟腿角色交换。 */
typedef enum {
    RL_MOTOR_L_FRONT = 0, RL_MOTOR_L_REAR, RL_MOTOR_R_FRONT, RL_MOTOR_R_REAR, RL_MOTOR_L_WHEEL, RL_MOTOR_R_WHEEL
} rl_motor_index_t;

/* 内部四腿两轮→网络[lf0,lf00,lfwheel,rf0,rf00,rfwheel]，目标和历史观测共用。 */
static inline uint8_t RL_Action_Index(uint8_t motor_index)
{
    static const uint8_t order[RL_ACTION_SIZE] = {0u, 1u, 3u, 4u, 2u, 5u};
    return order[motor_index];
}
typedef struct {
    float pos[RL_LEG_MOTOR_NUM];     /* 模型坐标四电机连续角，rad；轮位置观测固定0 */
    float vel[RL_ACTION_SIZE];      /* 同顺序四电机与两轮速度，rad/s */
    uint8_t valid;                  /* 源反馈可用且本次模型映射结果有限 */
} rl_motor_state_t;
#endif
