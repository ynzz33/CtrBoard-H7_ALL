/* 台阶候选特征、连续确认及重新武装；不推进动作、不访问自起状态。 */
#ifndef STAIR_DETECTION_H
#define STAIR_DETECTION_H

#include "machine_config.h"

enum {
    STAIR_DETECT_ELIGIBLE = 1u, STAIR_DETECT_ARMED = 2u,
    STAIR_DETECT_POSE = 4u, STAIR_DETECT_ACCELERATION = 8u,
    STAIR_DETECT_CANDIDATE = 16u, STAIR_DETECT_TRIGGER = 32u,
    STAIR_DETECT_CONFIRMED = 64u, STAIR_DETECT_PAUSED = 128u,
};

/* 最近一次候选结束原因，锁存到下一次结束；不是公共故障或动作退出原因。 */
enum {
    STAIR_CLEAR_NONE = 0, STAIR_CLEAR_REQUEST, STAIR_CLEAR_YAW,
    STAIR_CLEAR_PERMIT, STAIR_CLEAR_TIMEOUT, STAIR_CLEAR_CONFIRMED,
    STAIR_CLEAR_DISARM, STAIR_CLEAR_INVALID,
};
#define STAIR_CLEAR_SHIFT 13u /* VOFA ch8 bit13～15，避开处理结果及确认位 */

typedef struct {
    float angle[2], rate[2], length[2], reference[2]; /* rad、rad/s、m、rad */
    float velocity_request, yaw_request, yaw_rate; /* m/s、rad/s、rad/s */
    uint8_t permit; /* 调度已确认：已投入、有效NORMAL且普通平衡运行 */
} stair_detection_input_t;

typedef struct {
    float acceleration[2], previous_rate[2]; /* rad/s²、rad/s */
    float deviation[2]; /* 当前几何角与站立参考的包角差rad */
    float elapsed; /* 冲击后姿态条件连续时间s */
    float impact_elapsed; /* 本次冲击窗口已过去时间s */
    uint32_t events; /* 自动候选事件序号，OBSERVE也计数 */
    uint16_t flags; /* STAIR_DETECT_*，暂停时不得解释为新测量 */
    uint8_t seeded, armed, trigger;
    uint8_t impact_pending, impact_high; /* 窗口锁存、上一拍冲击条件，用于只接受上升沿 */
    uint8_t clear_reason; /* STAIR_CLEAR_*，无候选时不会被反复覆盖 */
} stair_detection_t;

extern stair_detection_t stair_detection;

void Stair_Detection_Reset(stair_detection_t *state);
void Stair_Detection_Disarm(stair_detection_t *state);
/* 每个非暂停执行拍调用一次；输入有效性由调度端保证，dt固定为MACHINE_LQR_DT。 */
void Stair_Detection_Update(stair_detection_t *state, const stair_detection_param_t *param, const stair_detection_input_t *input, float dt);

#endif
