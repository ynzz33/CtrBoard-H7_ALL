/* 世界腿角/负向角加速度联合连续确认；不推进动作、不访问自起状态。 */
#ifndef STAIR_DETECTION_H
#define STAIR_DETECTION_H

#include "machine_config.h"
#include "simple-function.h"

enum {
    STAIR_DETECT_ELIGIBLE = 1u, STAIR_DETECT_ARMED = 2u,
    STAIR_DETECT_POSE = 4u, STAIR_DETECT_ACCELERATION = 8u,
    STAIR_DETECT_CANDIDATE = 16u, STAIR_DETECT_TRIGGER = 32u,
    STAIR_DETECT_CONFIRMED = 64u, STAIR_DETECT_PAUSED = 128u,
};

typedef struct {
    float angle[2], rate[2], reference[2]; /* 世界腿角rad、世界腿角速度rad/s、参考rad */
    uint8_t permit; /* 调度已确认：已投入、有效NORMAL且普通平衡运行 */
} stair_detection_input_t;

typedef struct {
    lpf_t acceleration[2]; /* 复用公共低通，out为世界角加速度rad/s² */
    float previous_rate[2]; /* 上一拍世界腿角速度rad/s */
    float deviation[2]; /* 世界腿角与站立参考的包角差rad */
    float elapsed; /* 角度/冲击联合条件连续时间s */
    uint32_t events; /* 自动候选事件序号，OBSERVE也计数 */
    uint16_t flags; /* STAIR_DETECT_*，暂停时不得解释为新测量 */
    uint8_t seeded, armed, trigger;
} stair_detection_t;

extern stair_detection_t stair_detection;

void Stair_Detection_Reset(stair_detection_t *state);
void Stair_Detection_Disarm(stair_detection_t *state);
/* 每个非暂停执行拍调用一次；输入有效性由调度端保证，dt固定为MACHINE_LQR_DT。 */
void Stair_Detection_Update(stair_detection_t *state, const stair_detection_param_t *param, const stair_detection_input_t *input, float dt);

#endif
