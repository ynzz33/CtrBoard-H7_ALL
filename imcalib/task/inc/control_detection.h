/*
 * 执行层检测调度：消费同拍快照，推进公共离地观测和三态判断。
 * 不生成控制目标/力矩；超时撤力后复位旧历史并重新初判，返回进入FALLEN事件。
 */
#ifndef CONTROL_DETECTION_H
#define CONTROL_DETECTION_H

#include "robot_control.h"

uint8_t Control_Detection_Update(control_frame_t *frame);

typedef enum {
    CONTROL_POSTPROCESS_RUN = 0,
    CONTROL_POSTPROCESS_WITHDRAW_WAIT,
    CONTROL_POSTPROCESS_RECHECK
} control_postprocess_phase_t;

/* 任务层后处理期限与撤力状态；HAL毫秒时钟，不按调用次数累计。供Watch观察。 */
typedef struct {
    control_postprocess_phase_t phase;
    uint32_t wait_start_ms;
    uint32_t flight_start_ms;
    uint8_t flight_timing;
    uint8_t recovery_retry;
    uint8_t exhausted;
} control_postprocess_t;

extern control_postprocess_t control_postprocess;

void Control_Postprocess_Request(uint8_t recovery_timeout);
void Control_Postprocess_Flight_Update(uint8_t active);
uint8_t Control_Postprocess_Inhibited(void);

#endif
