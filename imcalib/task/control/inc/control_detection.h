/*
 * 公共三态选路、NORMAL内台阶检测，以及独立台阶退出时的公共重新初判。
 * 超时后先撤力等待再重判；动作超时不是姿态已经倒地的证据。
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

/* 自起/AIR后处理期限与撤力状态；台阶退出独立管理，不借用此处预算。 */
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
