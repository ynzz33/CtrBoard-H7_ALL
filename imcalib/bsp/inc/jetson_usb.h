#ifndef JETSON_USB_H
#define JETSON_USB_H

#include "robot_control.h"

void Jetson_Usb_Record_Action_Observation(const float obs[RL_OBS_SIZE], uint8_t warmup);
void Jetson_Usb_Record_Sample(const control_frame_t *input, uint8_t pd_valid, uint8_t dispatched);
void Jetson_Usb_Record_Pump(void);

#if MACHINE_JETSON_SHADOW_CAPTURE_ENABLE
typedef struct {
    uint64_t input_ready_us;
    uint32_t sequence;
    uint32_t reset_generation;
    uint32_t dropped;
    float obs[RL_OBS_SIZE];
    float history[RL_HISTORY_SIZE];
    float raw_action[RL_ACTION_SIZE];
} jetson_shadow_sample_t;

uint8_t Jetson_Usb_Shadow_Begin(const float *obs, const float *history, uint64_t input_ready_us);
void Jetson_Usb_Shadow_Publish(const float *raw_action);
uint8_t Jetson_Usb_Shadow_Take(jetson_shadow_sample_t *sample);
void Jetson_Usb_Shadow_Receive(const uint8_t *data, uint32_t length);
#endif

#endif
