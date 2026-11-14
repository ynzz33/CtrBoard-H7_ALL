#ifndef JETSON_USB_H
#define JETSON_USB_H

#include "robot_control.h"

void Jetson_Usb_Record_Action_Observation(const float obs[RL_OBS_SIZE], uint8_t warmup);
void Jetson_Usb_Record_Sample(const control_frame_t *input, uint8_t pd_valid, uint8_t dispatched);
void Jetson_Usb_Record_Pump(void);

#endif
