#ifndef JETSON_USB_H
#define JETSON_USB_H

#include "robot_control.h"

/* 行走策略的USB实控链路，以及Jetson端保存所需的PD遥测。 */
void Jetson_Usb_Record_Action_Observation(const float obs[RL_OBS_SIZE], uint8_t warmup);
void Jetson_Usb_Record_Sample(const control_frame_t *input, uint8_t pd_valid, uint8_t dispatched);
void Jetson_Usb_Task(void *argument);
void Jetson_Usb_Receive_From_ISR(const uint8_t *data, uint32_t length);
void Jetson_Usb_Notify_From_ISR(void);
uint8_t Jetson_Usb_External_Link_Ready(void);
void Jetson_Usb_External_Submit(const float *obs, const float *history, uint32_t reset);
uint8_t Jetson_Usb_External_Action_Allowed(void);
void Jetson_Usb_External_PD_Used(void);

#endif
