#ifndef JETSON_USB_H
#define JETSON_USB_H

#include "robot_control.h"
#include "jetson_protocol.h"

/* 行走策略的USB实控链路，以及Jetson端保存所需的PD遥测。 */
void Jetson_Usb_Record_Action_Observation(const float obs[RL_OBS_SIZE], uint8_t warmup);
void Jetson_Usb_Record_Sample(const control_frame_t *input, uint8_t pd_valid, uint8_t dispatched);
void Jetson_Usb_Task(void *argument);
void Jetson_Usb_Record_Swing(uint32_t trial, const uint32_t status[6], const float values[JRL_SWING_FLOATS]);
void Jetson_Usb_Receive_From_ISR(const uint8_t *data, uint32_t length);
void Jetson_Usb_Notify_From_ISR(void);
/* LQR接管时发起一次冷启动，收到新会话就绪后结束等待。 */
void Jetson_Usb_Request_Cold_Restart(void);
uint8_t Jetson_Usb_Cold_Restart_Pending(void);
uint8_t Jetson_Usb_External_Link_Ready(void);
void Jetson_Usb_External_Submit(const float *obs, const float *history, uint32_t reset);
uint8_t Jetson_Usb_External_Action_Allowed(void);
void Jetson_Usb_External_PD_Used(void);

#endif
