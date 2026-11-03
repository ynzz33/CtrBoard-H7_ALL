#ifndef LATENCY_USB_H
#define LATENCY_USB_H
#include <stdint.h>
void Latency_Usb_Pump(void);
void Latency_Usb_Receive(const uint8_t *data,uint32_t size);
#endif
