#ifndef VOFA_SEND_H
#define VOFA_SEND_H

#include "main.h"
#include "usart.h"

#define VOFA_MAX_CH  32

/* 串口号 */
#define VOFA_UART   &huart1

void Vofa_Send(const float *data, uint8_t n);

#endif
