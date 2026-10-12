/*
 * 文件职责：串口VOFA JustFloat发送接口。
 * 输入：浮点通道数组及通道数量；输出：经所选UART的DMA提交一帧。
 * 端口只由machine_config.h中的MACHINE_VOFA_PORT选择：大机UART8，小机USART1。
 * 本接口不再维护UART/USB类型标识、运行时切换状态或切换许可。
 * DMA忙时跳过，帧缓冲由实现文件持有；波特率与上位机保持一致。
 */
#ifndef VOFA_SEND_H
#define VOFA_SEND_H

#include "main.h"
#include "usart.h"
#include "machine_config.h"

#define VOFA_MAX_CH 39u /* 通道容量 */

/* 真正选择硬件串口，数字是端口号，不是传输类型。 */
#if MACHINE_VOFA_PORT == 8u
#define VOFA_UART (&huart8)
#elif MACHINE_VOFA_PORT == 1u
#define VOFA_UART (&huart1)
#else
#error "MACHINE_VOFA_PORT must be 8 (UART8) or 1 (USART1)"
#endif

/* 成功返回1表示DMA提交成功，不代表上位机已收到。 */
uint8_t Vofa_Send(const float *data, uint8_t n);

#endif
