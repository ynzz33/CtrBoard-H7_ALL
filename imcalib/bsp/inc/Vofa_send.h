/*
 * 文件职责：串口VOFA JustFloat发送接口。
 * 输入：浮点通道数组及通道数量；输出：经所选UART的DMA提交一帧。
 * 在线VOFA使用VOFA_UART。
 * 本接口不再维护UART/USB类型标识、运行时切换状态或切换许可。
 * DMA忙时跳过，帧缓冲由实现文件持有；波特率与上位机保持一致。
 */
#ifndef VOFA_SEND_H
#define VOFA_SEND_H

#include "main.h"
#include "usart.h"

#define VOFA_MAX_CH 48u /* 固定上限48路，不扩通道。 */

#define VOFA_UART (&huart8)

/* 成功返回1表示DMA提交成功，不代表上位机已收到。 */
uint8_t Vofa_Send(const float *data, uint8_t n);

#endif
