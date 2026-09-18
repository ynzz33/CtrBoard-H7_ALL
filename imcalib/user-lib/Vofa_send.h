#ifndef VOFA_SEND_H
#define VOFA_SEND_H

#include "main.h"
#include "usart.h"

#define VOFA_MAX_CH  32   /* 上限 32; 当前实际发 27 路 (见 task_comm.c) */

/* Vofa 发送串口: 改这个数字即可 (也可用 -DVOFA_PORT=n 覆盖) */
/*   8 = UART8  (默认, 921600, TX DMA 正常)           */
/*   1 = USART1 (空闲,   921600, TX DMA 正常)          */
/* 注意: UART7 被 HI229 占用且 TX DMA 是 CIRCULAR 模式; */
/*       UART9 被 DR16 占用且没有 TX DMA, 都不能选。     */
/* 波特率必须与 Vofa+ 一致 (CubeMX 里对应口已是 921600)。*/
#ifndef VOFA_PORT
#define VOFA_PORT   1
#endif

#if VOFA_PORT == 8
#define VOFA_UART   &huart8
#elif VOFA_PORT == 1
#define VOFA_UART   &huart1
#else
#error "VOFA_PORT must be 8 (UART8) or 1 (USART1)"
#endif

void Vofa_Send(const float *data, uint8_t n);

#endif
