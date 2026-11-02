/*
 * UART DMA环形缓冲在IDLE中断时提取新字节，携带接收时刻交给任务解析。
 * 跨环尾分段复制；Take返回本次取走字节数，缓冲/缓存维护不等同协议帧校验。
 */

#ifndef __UART_IDLE_H
#define __UART_IDLE_H

#include "main.h"
#include "dma_cache.h"

#define DBUS_BUF_SIZE       64
#define DEBUG_BUF_SIZE      256
#define HI229_BUF_SIZE      128

typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;
    uint8_t  dma_buf[DEBUG_BUF_SIZE] __attribute__((aligned(DMA_CACHE_LINE_SIZE)));
    uint8_t  isr_buf[DEBUG_BUF_SIZE];
    volatile uint16_t isr_len;
    volatile uint8_t  flag;
    uint16_t buf_size;
    volatile uint16_t dma_pos;
    volatile uint64_t received_ns;
} UART_Rx_t;

extern UART_Rx_t dbus_rx;
extern UART_Rx_t debug_rx; /* VOFA预留 */
extern UART_Rx_t hi229_rx;

void UART_Rx_Init(UART_Rx_t *rx, UART_HandleTypeDef *huart,
                  DMA_HandleTypeDef *hdma_rx, uint16_t size);
void UART_Idle_Isr(UART_HandleTypeDef *huart, UART_Rx_t *rx);
uint16_t UART_Rx_Take_Timed(UART_Rx_t *rx, uint8_t *dst, uint16_t capacity,
                           uint64_t *received_ns);

#endif
