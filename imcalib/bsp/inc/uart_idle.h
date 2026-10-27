/*
 * 文件职责：串口DMA环形接收与IDLE缓存交接（uart_idle.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：UART/DMA句柄、环形DMA位置、IDLE中断和接收时刻。
 * 输出/去向：复制本次新字节及时间戳，由任务侧Take后交给协议解析。
 * 关键边界：ISR负责取新字节与发布缓存，不做控制求解；跨环尾分两段复制，M7缓存由dma_cache维护。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
