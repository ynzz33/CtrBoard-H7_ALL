/*
 * 通过独立UART发送只读反馈镜像，接收并校验探测包，保存序号和往返探测信息。
 * 编译开关关闭时使用空入口；DMA忙跳过发送，不等待、不接收控制动作。
 */
/*
 * Jetson独立串口：发送固定48路镜像并接收CRC探测包。
 * 不复用VOFA发送函数、缓冲或端口，也不接收控制动作。
 */
#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <stdint.h>

#define JETSON_UART_PROTOCOL 1u
#define JETSON_UART_PACKET_BYTES 14u
#define JETSON_MIRROR_CH 48u

typedef struct {
    uint32_t sequence;
    uint32_t token;
    uint32_t received;
    uint32_t crc_errors;
    uint32_t rx_restarts;
} jetson_uart_status_t;

extern jetson_uart_status_t jetson_uart_status;

void Jetson_Uart_Init(void);
void Jetson_Uart_Process(void);
void Jetson_Mirror_Send(void);

#endif
