/*
 * 文件职责：将浮点数组打包为VOFA JustFloat帧，通过所选串口DMA发送。
 * 调用链：后台服务 → Vofa_Debug_Send → Vofa_Send → HAL_UART_Transmit_DMA。
 * 帧内容是n个float加固定4字节帧尾；静态缓冲在异步发送完成前不能被覆盖。
 * 串口由MACHINE_VOFA_PORT编译选择，本文件不再提供UART/USB运行时切换。
 */
#include "Vofa_send.h"
#include "dma_cache.h"
#include <string.h>

static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* 校验输入 → 检查DMA空闲 → 打包 → 清理缓存 → 提交DMA；失败/忙不等待。 */
uint8_t Vofa_Send(const float *data, uint8_t n)
{
    static uint8_t buf[VOFA_MAX_CH * sizeof(float) + 4]
        __attribute__((aligned(DMA_CACHE_LINE_SIZE)));
    uint16_t data_len;

    if (data == NULL || n == 0u || n > VOFA_MAX_CH)
    {
        return 0u;
    }
    /* 必须在memcpy前检查，不能先覆盖DMA仍在读取的上一帧。 */
    if (VOFA_UART->gState != HAL_UART_STATE_READY)
    {
        return 0u;
    }
    data_len = (uint16_t)(n * sizeof(float));
    memcpy(buf, data, data_len);
    memcpy(buf + data_len, tail, sizeof(tail));
    Dma_Cache_Clean_Tx(buf, data_len + sizeof(tail));
    return (uint8_t)(HAL_UART_Transmit_DMA(VOFA_UART, buf,
        (uint16_t)(data_len + sizeof(tail))) == HAL_OK);
}
