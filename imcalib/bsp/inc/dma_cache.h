/*
 * 文件职责：M7 DMA收发缓存一致性辅助接口（dma_cache.h）。
 * 本头文件提供编译宏或内联辅助，由引用方使用，不需要独立的同名实现文件。
 * 输入/来源：DMA缓冲地址、长度和cache line边界。
 * 输出/去向：发送前clean，接收读取前invalidate，统一处理地址对齐。
 * 关键边界：仅保证CPU cache与DMA内存可见性，不负责缓冲所有权、协议解析或任务同步。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef DMA_CACHE_H
#define DMA_CACHE_H

#include "main.h"
#include <stdint.h>

#define DMA_CACHE_LINE_SIZE  32u

/* 清理发送缓存 */
static inline void Dma_Cache_Clean_Tx(const void *address, uint32_t length)
{
    uintptr_t first_line;
    uintptr_t after_last_line;

    if (length == 0u || (SCB->CCR & SCB_CCR_DC_Msk) == 0u)
        return;

    first_line = (uintptr_t)address & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    after_last_line = ((uintptr_t)address + length + DMA_CACHE_LINE_SIZE - 1u) & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    SCB_CleanDCache_by_Addr((uint32_t *)first_line, (int32_t)(after_last_line - first_line));
    __DSB();
}

/* 刷新接收缓存 */
static inline void Dma_Cache_Invalidate_Rx(const void *address, uint32_t length)
{
    uintptr_t first_line;
    uintptr_t after_last_line;

    if (length == 0u || (SCB->CCR & SCB_CCR_DC_Msk) == 0u)
        return;

    first_line = (uintptr_t)address & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    after_last_line = ((uintptr_t)address + length + DMA_CACHE_LINE_SIZE - 1u) & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    SCB_InvalidateDCache_by_Addr((uint32_t *)first_line, (int32_t)(after_last_line - first_line));
    __DSB();
}

#endif
