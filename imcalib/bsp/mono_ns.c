/*
 * 文件职责：DWT周期扩展的单调纳秒时间（mono_ns.c）。
 * 输入/来源：CPU周期计数、初始化与TIM6周期维护。
 * 输出/去向：给反馈接收/发布、观测采集和时间预算提供64位单调时刻。
 * 关键边界：读取/扩展保留原中断屏蔽状态；这是上电相对时间，不是墙上日期，复位后从头计数。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "mono_ns.h"
#include "main.h"

/* 周期基数 */
static uint64_t cyc_base;
static uint32_t cyc_last;

/* 初始化 */
void Mono_Ns_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u;     /* M7 解锁 */
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cyc_base = 0u;
    cyc_last = DWT->CYCCNT;
}

/* 周期扩展: 随 TIM6 调用, 至少 7.8s 一次 */
void Mono_Ns_Tick(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now;

    __disable_irq();
    now = DWT->CYCCNT;
    cyc_base += (uint32_t)(now - cyc_last);
    cyc_last = now;
    __set_PRIMASK(primask);
}

/* 读 ns */
uint64_t Mono_Ns_Get(void)
{
    uint32_t primask = __get_PRIMASK();
    uint64_t cyc;

    __disable_irq();
    cyc = cyc_base + (uint32_t)(DWT->CYCCNT - cyc_last);
    __set_PRIMASK(primask);

    return (cyc * 1000ULL) / (uint64_t)MONO_NS_CPU_MHZ;
}
