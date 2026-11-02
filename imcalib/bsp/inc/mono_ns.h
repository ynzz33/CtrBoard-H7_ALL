/*
 * 将DWT的32位CPU周期计数扩展为64位并换算上电相对纳秒。
 * TIM6定期累计以跨越回绕；读取时保存并恢复中断屏蔽状态，不能用作日历时间。
 */

#ifndef __MONO_NS_H
#define __MONO_NS_H

#include <stdint.h>

/* SYSCLK (MHz) */
#define MONO_NS_CPU_MHZ   550u

void     Mono_Ns_Init(void);
void     Mono_Ns_Tick(void);
uint64_t Mono_Ns_Get(void);

#endif
