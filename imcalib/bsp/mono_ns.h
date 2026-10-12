/*
 * 文件职责：DWT周期扩展的单调纳秒时间（mono_ns.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：CPU周期计数、初始化与TIM6周期维护。
 * 输出/去向：给反馈接收/发布、观测采集和时间预算提供64位单调时刻。
 * 关键边界：读取/扩展保留原中断屏蔽状态；这是上电相对时间，不是墙上日期，复位后从头计数。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
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
