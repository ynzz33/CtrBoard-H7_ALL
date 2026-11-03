/* 诊断编号/时间不参与控制。执行任务写sample，策略任务在原临界区复制。 */
#ifndef LATENCY_STATE_H
#define LATENCY_STATE_H
#include "latency_capture.h"
#if MACHINE_LATENCY_CAPTURE_ENABLE
extern latency_sample_t trace_sample;
extern uint64_t trace_sample_rx[8];
extern uint32_t trace_policy_sequence;
extern uint64_t trace_policy_published_ns;
extern uint32_t trace_dispatch_sequence;
#endif
#endif
