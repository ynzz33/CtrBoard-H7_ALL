/* 板端时序旁带：只记录同一Mono_Ns时钟，不推进控制、不等待USB或CAN。
 * H7LAT001独立于镜像v2/H7OUT001；值的单位由kind定义，未知数据龄为UINT32_MAX。
 * CAN完成使用IRQ/复用前轮询时间，是含软件响应的上界，不代表电机力矩生效。
 */
#ifndef LATENCY_CAPTURE_H
#define LATENCY_CAPTURE_H
#include <stdint.h>
#include "machine_config.h"

extern volatile uint8_t latency_recording; /* 仅控制测量，不参与机器人许可。 */
#define LATENCY_SAMPLE 1u
#define LATENCY_POLICY 2u
#define LATENCY_PD 3u
#define LATENCY_ACTUATION 4u
#define LATENCY_CAN 5u
#define LATENCY_CAPACITY 1024u
#define LATENCY_BATCH 128u
#define LATENCY_RECORD_BYTES 64u
#define LATENCY_PACKET_MAX (40u + LATENCY_BATCH * LATENCY_RECORD_BYTES + 2u)

/* 所有记录均64字节小端；correlation为样本/策略/执行拍/CAN尝试编号。
 * SAMPLE: begin=反馈快照，end=缓存发布，value=IMU/四髋/双轮/遥控接收年龄us。
 * POLICY: begin=任务入口，end=发布/退出，value=sample序号/read偏移ns/build耗时us/
 *         SDK耗时us(失败或未执行为UINT32_MAX)/此前policy序号/模式/恢复阶段/最新策略触发序号。
 *         flags=0未选择、1坏观测、2SDK失败、3跨reset丢弃、4发布成功。
 * PD: begin/end=PD求解，value=所用policy序号/action年龄us/六电机接收年龄us；
 *     flags bit0=PD有效，bit1=该reset/policy首次有效PD消费；不证明最终许可出力。
 * ACTUATION: begin/end=执行函数，value=触发序号/入口距最近触发us/实际分发序号/
 *            PD所用policy序号/策略/故障/恢复阶段/样本序号；flags bit0=实际分发。
 * CAN: begin=提交入口，end=完成观测，source=总线，value=CAN ID/硬件buffer索引/
 *      HAL返回ns低高32位/原始8字节/分发序号/所用policy序号。
 *      flags低7位=1完成IRQ、2复用前完成轮询、3HAL失败、4停止/复用但未见成功；bit7=有同任务分发上下文。
 *      correlation只关联同一次提交；HAL失败时value[1]改为原HAL返回状态。
 */
typedef struct {
    uint32_t sequence;
    uint16_t kind;
    uint8_t source;
    uint8_t flags;
    uint64_t begin_ns;
    uint64_t end_ns;
    uint32_t correlation;
    uint32_t reset_count;
    uint32_t value[8];
} latency_record_t;

/* 执行任务发布、策略任务在原样本临界区复制；仅为配对，不参与控制。 */
typedef struct {
    uint32_t sequence;
    uint64_t cached_ns;
} latency_sample_t;
typedef struct {
    uint32_t sequence;
    uint64_t ns;
} latency_tick_t;

#if MACHINE_LATENCY_CAPTURE_ENABLE
uint16_t Latency_Crc16(const uint8_t *data, uint16_t length);
void Latency_Capture_Push(const latency_record_t *record);
#else
static inline void Latency_Capture_Push(const latency_record_t *record) { (void)record; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
uint32_t Latency_Age_Us(uint64_t now_ns, uint64_t source_ns);
#else
static inline uint32_t Latency_Age_Us(uint64_t now_ns, uint64_t source_ns) { (void)now_ns; (void)source_ns; return 0u; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Latency_Tick(uint8_t policy);
#else
static inline void Latency_Tick(uint8_t policy) { (void)policy; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
latency_tick_t Latency_Tick_Read(uint8_t policy);
#else
static inline latency_tick_t Latency_Tick_Read(uint8_t policy) { (void)policy; return  (latency_tick_t){0}; }
#endif
/* 分发上下文仅对设置它的任务有效；通信任务抢占时不会继承别人的控制拍。 */
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Latency_Dispatch_Begin(uint32_t output_sequence, uint32_t policy_sequence, uint32_t reset_count);
#else
static inline void Latency_Dispatch_Begin(uint32_t output_sequence, uint32_t policy_sequence, uint32_t reset_count) { (void)output_sequence; (void)policy_sequence; (void)reset_count; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Latency_Dispatch_End(void);
#else
static inline void Latency_Dispatch_End(void) {  }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Latency_Can_Context(latency_record_t *record);
#else
static inline void Latency_Can_Context(latency_record_t *record) { (void)record; }
#endif
/* 成功入队后增加、完成/停止后减少；包头保留当前未导出完成的CAN项数。 */
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Latency_Can_Pending(uint8_t added);
#else
static inline void Latency_Can_Pending(uint8_t added) { (void)added; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
uint16_t Latency_Capture_Packet(uint8_t *out);
#else
static inline uint16_t Latency_Capture_Packet(uint8_t *out) { (void)out; return 0u; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Latency_Capture_Commit(void);
#else
static inline void Latency_Capture_Commit(void) {  }
#endif
#endif
