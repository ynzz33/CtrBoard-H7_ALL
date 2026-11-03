/* 只读输出队列：控制快照与实际DJI CAN提交分开记录；不生成目标或发送电机命令。 */
#ifndef OUTPUT_CAPTURE_H
#define OUTPUT_CAPTURE_H

#include <stdint.h>
#include "machine_config.h"

#define OUTPUT_CAPTURE_CONTROL 1u
#define OUTPUT_CAPTURE_CAN 2u
#define OUTPUT_CAPTURE_CAPACITY 128u
#define OUTPUT_CAPTURE_BATCH 48u /* 实测镜像约78Hz；批量需覆盖禁用档每秒约2000条记录。 */
#define OUTPUT_CAPTURE_HEADER_BYTES 40u
#define OUTPUT_CAPTURE_RECORD_BYTES 88u
#define OUTPUT_CAPTURE_PACKET_MAX (OUTPUT_CAPTURE_HEADER_BYTES + OUTPUT_CAPTURE_BATCH * OUTPUT_CAPTURE_RECORD_BYTES + 2u)

typedef struct {
    uint32_t sequence; /* 所有入队尝试编号；队满仍递增，缺口不能冒充完整数据。 */
    uint32_t kind;
    uint64_t begin_ns; /* 控制分发开始或实际CAN调用开始；不是电机生效时刻。 */
    uint64_t end_ns;
    uint32_t id; /* CONTROL:输出拍序号；CAN:实际报文ID。 */
    uint32_t fault;
    float torque_nm[6]; /* CONTROL:门控后四腿两轮请求，固件方向，非实测轴矩。 */
    float wheel_speed[2]; /* CONTROL:该执行拍实际使用的公共反馈，轮轴rad/s。 */
    uint32_t wheel_age_us[2]; /* CONTROL:上述反馈原接收年龄，未知UINT32_MAX。 */
    int16_t current_raw[4]; /* CAN:从实际待提交报文字节读取的四槽电流码。 */
    uint16_t flags; /* CONTROL:镜像输出标志；CAN:HAL返回值，0仅表示入队成功。 */
    uint8_t strategy;
    uint8_t bench_phase;
    uint8_t bench_reason;
    uint8_t machine;
    uint8_t wheel_online;
    uint8_t controller_mode; /* 本轮固定原控制器0，不接入补偿。 */
} output_capture_record_t;

/* 多任务生产端只做固定大小复制；队满只计数，不等待、不影响原输出。 */
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Output_Capture_Push(const output_capture_record_t *record);
#else
static inline void Output_Capture_Push(const output_capture_record_t *record) { (void)record; }
#endif
/* 从实际Can_Bus_Transmit使用的8字节读取电流码，覆盖通信任务紧急零输出。 */
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Output_Capture_CAN(const uint8_t data[8], uint32_t can_id, uint8_t status, uint64_t begin_ns, uint64_t end_ns);
#else
static inline void Output_Capture_CAN(const uint8_t data[8], uint32_t can_id, uint8_t status, uint64_t begin_ns, uint64_t end_ns) { (void)data; (void)can_id; (void)status; (void)begin_ns; (void)end_ns; }
#endif
/* 只在原VOFA后台使用；USB提交失败不出队，成功才Commit。 */
#if MACHINE_LATENCY_CAPTURE_ENABLE
uint16_t Output_Capture_Packet(uint8_t *out);
#else
static inline uint16_t Output_Capture_Packet(uint8_t *out) { (void)out; return 0u; }
#endif
#if MACHINE_LATENCY_CAPTURE_ENABLE
void Output_Capture_Commit(void);
#else
static inline void Output_Capture_Commit(void) {  }
#endif

#endif
