#include "machine_config.h"
#if MACHINE_LATENCY_CAPTURE_ENABLE
/* 固定容量时序队列：IRQ/任务只短临界区复制，CRC与USB打包在原后台完成。
 * 满队列只丢记录并计数，不阻塞/改控制；中断屏蔽状态必须原样恢复。
 */
#include "latency_capture.h"
#include "main.h"
#include "mono_ns.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef char latency_wire_size[(sizeof(latency_record_t) == LATENCY_RECORD_BYTES) ? 1 : -1];
volatile uint8_t latency_recording;
static latency_record_t records[LATENCY_CAPACITY];
static volatile uint32_t write_index, read_index;
static uint32_t sequence, dropped, high_water, packet_sequence, prepared_read, prepared_count;
static uint32_t can_pending;
static latency_tick_t ticks[2];
static TaskHandle_t dispatch_owner;
static uint32_t dispatch_sequence, dispatch_policy, dispatch_reset;

uint32_t Latency_Age_Us(uint64_t now_ns, uint64_t source_ns)
{
    uint64_t age;
    if (source_ns == 0u || source_ns > now_ns) { return UINT32_MAX; }
    age = (now_ns - source_ns) / 1000u;
    return age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
}

void Latency_Capture_Push(const latency_record_t *record)
{
    uint32_t mask, used, next;
    if (!latency_recording && record->kind != LATENCY_CAN) { return; }
    mask = __get_PRIMASK();
    __disable_irq();
    next = ++sequence;
    used = write_index - read_index;
    if (used == LATENCY_CAPACITY) { dropped++; }
    else
    {
        records[write_index % LATENCY_CAPACITY] = *record;
        records[write_index % LATENCY_CAPACITY].sequence = next;
        __DMB();
        write_index++;
        if (used + 1u > high_water) { high_water = used + 1u; }
    }
    __set_PRIMASK(mask);
}

void Latency_Tick(uint8_t policy)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ticks[policy].sequence++;
    ticks[policy].ns = Mono_Ns_Get();
    __set_PRIMASK(mask);
}

latency_tick_t Latency_Tick_Read(uint8_t policy)
{
    latency_tick_t result;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    result = ticks[policy];
    __set_PRIMASK(mask);
    return result;
}

void Latency_Dispatch_Begin(uint32_t output_sequence, uint32_t policy_sequence, uint32_t reset_count)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    dispatch_sequence = output_sequence;
    dispatch_policy = policy_sequence;
    dispatch_reset = reset_count;
    dispatch_owner = xTaskGetCurrentTaskHandle();
    __set_PRIMASK(mask);
}

void Latency_Dispatch_End(void) { dispatch_owner = NULL; }

void Latency_Can_Context(latency_record_t *record)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (dispatch_owner != NULL && dispatch_owner == xTaskGetCurrentTaskHandle())
    {
        record->value[6] = dispatch_sequence;
        record->value[7] = dispatch_policy;
        record->reset_count = dispatch_reset;
        record->flags |= 0x80u; /* CAN记录确有同任务分发上下文，零序号回绕仍可区分。 */
    }
    __set_PRIMASK(mask);
}

void Latency_Can_Pending(uint8_t added)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (added) { can_pending++; }
    else { can_pending--; }
    __set_PRIMASK(mask);
}

static uint16_t Latency_Crc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t byte;

    /* 与逐位CCITT-FALSE等价的逐字节展开，避免大批次每字节循环8次。 */
    for (uint16_t i = 0u; i < length; i++)
    {
        byte = (uint16_t)((crc >> 8) ^ data[i]);
        byte ^= byte >> 4;
        crc = (uint16_t)((crc << 8) ^ (byte << 12) ^ (byte << 5) ^ byte);
    }
    return crc;
}

/* 与H7OUT001同一次提交；头含独立序号/丢弃/积压，绝不以零值填补缺号。 */
uint16_t Latency_Capture_Packet(uint8_t *out)
{
    static const uint8_t magic[8] = {'H','7','L','A','T','0','0','1'};
    uint32_t header[6], mask;
    uint16_t format[4], size, crc;
    mask = __get_PRIMASK();
    __disable_irq();
    prepared_read = read_index;
    prepared_count = write_index - read_index;
    header[0] = packet_sequence + 1u; header[1] = sequence; header[2] = dropped;
    header[3] = prepared_count; header[4] = high_water; header[5] = can_pending;
    __set_PRIMASK(mask);
    if (prepared_count > LATENCY_BATCH) { prepared_count = LATENCY_BATCH; }
    format[0] = 1u; format[1] = LATENCY_RECORD_BYTES;
    format[2] = (uint16_t)prepared_count; format[3] = 0u;
    memcpy(out, magic, 8u); memcpy(out + 8u, format, 8u); memcpy(out + 16u, header, 24u);
    for (uint32_t i = 0u; i < prepared_count; i++)
    { memcpy(out + 40u + i * LATENCY_RECORD_BYTES, &records[(prepared_read + i) % LATENCY_CAPACITY], LATENCY_RECORD_BYTES); }
    size = (uint16_t)(40u + prepared_count * LATENCY_RECORD_BYTES);
    crc = Latency_Crc(out, size); memcpy(out + size, &crc, 2u);
    return size + 2u;
}

void Latency_Capture_Commit(void)
{
    __DMB(); read_index = prepared_read + prepared_count; packet_sequence++; prepared_count = 0u;
}

#endif

#if MACHINE_LATENCY_CAPTURE_ENABLE
#include "latency_state.h"
latency_sample_t trace_sample;
uint64_t trace_sample_rx[8];
uint32_t trace_policy_sequence;
uint64_t trace_policy_published_ns;
uint32_t trace_dispatch_sequence;
#endif
