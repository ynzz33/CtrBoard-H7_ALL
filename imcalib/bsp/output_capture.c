#include "machine_config.h"
#if MACHINE_LATENCY_CAPTURE_ENABLE
/* 固定容量只读采集：短临界区入队；CRC和批量打包留在原低优先级VOFA后台。 */
#include "output_capture.h"
#include "latency_capture.h"
#include "main.h"
#include "machine_config.h"
#include <string.h>

typedef char output_capture_wire_size[(sizeof(output_capture_record_t) == OUTPUT_CAPTURE_RECORD_BYTES) ? 1 : -1];
static output_capture_record_t records[OUTPUT_CAPTURE_CAPACITY];
static volatile uint32_t write_index;
static volatile uint32_t read_index;
static uint32_t sequence;
static uint32_t dropped;
static uint32_t high_water;
static uint32_t packet_sequence;
static uint32_t prepared_read;
static uint32_t prepared_count;

/* IRQ保护同时覆盖执行任务和通信任务的停机报文；临界区不做CRC、USB或等待。 */
void Output_Capture_Push(const output_capture_record_t *record)
{
    uint32_t primask;
    uint32_t used;
    uint32_t next_sequence;

    if (!latency_recording) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    next_sequence = ++sequence;
    used = write_index - read_index;
    if (used >= OUTPUT_CAPTURE_CAPACITY)
    {
        dropped++;
    }
    else
    {
        records[write_index % OUTPUT_CAPTURE_CAPACITY] = *record;
        records[write_index % OUTPUT_CAPTURE_CAPACITY].sequence = next_sequence;
        __DMB();
        write_index++;
        if (used + 1u > high_water)
        {
            high_water = used + 1u;
        }
    }
    __set_PRIMASK(primask);
}

/* CAN字节是最终发送刻度；不调用力矩换算函数，也不回写wheel_current。 */
void Output_Capture_CAN(const uint8_t data[8], uint32_t can_id, uint8_t status, uint64_t begin_ns, uint64_t end_ns)
{
    output_capture_record_t record = {0};

    record.kind = OUTPUT_CAPTURE_CAN;
    record.begin_ns = begin_ns;
    record.end_ns = end_ns;
    record.id = can_id;
    record.flags = status;
    record.machine = (uint8_t)MACHINE_DEFAULT;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        record.current_raw[i] = (int16_t)(((uint16_t)data[2u * i] << 8) | data[2u * i + 1u]);
    }
    Output_Capture_Push(&record);
}

static uint16_t Capture_Crc16(const uint8_t *data, uint16_t count)
{
    uint16_t crc = 0xFFFFu;
    uint16_t byte;

    /* 与逐位CCITT-FALSE等价的逐字节展开，避免大批次每字节循环8次。 */
    for (uint16_t i = 0u; i < count; i++)
    {
        byte = (uint16_t)((crc >> 8) ^ data[i]);
        byte ^= byte >> 4;
        crc = (uint16_t)((crc << 8) ^ (byte << 12) ^ (byte << 5) ^ byte);
    }
    return crc;
}

/* H7OUT001 + 40字节头 + count*88字节记录 + CRC16；本地USB独立发送，不含ROS镜像前缀。 */
uint16_t Output_Capture_Packet(uint8_t *out)
{
    static const uint8_t magic[8] = {'H', '7', 'O', 'U', 'T', '0', '0', '1'};
    uint32_t primask;
    uint32_t header[6];
    uint16_t format[4];
    uint16_t size;
    uint16_t crc;

    primask = __get_PRIMASK();
    __disable_irq();
    prepared_read = read_index;
    prepared_count = write_index - read_index;
    header[0] = packet_sequence + 1u;
    header[1] = sequence;
    header[2] = dropped;
    header[3] = prepared_count;
    header[4] = high_water;
    header[5] = 0u;
    __set_PRIMASK(primask);
    if (prepared_count > OUTPUT_CAPTURE_BATCH)
    {
        prepared_count = OUTPUT_CAPTURE_BATCH;
    }
    format[0] = 1u;
    format[1] = OUTPUT_CAPTURE_RECORD_BYTES;
    format[2] = (uint16_t)prepared_count;
    format[3] = 0u;
    memcpy(out, magic, sizeof(magic));
    memcpy(out + 8u, format, sizeof(format));
    memcpy(out + 16u, header, sizeof(header));
    for (uint32_t i = 0u; i < prepared_count; i++)
    {
        memcpy(out + OUTPUT_CAPTURE_HEADER_BYTES + i * OUTPUT_CAPTURE_RECORD_BYTES, &records[(prepared_read + i) % OUTPUT_CAPTURE_CAPACITY], OUTPUT_CAPTURE_RECORD_BYTES);
    }
    size = (uint16_t)(OUTPUT_CAPTURE_HEADER_BYTES + prepared_count * OUTPUT_CAPTURE_RECORD_BYTES);
    crc = Capture_Crc16(out, size);
    memcpy(out + size, &crc, sizeof(crc));
    return (uint16_t)(size + sizeof(crc));
}

/* 只有USB接受整块静态缓冲后才释放队列；异步传输完成前由Vofa_Send保护缓冲。 */
void Output_Capture_Commit(void)
{
    __DMB();
    read_index = prepared_read + prepared_count;
    packet_sequence++;
    prepared_count = 0u;
}

#endif
