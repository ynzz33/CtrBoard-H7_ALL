/*
 * UART10 Jetson链路：100Hz只读镜像和14字节CRC探测包。
 * 控制、VOFA、动作状态及物理标定均不在本文件修改。
 */
#include "jetson_uart.h"
#include "machine_config.h"
#include "usart.h"
#include "dma_cache.h"
#include "feedback_snapshot.h"
#include "robot_state.h"
#include "dm.h"
#include "control_compute.h"
#include "mono_ns.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define JETSON_UART_DMA_BYTES 256u

jetson_uart_status_t jetson_uart_status;

#if MACHINE_JETSON_PORT == 10u
#define JETSON_UART (&huart10)
static uint8_t rx_dma[JETSON_UART_DMA_BYTES] __attribute__((aligned(DMA_CACHE_LINE_SIZE)));
static uint8_t tx_dma[JETSON_MIRROR_CH * sizeof(float) + 4u] __attribute__((aligned(DMA_CACHE_LINE_SIZE)));
static float mirror_frame[JETSON_MIRROR_CH];
static uint8_t packet[JETSON_UART_PACKET_BYTES];
static uint16_t read_pos;
static uint8_t packet_size;
static const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

static uint16_t Jetson_Crc16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < size; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t Jetson_U32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void Jetson_Mirror_U32(float *out, uint32_t value)
{
    out[0] = (float)(value & 0xFFFFu);
    out[1] = (float)(value >> 16);
}

static float Jetson_Publication_Age(const feedback_meta_t *meta, uint64_t captured_ns)
{
    uint64_t age_ms;
    if (!meta->seen) { return -1.0f; }
    age_ms = (captured_ns - meta->published_ns) / 1000000u;
    return age_ms > 60000u ? 60000.0f : (float)age_ms;
}

static void Jetson_Receive_Byte(uint8_t value)
{
    uint16_t expected;

    packet[packet_size++] = value;
    while (packet_size >= 4u && memcmp(packet, "J7P1", 4u) != 0)
    {
        memmove(packet, packet + 1u, --packet_size);
    }
    if (packet_size != JETSON_UART_PACKET_BYTES) { return; }
    expected = (uint16_t)packet[12] | ((uint16_t)packet[13] << 8);
    if (Jetson_Crc16(packet, 12u) == expected)
    {
        jetson_uart_status.sequence = Jetson_U32(packet + 4u);
        jetson_uart_status.token = Jetson_U32(packet + 8u);
        jetson_uart_status.received++;
        packet_size = 0u;
    }
    else
    {
        jetson_uart_status.crc_errors++;
        memmove(packet, packet + 1u, --packet_size);
    }
}

static uint8_t Jetson_Send(const float data[JETSON_MIRROR_CH])
{
    if (JETSON_UART->gState != HAL_UART_STATE_READY) { return 0u; }
    memcpy(tx_dma, data, JETSON_MIRROR_CH * sizeof(float));
    memcpy(tx_dma + JETSON_MIRROR_CH * sizeof(float), tail, sizeof(tail));
    Dma_Cache_Clean_Tx(tx_dma, sizeof(tx_dma));
    return (uint8_t)(HAL_UART_Transmit_DMA(JETSON_UART, tx_dma, sizeof(tx_dma)) == HAL_OK);
}

void Jetson_Uart_Init(void)
{
    read_pos = 0u;
    packet_size = 0u;
    Dma_Cache_Invalidate_Rx(rx_dma, sizeof(rx_dma));
    if (HAL_UART_Receive_DMA(JETSON_UART, rx_dma, sizeof(rx_dma)) == HAL_OK)
    {
        /* 保留TC，HAL处理错误后的DMA异步终止依赖它释放接收状态。 */
        __HAL_DMA_DISABLE_IT(JETSON_UART->hdmarx, DMA_IT_HT);
    }
}

void Jetson_Uart_Process(void)
{
    uint16_t write_pos;

    if (JETSON_UART->RxState != HAL_UART_STATE_BUSY_RX)
    {
        jetson_uart_status.rx_restarts++;
        Jetson_Uart_Init();
        return;
    }
    write_pos = (uint16_t)(JETSON_UART_DMA_BYTES - __HAL_DMA_GET_COUNTER(JETSON_UART->hdmarx));
    write_pos %= JETSON_UART_DMA_BYTES;
    if (read_pos == write_pos) { return; }
    Dma_Cache_Invalidate_Rx(rx_dma, sizeof(rx_dma));
    while (read_pos != write_pos)
    {
        Jetson_Receive_Byte(rx_dma[read_pos]);
        read_pos = (uint16_t)((read_pos + 1u) % JETSON_UART_DMA_BYTES);
    }
}

void Jetson_Mirror_Send(void)
{
    static robot_feedback_t feedback;
    static uint32_t last_attempt_ms;
    static uint32_t sequence;
    float *frame = mirror_frame;
    uint32_t now_ms = HAL_GetTick();
    uint32_t candidate = sequence + 1u;
    uint16_t state_bits;
    uint8_t online_mask;
    uint8_t valid_mask = 0u;

    if ((uint32_t)(now_ms - last_attempt_ms) < 10u) { return; }
    last_attempt_ms = now_ms;
    (void)Robot_Feedback_Read(&feedback);
    memset(frame, 0, sizeof(mirror_frame));
    online_mask = feedback.imu.online ? 0x01u : 0u;
    online_mask |= feedback.rc.online ? 0x02u : 0u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        online_mask |= feedback.motor.dm.online[i] ? (uint8_t)(1u << (i + 2u)) : 0u;
        if (feedback.motor.dm.online[i] && isfinite(feedback.motor.dm.pos_continuous_rad[i]) && isfinite(feedback.motor.dm.vel_rad_s[i]))
        {
            frame[10u + i] = feedback.motor.dm.pos_continuous_rad[i];
            frame[14u + i] = feedback.motor.dm.vel_rad_s[i];
            valid_mask |= (uint8_t)(1u << (i + 1u));
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        online_mask |= feedback.motor.dji.online[i] ? (uint8_t)(1u << (i + 6u)) : 0u;
        if (feedback.motor.dji.online[i] && isfinite(feedback.motor.dji.angle_total_rad[i]) && isfinite(feedback.motor.dji.vel_rad_s[i]))
        {
            frame[18u + i] = feedback.motor.dji.angle_total_rad[i];
            frame[20u + i] = feedback.motor.dji.vel_rad_s[i];
            valid_mask |= (uint8_t)(1u << (i + 5u));
        }
    }
    if (feedback.imu.online)
    {
        valid_mask |= 0x01u;
        for (uint8_t i = 0u; i < 4u; i++) { frame[22u + i] = feedback.imu.quat[i]; }
        for (uint8_t i = 0u; i < 3u; i++) { frame[26u + i] = feedback.imu.gyro_rad_s[i]; }
    }
    taskENTER_CRITICAL();
    state_bits = robot_state.motor_enabled ? 0x01u : 0u;
    state_bits |= robot_state.fallen ? 0x02u : 0u;
    state_bits |= feedback.left.output.valid ? 0x04u : 0u;
    state_bits |= feedback.right.output.valid ? 0x08u : 0u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_LFT) ? 0x10u : 0u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_LFT) ? 0x20u : 0u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_RGT) ? 0x40u : 0u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_RGT) ? 0x80u : 0u;
    state_bits |= Control_Compute_LQR_Engaged() ? 0x100u : 0u;
    state_bits |= Control_Compute_RL_Engaged() ? 0x200u : 0u;
    frame[2] = (float)ctrl_fault;
    frame[32] = (float)JETSON_UART_PROTOCOL;
    Jetson_Mirror_U32(&frame[33], jetson_uart_status.sequence);
    Jetson_Mirror_U32(&frame[35], jetson_uart_status.token);
    Jetson_Mirror_U32(&frame[37], jetson_uart_status.received);
    taskEXIT_CRITICAL();
    frame[0] = (float)online_mask;
    frame[1] = (float)state_bits;
    frame[3] = 7101.0f;
    frame[4] = 1.0f;
    frame[5] = (float)MACHINE_DEFAULT;
    Jetson_Mirror_U32(&frame[6], candidate);
    Jetson_Mirror_U32(&frame[8], (uint32_t)(feedback.captured_ns / 1000000u));
    frame[29] = (float)valid_mask;
    frame[30] = Jetson_Publication_Age(&feedback.imu_meta, feedback.captured_ns);
    frame[31] = Jetson_Publication_Age(&feedback.motor_meta, feedback.captured_ns);
    frame[47] = (float)Jetson_Crc16((const uint8_t *)frame, 47u * sizeof(float));
    if (Jetson_Send(frame)) { sequence = candidate; }
}
#else
void Jetson_Uart_Init(void) {}
void Jetson_Uart_Process(void) {}
void Jetson_Mirror_Send(void) {}
#endif
