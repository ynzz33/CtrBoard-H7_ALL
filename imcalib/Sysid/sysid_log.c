#include "sysid_config.h"
#include "sysid_log.h"

#if SYSID_ENABLE

#include "dma_cache.h"
#include "Vofa_send.h"
#include <string.h>

/* JustFloat 帧尾 */
static const uint8_t sysid_tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* 环形缓冲 (SPSC: actuationTask 写 / commTask 读) */
static sysid_snap_t ring_buf[SYSID_RING_CAP];
static volatile uint32_t ring_w;
static volatile uint32_t ring_r;

/* 全局序列号 */
static uint16_t global_seq;

/* 发送泵分频 */
static uint8_t send_div;

/* 发送缓冲 (独立于 32ch 的 Vofa_Send 静态缓冲) */
static uint8_t tx_buf[SYSID_LOG_FRAME_N * 4u + 4u]
    __attribute__((aligned(DMA_CACHE_LINE_SIZE)));

/* 初始化 */
void Sysid_Log_Init(void)
{
    ring_w   = 0u;
    ring_r   = 0u;
    global_seq = 0u;
    send_div = 0u;
}

/* 推入快照 (控制任务调用), seq 自动分配 */
bool Sysid_Log_Push(const sysid_snap_t *snap)
{
    uint32_t next_w;

    if (snap == NULL)
    {
        return false;
    }

    next_w = ring_w + 1u;
    if (next_w - ring_r > SYSID_RING_CAP)
    {
        sysid_log_drop_cnt++;
        return false;
    }

    /* 自动分配 seq */
    ring_buf[ring_w & SYSID_RING_MASK] = *snap;
    ring_buf[ring_w & SYSID_RING_MASK].seq = global_seq++;
    __DMB();
    ring_w = next_w;
    return true;
}

/* 拆分 ns → t_hi / t_lo (float32 可精确表示) */
static void split_ns(uint64_t ns, float *t_hi, float *t_lo)
{
    *t_hi = (float)(uint32_t)(ns >> 20);
    *t_lo = (float)(uint32_t)(ns & 0xFFFFFu);
}

/* 组装一帧到 tx_buf */
static uint16_t assemble_frame(const sysid_snap_t *s)
{
    float f[SYSID_LOG_FRAME_N];
    uint16_t len;

    f[0]  = (float)s->kind;
    f[1]  = (float)s->seq;
    f[2]  = (float)s->phase_or_event;
    split_ns(s->t_cmd_ns, &f[3], &f[4]);
    f[5]  = s->tau_cmd[0];
    f[6]  = s->tau_cmd[1];
    f[7]  = s->tau_cmd[2];
    f[8]  = s->tau_cmd[3];
    f[9]  = s->pos_rad[0];
    f[10] = s->pos_rad[1];
    f[11] = s->pos_rad[2];
    f[12] = s->pos_rad[3];
    f[13] = s->vel_rad_s[0];
    f[14] = s->vel_rad_s[1];
    f[15] = s->vel_rad_s[2];
    f[16] = s->vel_rad_s[3];
    f[17] = s->leg_length[0];
    f[18] = s->leg_pitch[0];
    f[19] = s->leg_length[1];
    f[20] = s->leg_pitch[1];
    split_ns(s->t_rx_ns, &f[21], &f[22]);
    if (s->kind == SYSID_KIND_CMD_LEG)
    {
        /* 腿行: 23~30 = DM 接收时刻 / 腿解算导数 / 削平 / 预压力 */
        split_ns(s->t_dm_rx_ns, &f[23], &f[24]);
        f[25] = s->d_leg[0];
        f[26] = s->d_pitch[0];
        f[27] = s->d_leg[1];
        f[28] = s->d_pitch[1];
        f[29] = (float)s->clamp_cnt;
        f[30] = s->preload_n;
    }
    else
    {
        f[23] = s->ecd_raw[0];
        f[24] = s->ecd_raw[1];
        f[25] = s->speed_rpm[0];
        f[26] = s->speed_rpm[1];
        f[27] = s->current_raw[0];
        f[28] = s->current_raw[1];
        f[29] = 0.0f;
        f[30] = 0.0f;
    }
    f[31] = (float)s->whl_drop_cnt;
    f[32] = (float)s->leg_drop_cnt;
    f[33] = s->pose_tgt[0];
    f[34] = s->pose_tgt[1];
    f[35] = s->pose_now[0];
    f[36] = s->pose_now[1];

    len = SYSID_LOG_FRAME_N * 4u;
    memcpy(tx_buf, f, len);
    memcpy(tx_buf + len, sysid_tail, 4u);
    return len + 4u;
}

/* 诊断计数 (sysid_log.h 里 extern) */
volatile uint32_t sysid_log_drop_cnt;
volatile uint32_t sysid_log_busy_cnt;
volatile uint32_t sysid_log_stall_cnt;

/* 连续忙起始时刻 (看门狗用) */
static uint32_t sysid_busy_since;

/* 发送泵 (通信任务调用) */
uint8_t Sysid_Log_Send_Pump(void)
{
    sysid_snap_t snap;
    uint16_t frame_len;
    uint8_t sent;

    send_div++;
    if (send_div < SYSID_TX_DIV)
    {
        return 0u;
    }
    send_div = 0u;

    sent = 0u;

    /* 发送状态卡死看门狗: HAL 丢 TC 中断会永远 BUSY */
    if ((VOFA_UART)->gState != HAL_UART_STATE_READY)
    {
        if (sysid_busy_since == 0u)
        {
            sysid_busy_since = HAL_GetTick();
        }
        else if ((HAL_GetTick() - sysid_busy_since) >= SYSID_TX_STALL_MS)
        {
            (void)HAL_UART_AbortTransmit(VOFA_UART);
            sysid_log_stall_cnt++;
            sysid_busy_since = 0u;
        }
        sysid_log_busy_cnt++;
        return 0u;
    }
    sysid_busy_since = 0u;

    /* 每周期最多取 1 帧 */
    __DMB();
    if (ring_r == ring_w)
    {
        return 0u;
    }

    snap = ring_buf[ring_r & SYSID_RING_MASK];
    __DMB();
    ring_r++;

    frame_len = assemble_frame(&snap);
    Dma_Cache_Clean_Tx(tx_buf, frame_len);
    HAL_UART_Transmit_DMA(VOFA_UART, tx_buf, frame_len);
    sent = 1u;

    return sent;
}

#endif /* SYSID_ENABLE */
