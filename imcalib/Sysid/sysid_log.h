#ifndef SYSID_LOG_H
#define SYSID_LOG_H

#include "../Sysid/sysid_config.h"

#if SYSID_ENABLE

#include "can_bus.h"
#include "mono_ns.h"
#include "dm.h"
#include "dji.h"
#include "leg_solver.h"
#include "machine_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 固定列宽 = 37 float, 帧长 152 B */
#define SYSID_LOG_FRAME_N   37u

/* 环形缓冲容量 */
#define SYSID_RING_CAP      128u
#define SYSID_RING_MASK     (SYSID_RING_CAP - 1u)

/* 行类型 */
#define SYSID_KIND_CMD_LEG      1u
#define SYSID_KIND_FB_LEG       2u
#define SYSID_KIND_CMD_WHEEL    3u
#define SYSID_KIND_FB_WHEEL     4u
#define SYSID_KIND_MARKER       5u

/* 标记事件 */
#define SYSID_EVENT_HEARTBEAT   0
#define SYSID_EVENT_RUN_START   (-1)
#define SYSID_EVENT_RUN_END     (-2)
#define SYSID_EVENT_ABORT       (-3)

/*
 * 33 列帧布局 (JustFloat, VOFA_UART @921600)
 *
 *  列  含义                   单位    来源
 *  0   kind                   -       行类型
 *  1   seq                    -       全局递增
 *  2   phase_or_event         -       正=phase, 负=event
 *  3   t_cmd_hi               -       leg tx_ns >> 20
 *  4   t_cmd_lo               -       leg tx_ns & 0xFFFFF
 *  5   tau_lf0                Nm      dm[0] 指令力矩(净: 预压+激励)
 *  6   tau_lf00               Nm      dm[1]
 *  7   tau_rf0                Nm      dm[2]
 *  8   tau_rf00               Nm      dm[3]
 *  9   pos_lf0                rad     dm_feedback[0] pos_zero_rad
 *  10  pos_lf00               rad     dm_feedback[1]
 *  11  pos_rf0                rad     dm_feedback[2]
 *  12  pos_rf00               rad     dm_feedback[3]
 *  13  vel_lf0                rad/s   dm_feedback[0] vel_rad_s
 *  14  vel_lf00               rad/s   dm_feedback[1]
 *  15  vel_rf0                rad/s   dm_feedback[2]
 *  16  vel_rf00               rad/s   dm_feedback[3]
 *  17  leg_length_L           m       leg_l.output.virtual_leg_length
 *  18  leg_pitch_L            rad     leg_l.output.virtual_leg_angle
 *  19  leg_length_R           m       leg_r
 *  20  leg_pitch_R            rad     leg_r
 *  21  t_rx_hi                -       whl rx_ns >> 20
 *  22  t_rx_lo                -       whl rx_ns & 0xFFFFF
 *  23  t_leg_hi               -       DM 反馈最近接收时刻 >> 20
 *  24  t_leg_lo               -       & 0xFFFFF
 *  25  d_len_L                m/s     leg_l d_virtual_leg_length
 *  26  d_pitch_L              rad/s   leg_l d_virtual_leg_angle
 *  27  d_len_R                m/s     leg_r
 *  28  d_pitch_R              rad/s   leg_r
 *  29  clamp_cnt              -       被限幅的电机数(0=没削平)
 *  30  preload_n              N       预压沿腿力(正=伸 负=收)
 *  31  whl_drop_cnt累         -       dji_bus tx_drop_cnt
 *  32  leg_drop_cnt累         -       dm_bus tx_drop_cnt
 *  33  thigh_tgt              rad     位置扫描: 大腿角目标
 *  34  shank_tgt              rad     位置扫描: 虚拟小腿角目标
 *
 * 行类型补充约定:
 *  列 5-8 在 kind=1 行 = 腿力矩 Nm; 在 kind=3 行 = 轮命令 raw
 *  kind=1 (腿行) 的列 23~28 = 腿的时间戳与解算导数(见上表), 29/30 = 削平/预压
 *  kind=3 (轮行) 的列 23~28 = 轮编码器/转速/电流, 29/30 = 0
 *  kind=3: 列 5 = 左轮命令 raw(逻辑值), 列 6 = 右轮命令 raw; 未激励的轮为 0
 *  kind=5: 列 5 = run_index, 列 6 = test_id, 列 7 = 总段数, 列 8 = 0
 *  kind=5 的列 3/4 为腿总线 TX 时刻, 仅作参考
 *
 * 心跳行 (kind=5, phase_or_event=0, 每 250ms 一行, 停机时也照推):
 *  列 5 = 当前 run_index        列 6 = 当前 test_id
 *  列 7 = 当前段号              列 8 = 状态码
 *       状态码: 0=正常 1=总输出关闭 2=DM离线 3=解算无效 4=表为空
 *  列 9 = 重入重置累计          列 10 = 环形缓冲丢帧累计
 *  列 11 = 串口忙跳过累计       列 12 = 心跳计数
 *  列 3/4 = 心跳推入时刻 (推入即取, 与其他行同源可比)
 */

/* 诊断计数 (sysid_log.c 定义, 心跳行读) */
extern volatile uint32_t sysid_log_drop_cnt;
extern volatile uint32_t sysid_log_busy_cnt;
extern volatile uint32_t sysid_log_stall_cnt;

/* 发送分频: commTask 1kHz ÷ 该值 (2=500Hz 3=333Hz 4=250Hz) */
/* 帧 136B 时 500Hz 占 921600 的 73.8%, 余量薄; 250Hz 只占 37% */
#ifndef SYSID_TX_DIV
#define SYSID_TX_DIV        4u
#endif

/* 串口卡死看门狗: 连续忙超该毫秒数则强制复位发送 */
#ifndef SYSID_TX_STALL_MS
#define SYSID_TX_STALL_MS   50u
#endif

/* 单行快照 (控制任务写, 通信任务读) */
typedef struct {
    uint8_t  kind;
    uint16_t seq;
    int16_t  phase_or_event;    /* 正=phase, 负=event */
    /* 腿 */
    float    tau_cmd[DM_MOTOR_NUM];
    float    pos_rad[DM_MOTOR_NUM];
    float    vel_rad_s[DM_MOTOR_NUM];
    float    leg_length[2];
    float    leg_pitch[2];
    uint64_t t_cmd_ns;
    /* 轮 */
    float    ecd_raw[DJI_MOTOR_NUM];
    float    speed_rpm[DJI_MOTOR_NUM];
    float    current_raw[DJI_MOTOR_NUM];
    uint64_t t_rx_ns;
    /* 腿行附加 (列 23~30) */
    uint64_t t_dm_rx_ns;        /* DM 反馈最近接收时刻 */
    float    d_leg[2];          /* 腿长速度 m/s */
    float    d_pitch[2];        /* 腿摆角速度 rad/s */
    float    preload_n;         /* 沿腿预压力 N */
    uint8_t  clamp_cnt;         /* 被限幅电机数 */
    /* 位置控制目标与实际 (列 33~36, 左腿实测) */
    float    pose_tgt[2];
    float    pose_now[2];
    /* 诊断 */
    uint32_t whl_drop_cnt;
    uint32_t leg_drop_cnt;
} sysid_snap_t;

/* 初始化 */
void Sysid_Log_Init(void);

/* 推入一帧快照 (控制任务调用); false = 缓冲满, 帧丢弃 */
bool Sysid_Log_Push(const sysid_snap_t *snap);

/* 发送泵 (通信任务调用, 每 2 个1kHz 周期发一次); 返回已发帧数 */
uint8_t Sysid_Log_Send_Pump(void);

#endif /* SYSID_ENABLE */
#endif /* SYSID_LOG_H */
