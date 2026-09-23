#include "sysid_config.h"
#include "sysid_mode.h"

#if SYSID_ENABLE

#include "sysid_log.h"
#include "robot_control.h"
#include "machine_config.h"
#include "dr16.h"
#include "simple-function.h"
#include <math.h>
#include <string.h>

/* ========= 安全限幅 (文件顶部, 易改) ========= */
#ifndef SYSID_TRQ_LIMIT_NM
#define SYSID_TRQ_LIMIT_NM      10.0f   /* 腿力矩测试限幅 Nm (手动模式是 20) */
#endif
#ifndef SYSID_CURRENT_LIMIT_RAW
#define SYSID_CURRENT_LIMIT_RAW 4096    /* 轮电流限幅 raw, ±5A */
#endif
#define SYSID_RAW_PER_A         819.2f  /* C620 raw/A */
#define SYSID_TEMP_LIMIT_C      80u     /* 温度门限, 当前未启用 */
#define SYSID_DT                CTRL_DT
#define SYSID_REENTRY_MS        10u     /* 重入判定 */

/* 预压: 沿腿的常值力 N (负=收腿), 把腿压到行程中段; 0=不加 */
/* 换算: 单髋力矩 ≈ 沿腿力 × 0.138 (43N≈6Nm, 51N≈7Nm, 54N≈7.5Nm, 58N≈8Nm) */
/* 雅可比随姿态变, 所以实际力矩以帧列 23~26 显示为准 */
/* 预压 + 激励 之和不得超过上面限幅, 否则被削平 */
#ifndef SYSID_PRELOAD_L_N
#define SYSID_PRELOAD_L_N       (-58.0f)    /* 作者定: 8Nm/髋 */
#endif
#ifndef SYSID_PRELOAD_R_N
#define SYSID_PRELOAD_R_N       (-58.0f)    /* 作者定: 8Nm/髋 */
#endif

/* 实时预压调节: 1=遥控滚轮当预压旋钮(左右同值), 0=用上面两个固定宏 */
#ifndef SYSID_PRELOAD_TUNE
#define SYSID_PRELOAD_TUNE      0
#endif
#define SYSID_PRELOAD_TUNE_N    150.0f  /* 旋钮到底 = 沿腿 150N (≈21Nm/髋) */

/* 两条腿同时激励: 1=激励某一路时, 对侧腿的对应电机给同值 (镜像同向) */
#ifndef SYSID_EXCITE_BOTH
#define SYSID_EXCITE_BOTH       1
#endif

/* ========= 测试方式 (作者 2026-09-18 定) ========= */
/* 0 = 位置扫描: 控制大腿角/虚拟小腿角, 记录该姿态下需要多少力矩 (准静态) */
/* 1 = 力矩激励: 直接下发力矩, 记录角度响应 (动态辨识, 原方案) */
/* 手动 PD 手测不用这里: 左拨杆上位(不动右拨杆)=正常手动模式, 同一套力矩链路+摇杆偏移 */
#define SYSID_MODE_POSE     0
#define SYSID_MODE_TORQUE   1
#ifndef SYSID_MODE
#define SYSID_MODE          SYSID_MODE_POSE    /* 当前: 髋位置扫描 (测轮时改 _TORQUE) */
#endif

/* 姿态表跑几遍后停 (0 = 一直循环) */
#ifndef SYSID_LOOP_CNT
#define SYSID_LOOP_CNT      1u
#endif

/* 位置扫描参数 (文件顶部, 易改) */
#define SYSID_POSE_RAMP_RATE   1.2f    /* 目标角最大斜率 rad/s (斜坡在 simple-function 里) */
#define SYSID_POSE_HOLD_S   3.0f    /* 到位后保持 s (准静态取数) */
#define SYSID_POSE_KP       10.0f   /* 虚拟关节 P (待台架) */
#define SYSID_POSE_KD       0.0f    /* 虚拟关节 D (作者定: 去掉) */

/* ========= 计划选择: 1=仅腿 2=仅轮 3=全部 ========= */
#ifndef SYSID_PLAN
#define SYSID_PLAN  1
#endif

/* ========= 模板类型 ========= */
enum {
    TPL_STEP = 0,
    TPL_CHIRP,
    TPL_STICTION,
    TPL_BASELINE_SIGN,
    TPL_POSE,           /* 位置扫描: 斜坡到位后保持 */
};

/* ========= 测试 ID ========= */
enum {
    TID_BASELINE = 0,
    TID_STEP,
    TID_CHIRP,
    TID_HOLDOUT_CHIRP,
    TID_WHL_BASELINE_SIGN,
    TID_WHL_STICTION,
    TID_WHL_PLATEAU,
    TID_WHL_STEP,
    TID_WHL_HOLDOUT_STEP,
    TID_POSE,
};

/* ========= 阶梯表 (stiction 用, 17级, 每级1s) ========= */
/* ========= 阶梯表 (stiction 用, 6级, 每级1s, 最高4A) ========= */
static const float stiction_levels[6] = {
    0.0f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f
};
#define STICTION_LEVELS  6u

/* ========= Run 描述 ========= */
typedef struct {
    uint8_t  test_id;
    uint8_t  kind;       /* 1=leg 3=wheel */
    uint8_t  target;     /* leg 0-3 / wheel 0-1 */
    uint8_t  tpl;        /* 模板 */
    float    amplitude;  /* Nm(leg) A(wheel) */
    float    t_pre;      /* 前置零 s */
    float    t_excite;   /* 激励段 s */
    float    t_post;     /* 后置零 s */
    uint8_t  reps;       /* 重复次数 */
    float    freq_lo;    /* 扫频起始 Hz */
    float    freq_hi;    /* 扫频终止 Hz */
    float    amp2;       /* 位置扫描: 虚拟小腿角目标 rad */
} sysid_run_t;

/* ========= Run 表 ========= */
#define STEP_L(m, a) \
    { TID_STEP, SYSID_KIND_CMD_LEG, m, TPL_STEP, \
      a, 2.0f, 0.5f, 3.0f, 3, 0, 0 }
#define CHIRP_L(m, amp, tid) \
    { tid, SYSID_KIND_CMD_LEG, m, TPL_CHIRP, \
      amp, 2.0f, 10.0f, 3.0f, 1, 0.2f, 10.0f }
#define PLAT_W(a) \
    { TID_WHL_PLATEAU, SYSID_KIND_CMD_WHEEL, 0, TPL_STEP, \
      a, 0.5f, 1.0f, 0.5f, 1, 0, 0 }
#define STEP_W(a) \
    { TID_WHL_STEP, SYSID_KIND_CMD_WHEEL, 0, TPL_STEP, \
      a, 0.5f, 0.5f, 0.5f, 1, 0, 0 }
#define HSTEP_W(a) \
    { TID_WHL_HOLDOUT_STEP, SYSID_KIND_CMD_WHEEL, 0, TPL_STEP, \
      a, 0.5f, 0.5f, 0.5f, 1, 0, 0 }

#if SYSID_MODE == SYSID_MODE_TORQUE
static const sysid_run_t sysid_runs[] = {

#if SYSID_PLAN == 1 || SYSID_PLAN == 3
    /* 腿力矩 baseline: 全零 5s */
    { TID_BASELINE, SYSID_KIND_CMD_LEG, 0, TPL_STEP,
      0.0f, 5.0f, 0.0f, 0.0f, 1, 0, 0 },

    /* torque_step: 4路 × 3幅值(±2/±4/±6) × 2极性 */
    STEP_L(0,  2.0f), STEP_L(0, -2.0f),
    STEP_L(0,  4.0f), STEP_L(0, -4.0f),
    STEP_L(0,  6.0f), STEP_L(0, -6.0f),
    STEP_L(1,  2.0f), STEP_L(1, -2.0f),
    STEP_L(1,  4.0f), STEP_L(1, -4.0f),
    STEP_L(1,  6.0f), STEP_L(1, -6.0f),
    STEP_L(2,  2.0f), STEP_L(2, -2.0f),
    STEP_L(2,  4.0f), STEP_L(2, -4.0f),
    STEP_L(2,  6.0f), STEP_L(2, -6.0f),
    STEP_L(3,  2.0f), STEP_L(3, -2.0f),
    STEP_L(3,  4.0f), STEP_L(3, -4.0f),
    STEP_L(3,  6.0f), STEP_L(3, -6.0f),

    /* torque_chirp: 4路各4.0Nm */
    CHIRP_L(0, 4.0f, TID_CHIRP),
    CHIRP_L(1, 4.0f, TID_CHIRP),
    CHIRP_L(2, 4.0f, TID_CHIRP),
    CHIRP_L(3, 4.0f, TID_CHIRP),

    /* holdout_chirp: 4路各5.0Nm */
    CHIRP_L(0, 5.0f, TID_HOLDOUT_CHIRP),
    CHIRP_L(1, 5.0f, TID_HOLDOUT_CHIRP),
    CHIRP_L(2, 5.0f, TID_HOLDOUT_CHIRP),
    CHIRP_L(3, 5.0f, TID_HOLDOUT_CHIRP),
#endif

#if SYSID_PLAN == 2 || SYSID_PLAN == 3
    /* plateau: ±0.5~±4A (左轮) 稳态 */
    PLAT_W( 0.5f), PLAT_W(-0.5f),
    PLAT_W( 1.0f), PLAT_W(-1.0f),
    PLAT_W( 2.0f), PLAT_W(-2.0f),
    PLAT_W( 3.0f), PLAT_W(-3.0f),
    PLAT_W( 4.0f), PLAT_W(-4.0f),

#endif
};

#endif

#if SYSID_MODE == SYSID_MODE_POSE
/* ========= 位置扫描表: {大腿角, 虚拟小腿角} rad ========= */
/* 两条腿同时走同一目标; 角度到不了(撞限位)的姿态数据判无效 */
/* t_pre = 斜坡段 4.2s (大腿最大跨度 90° @0.4rad/s = 3.93s, 留余量) */
#define POSE_RUN(th, sh) \
    { TID_POSE, SYSID_KIND_CMD_LEG, 0, TPL_POSE, \
      th, 4.2f, SYSID_POSE_HOLD_S, 0.0f, 1, 0.0f, 0.0f, sh }
static const sysid_run_t sysid_pose_runs[] = {
    /* 大腿角 45°/90°/135° = 0.7854/1.5708/2.3562 rad; 虚拟小腿角 2.40/2.60/2.80 rad */
    POSE_RUN(0.7854f, 2.40f), POSE_RUN(1.5708f, 2.40f), POSE_RUN(2.3562f, 2.40f),
    POSE_RUN(0.7854f, 2.60f), POSE_RUN(1.5708f, 2.60f), POSE_RUN(2.3562f, 2.60f),
    POSE_RUN(0.7854f, 2.80f), POSE_RUN(1.5708f, 2.80f), POSE_RUN(2.3562f, 2.80f),
};
#define SYSID_RUNS     sysid_pose_runs
#else
#define SYSID_RUNS     sysid_runs
#endif

#define SYSID_RUN_CNT  (sizeof(SYSID_RUNS) / sizeof(SYSID_RUNS[0]))

/* ========= 状态变量 ========= */
static uint16_t run_idx;
static uint32_t tick_in_run;
static uint8_t  run_active;
static uint8_t  stopped;
static uint8_t  stop_code;      /* 停机状态码 */
static uint32_t last_call_tick;
static uint32_t reinit_cnt;     /* 重入重置累计 */
static uint32_t hb_tick;        /* 心跳节拍 */
static uint32_t hb_cnt;         /* 心跳计数 */
static uint32_t loop_cnt;       /* 姿态表已跑遍数 */
#if SYSID_MODE == SYSID_MODE_TORQUE
static float    sysid_pre[DM_MOTOR_NUM];   /* 本周期预压分量 */
#endif

/* 轮测试当前命令电流 raw (逻辑值), 给 VOFA 显示 */
volatile int16_t sysid_wheel_cmd_raw[DJI_MOTOR_NUM];

/* 当前测试方式是否是"轮" (给 VOFA 选帧用): 1=轮 0=髋位置扫描 */
const uint8_t sysid_is_wheel_mode = (SYSID_MODE == SYSID_MODE_TORQUE) ? 1u : 0u;

/* 位置控制 (位置扫描/手动): 虚拟关节 PD 与上一姿态目标 */
#if SYSID_MODE != SYSID_MODE_TORQUE
static rl_torque_param_t sysid_pose_param;
static rl_torque_state_t sysid_pose_state;
static ramp_t   sysid_ramp_th = {0.0f, 0.4f};   /* 大腿角斜坡, 斜率 0.4 rad/s */
static ramp_t   sysid_ramp_sh = {0.0f, 0.2f};   /* 虚拟小腿角斜坡, 0.2 rad/s */
static uint8_t  sysid_pose_ready;
#endif

/* ========= 辅助函数 ========= */

static float sysid_clamp_f(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static int16_t sysid_to_raw(float amp_a)
{
    float raw_f = amp_a * SYSID_RAW_PER_A;
    raw_f = sysid_clamp_f(raw_f,
        -(float)SYSID_CURRENT_LIMIT_RAW,
        (float)SYSID_CURRENT_LIMIT_RAW);
    return (int16_t)raw_f;
}

/* run 总时长 */
static float sysid_run_dur(const sysid_run_t *r)
{
    float pd;
    switch (r->tpl)
    {
    case TPL_STEP:
        pd = r->t_pre + r->t_excite + r->t_post;
        return (float)r->reps * pd;
    case TPL_CHIRP:
        return r->t_pre + r->t_excite + r->t_post;
    case TPL_STICTION:
        return (float)STICTION_LEVELS;
    case TPL_BASELINE_SIGN:
        return 14.0f;
    case TPL_POSE:
        return r->t_pre + r->t_excite;      /* 斜坡 + 保持 */
    default:
        return 0.0f;
    }
}

/* 总段数 (标记行用) */
static uint8_t sysid_phases(const sysid_run_t *r)
{
    switch (r->tpl)
    {
    case TPL_STEP:
        return (uint8_t)((uint32_t)r->reps * 3u);
    case TPL_CHIRP:
        return 3u;
    case TPL_STICTION:
        return (uint8_t)STICTION_LEVELS;
    case TPL_BASELINE_SIGN:
        return 5u;
    default:
        return 0u;
    }
}

/* 当前段号 (1起) */
static int16_t sysid_phase_num(const sysid_run_t *r, float t)
{
    float pd;
    uint32_t rep, sub, lv;
    switch (r->tpl)
    {
    case TPL_STEP:
        pd = r->t_pre + r->t_excite + r->t_post;
        if (pd <= 0.0f) { return 1; }
        rep = (uint32_t)(t / pd);
        {
            float tir = t - (float)rep * pd;
            if (tir < r->t_pre) { sub = 0u; }
            else if (tir < r->t_pre + r->t_excite) { sub = 1u; }
            else { sub = 2u; }
        }
        return (int16_t)(rep * 3u + sub + 1u);
    case TPL_CHIRP:
        if (t < r->t_pre) { return 1; }
        if (t < r->t_pre + r->t_excite) { return 2; }
        return 3;
    case TPL_STICTION:
        lv = (uint32_t)t;
        if (lv >= STICTION_LEVELS) { return (int16_t)STICTION_LEVELS; }
        return (int16_t)(lv + 1u);
    case TPL_BASELINE_SIGN:
        if (t < 5.0f) { return 1; }
        if (t < 6.0f) { return 2; }
        if (t < 8.0f) { return 3; }
        if (t < 9.0f) { return 4; }
        return 5;
    case TPL_POSE:
        return (t < r->t_pre) ? 1 : 2;      /* 1=斜坡 2=保持 */
    default:
        return 1;
    }
}

/* 计算目标值 */
static float sysid_target(const sysid_run_t *r, float t)
{
    float pd, tc, T, phi;
    uint32_t lv;
    float sgn;
    switch (r->tpl)
    {
    case TPL_STEP:
        pd = r->t_pre + r->t_excite + r->t_post;
        if (pd <= 0.0f) { return 0.0f; }
        {
            float tir = t - floorf(t / pd) * pd;
            if (tir < r->t_pre) { return 0.0f; }
            if (tir < r->t_pre + r->t_excite) { return r->amplitude; }
            return 0.0f;
        }
    case TPL_CHIRP:
        if (t < r->t_pre) { return 0.0f; }
        tc = t - r->t_pre;
        T = r->t_excite;
        if (tc >= T) { return 0.0f; }
        phi = 6.2831853f * (r->freq_lo * tc
            + (r->freq_hi - r->freq_lo) * tc * tc / (2.0f * T));
        return r->amplitude * sinf(phi);
    case TPL_STICTION:
        lv = (uint32_t)t;
        if (lv >= STICTION_LEVELS) { return 0.0f; }
        sgn = (r->amplitude >= 0.0f) ? 1.0f : -1.0f;
        return stiction_levels[lv] * sgn;
    case TPL_BASELINE_SIGN:
        if (t < 5.0f) { return 0.0f; }
        if (t < 6.0f) { return r->amplitude; }
        if (t < 8.0f) { return 0.0f; }
        if (t < 9.0f) { return -r->amplitude; }
        return 0.0f;
    default:
        return 0.0f;
    }
}

/* 填充反馈字段 */
static void sysid_fill_fb(sysid_snap_t *s)
{
    uint8_t i;
    uint64_t rx_new;

    rx_new = 0u;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        s->pos_rad[i]   = dm_motor_feedback[i].pos_zero_rad;
        s->vel_rad_s[i]  = dm_motor_feedback[i].vel_rad_s;
        if (dm_motor_feedback[i].rx_ns > rx_new)
        {
            rx_new = dm_motor_feedback[i].rx_ns;
        }
    }
    s->t_dm_rx_ns    = rx_new;
    s->leg_length[0] = leg_l.output.virtual_leg_length;
    s->leg_pitch[0]  = leg_l.output.virtual_leg_angle;
    s->leg_length[1] = leg_r.output.virtual_leg_length;
    s->leg_pitch[1]  = leg_r.output.virtual_leg_angle;
    s->d_leg[0]      = leg_l.output.d_virtual_leg_length;
    s->d_pitch[0]    = leg_l.output.d_virtual_leg_angle;
    s->d_leg[1]      = leg_r.output.d_virtual_leg_length;
    s->d_pitch[1]    = leg_r.output.d_virtual_leg_angle;
    s->pose_now[0]   = leg_l.output.thigh_angle;          /* 实测大腿角 */
    s->pose_now[1]   = leg_l.output.virtual_shank_angle;  /* 实测虚拟小腿角 */
    s->ecd_raw[0]     = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].angle_raw;
    s->ecd_raw[1]     = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].angle_raw;
    s->speed_rpm[0]   = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].vel_raw;
    s->speed_rpm[1]   = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].vel_raw;
    s->current_raw[0] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].current_raw;
    s->current_raw[1] = (float)dji_motor_feedback[DJI_MOTOR_WHEEL_RGT].current_raw;
    s->whl_drop_cnt   = Can_Bus_Tx_Drop_Count((uint8_t)machine->dji_bus);
    s->leg_drop_cnt   = Can_Bus_Tx_Drop_Count((uint8_t)machine->dm_bus[0]);
}

/* 推标记行 */
static void sysid_push_marker(int16_t event, uint64_t cmd_ns, uint64_t rx_ns)
{
    sysid_snap_t s;
    memset(&s, 0, sizeof(s));
    s.kind           = SYSID_KIND_MARKER;
    s.phase_or_event = event;
    s.t_cmd_ns       = cmd_ns;
    s.t_rx_ns        = rx_ns;
    sysid_fill_fb(&s);
    (void)Sysid_Log_Push(&s);
}

/* 推 run 开始标记 */
static void sysid_push_start(uint16_t idx, const sysid_run_t *r,
                              uint64_t cmd_ns, uint64_t rx_ns)
{
    sysid_snap_t s;
    memset(&s, 0, sizeof(s));
    s.kind           = SYSID_KIND_MARKER;
    s.phase_or_event = SYSID_EVENT_RUN_START;
    s.tau_cmd[0]     = (float)idx;
    s.tau_cmd[1]     = (float)r->test_id;
    s.tau_cmd[2]     = (float)sysid_phases(r);
    s.tau_cmd[3]     = 0.0f;
    s.t_cmd_ns       = cmd_ns;
    s.t_rx_ns        = rx_ns;
    sysid_fill_fb(&s);
    (void)Sysid_Log_Push(&s);
}

/* 取两路 TX 完成时间戳 */
static void sysid_pop_tx(uint64_t *leg_tx, uint64_t *whl_tx)
{
    uint64_t tx_ns;
    uint8_t k;
    uint16_t sq;
    *leg_tx = 0u;
    *whl_tx = 0u;
    while (Can_Bus_Tx_Pop((uint8_t)machine->dm_bus[0], &k, &sq, &tx_ns))
    {
        if (tx_ns > *leg_tx) { *leg_tx = tx_ns; }
    }
    while (Can_Bus_Tx_Pop((uint8_t)machine->dji_bus, &k, &sq, &tx_ns))
    {
        if (tx_ns > *whl_tx) { *whl_tx = tx_ns; }
    }
}

/* 检查硬故障: 返回状态码, 0=正常 */
#define SYSID_ST_OK         0u
#define SYSID_ST_OUT_OFF    1u
#define SYSID_ST_DM_OFF     2u
#define SYSID_ST_SOLVER     3u
#define SYSID_ST_EMPTY      4u
#define SYSID_ST_DONE       5u      /* 姿态表已跑完设定遍数 */

/* 检查硬故障: 返回状态码, 0=正常 */
static uint8_t sysid_fault(void)
{
    uint8_t i;
    if (!torque_output_enabled) { return SYSID_ST_OUT_OFF; }
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!Dm_Is_Online(i)) { return SYSID_ST_DM_OFF; }
    }
    if (!leg_l.output.valid || !leg_r.output.valid) { return SYSID_ST_SOLVER; }
    return SYSID_ST_OK;
}

/* 心跳周期 (1kHz × 250 = 250ms) */
#define SYSID_HB_TICKS  250u

/* 推心跳行 (kind=5, 事件 0): 250ms 一行, 停机时也推, 保证流不断 */
static void sysid_push_heartbeat(uint64_t cmd_ns, uint64_t rx_ns)
{
    sysid_snap_t s;
    uint8_t code;
    float phase;

    code = stopped ? stop_code : SYSID_ST_OK;
    if (SYSID_RUN_CNT == 0u)
    {
        code = SYSID_ST_EMPTY;
    }
    phase = 0.0f;
    if (run_active && run_idx < SYSID_RUN_CNT)
    {
        phase = (float)sysid_phase_num(&SYSID_RUNS[run_idx],
            (float)tick_in_run * SYSID_DT);
    }

    memset(&s, 0, sizeof(s));
    s.kind           = SYSID_KIND_MARKER;
    s.phase_or_event = SYSID_EVENT_HEARTBEAT;
    s.tau_cmd[0]     = (float)run_idx;
    s.tau_cmd[1]     = (run_idx < SYSID_RUN_CNT)
                       ? (float)SYSID_RUNS[run_idx].test_id : 0.0f;
    s.tau_cmd[2]     = phase;
    s.tau_cmd[3]     = (float)code;
    s.t_cmd_ns       = cmd_ns;
    s.t_rx_ns        = rx_ns;
    sysid_fill_fb(&s);
    /* 诊断位 (心跳行专用) */
    s.pos_rad[0] = (float)reinit_cnt;
    s.pos_rad[1] = (float)sysid_log_drop_cnt;
    s.pos_rad[2] = (float)sysid_log_busy_cnt;
    s.pos_rad[3] = (float)hb_cnt;
    s.vel_rad_s[0] = (float)sysid_log_stall_cnt;    /* 列13: 串口卡死自恢复次数 */
    (void)Sysid_Log_Push(&s);
    hb_cnt++;
}

/* ========= 初始化 ========= */
void Sysid_Mode_Init(void)
{
    Sysid_Log_Init();
    run_idx      = 0u;
    tick_in_run  = 0u;
    run_active   = 0u;
    stopped      = 0u;
    stop_code    = SYSID_ST_OK;
    hb_tick      = 0u;
    loop_cnt     = 0u;
#if SYSID_MODE != SYSID_MODE_TORQUE
    sysid_pose_ready = 0u;   /* 重新进入: 斜坡起点重新对齐实测角 */
#endif
    reinit_cnt++;   /* 累计不清零: 用来发现反复重入 */
#if SYSID_MODE != SYSID_MODE_TORQUE
    /* 位置控制: 虚拟关节 PD (只保留腿的位置环, 轮子增益归零) */
    RL_Torque_Param_Init(&sysid_pose_param, RL_MODEL_STANDUP);
    sysid_pose_param.p_gains[0] = SYSID_POSE_KP;    /* 左大腿 */
    sysid_pose_param.p_gains[1] = SYSID_POSE_KP;    /* 左小腿 */
    sysid_pose_param.p_gains[3] = SYSID_POSE_KP;    /* 右大腿 */
    sysid_pose_param.p_gains[4] = SYSID_POSE_KP;    /* 右小腿 */
    sysid_pose_param.p_gains[2] = 0.0f;
    sysid_pose_param.p_gains[5] = 0.0f;
    sysid_pose_param.d_gains[0] = SYSID_POSE_KD;
    sysid_pose_param.d_gains[1] = SYSID_POSE_KD;
    sysid_pose_param.d_gains[3] = SYSID_POSE_KD;
    sysid_pose_param.d_gains[4] = SYSID_POSE_KD;
    sysid_pose_param.d_gains[2] = 0.0f;
    sysid_pose_param.d_gains[5] = 0.0f;
    sysid_pose_param.wheel_pid[0][0] = 0.0f;
    sysid_pose_param.wheel_pid[0][1] = 0.0f;
    sysid_pose_param.wheel_pid[0][2] = 0.0f;
    sysid_pose_param.wheel_pid[1][0] = 0.0f;
    sysid_pose_param.wheel_pid[1][1] = 0.0f;
    sysid_pose_param.wheel_pid[1][2] = 0.0f;
    RL_Torque_State_Init(&sysid_pose_state, &sysid_pose_param);
#endif
}

/* ========= 单周期体 (1kHz) ========= */
void Sysid_Mode_Run(void)
{
    const sysid_run_t *run;
    float t, target, run_dur;
    float tau_cmd[DM_MOTOR_NUM];
    int16_t whl_cur[4];
    sysid_snap_t snap;
    uint64_t leg_tx, whl_tx, rx_ns;
    uint8_t i;
    uint8_t fault;
    uint32_t now;
#if SYSID_MODE == SYSID_MODE_TORQUE
    float pre_n;
#if SYSID_PRELOAD_TUNE
    int16_t dial;
#endif
#endif
#if SYSID_MODE != SYSID_MODE_TORQUE
    float act_buf[RL_ACTION_SIZE];
    float wheel_vel[2];
    float thigh_t;
    float shank_t;
    torque_output_t torque_out;
#endif

    now = HAL_GetTick();

#if SYSID_MODE == SYSID_MODE_TORQUE
    /* 预压沿腿力: 试验阶段用遥控滚轮实时调 (一边看列17腿长一边定值) */
#if SYSID_PRELOAD_TUNE
    dial = DR16_Deadline(DR16_Snapshot().wheel, 20u);
    if (dial > (int16_t)DR16_CH_LIMIT)
    {
        dial = (int16_t)DR16_CH_LIMIT;
    }
    if (dial < -(int16_t)DR16_CH_LIMIT)
    {
        dial = -(int16_t)DR16_CH_LIMIT;
    }
    pre_n = -SYSID_PRELOAD_TUNE_N * (float)dial / (float)DR16_CH_LIMIT;
#else
    pre_n = SYSID_PRELOAD_L_N;
#endif
#endif

    /* 重入检测 */
    if (now - last_call_tick > SYSID_REENTRY_MS)
    {
        Sysid_Mode_Init();
    }
    last_call_tick = now;

    /* 心跳: 任何状态都推, 停机时也能看出卡在哪 */
    hb_tick++;
    if (hb_tick >= SYSID_HB_TICKS)
    {
        hb_tick = 0u;
        sysid_push_heartbeat(Mono_Ns_Get(),
            dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].rx_ns);
    }

    /* 已中止: 只发零 */
    if (stopped)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
        return;
    }

    /* 硬故障检查 */
    fault = sysid_fault();
    if (fault != SYSID_ST_OK)
    {
        if (run_active)
        {
            sysid_pop_tx(&leg_tx, &whl_tx);
            rx_ns = dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].rx_ns;
            sysid_push_marker(SYSID_EVENT_ABORT, leg_tx, rx_ns);
            run_active = 0u;
        }
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
        stop_code = fault;
        stopped = 1u;
        return;
    }

    /* 空表 */
    if (SYSID_RUN_CNT == 0u)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
        return;
    }

    /* 循环 */
    if (run_idx >= SYSID_RUN_CNT)
    {
        /* 整表跑完一遍: 到设定遍数就停机 */
        loop_cnt++;
#if SYSID_LOOP_CNT > 0
        if (loop_cnt >= (uint32_t)SYSID_LOOP_CNT)
        {
            output_debug_dm_sent  = 0u;
            output_debug_dji_sent = 0u;
            (void)Dm_Send_Zero();
            (void)Dji_All_Stop();
            stop_code = SYSID_ST_DONE;
            stopped   = 1u;
            return;
        }
#endif
        run_idx     = 0u;
        tick_in_run = 0u;
        run_active  = 0u;
    }

    run = &SYSID_RUNS[run_idx];

    /* run 开始: 推标记后返回 */
    if (!run_active)
    {
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
        sysid_pop_tx(&leg_tx, &whl_tx);
        rx_ns = dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].rx_ns;
        sysid_push_start(run_idx, run, leg_tx, rx_ns);
        run_active = 1u;
        return;
    }

    /* 目标计算 */
    run_dur = sysid_run_dur(run);
    t = (float)tick_in_run * SYSID_DT;

    if (t < run_dur)
    {
        target = sysid_target(run, t);

        if (run->kind == SYSID_KIND_CMD_LEG)
        {
#if SYSID_MODE != SYSID_MODE_TORQUE
            /* ---- 位置控制: 目标角 → 虚拟关节 PD → 力矩 ---- */
            if (sysid_pose_ready == 0u)
            {
                /* 首次: 斜坡起点对齐当时的实测角 */
                (void)Ramp_Reset(&sysid_ramp_th, leg_l.output.thigh_angle);
                (void)Ramp_Reset(&sysid_ramp_sh, leg_l.output.virtual_shank_angle);
                sysid_pose_ready = 1u;
            }
            /* 目标按斜率一点一点加到姿态表的值 (1 rad/s) */
            thigh_t = Ramp_Update(&sysid_ramp_th, run->amplitude, SYSID_DT);
            shank_t = Ramp_Update(&sysid_ramp_sh, run->amp2, SYSID_DT);
            for (i = 0u; i < (uint8_t)RL_ACTION_SIZE; i++)
            {
                act_buf[i] = 0.0f;
            }
            /* 动作→目标: target = act×0.5 + dof_pos (见 rl_torque.c) */
            act_buf[0] = (thigh_t - sysid_pose_param.dof_pos[0]) * 2.0f;  /* 左大腿 */
            act_buf[1] = (shank_t - sysid_pose_param.dof_pos[1]) * 2.0f;  /* 左小腿 */
            act_buf[3] = (thigh_t - sysid_pose_param.dof_pos[3]) * 2.0f;  /* 右大腿 */
            act_buf[4] = (shank_t - sysid_pose_param.dof_pos[4]) * 2.0f;  /* 右小腿 */
            wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
            wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
            for (i = 0u; i < DM_MOTOR_NUM; i++)
            {
                tau_cmd[i]  = 0.0f;
            }
            if (RL_Torque_Compute(&leg_l, &leg_r, &rl_control.torque_param[rl_control.policy.selected_model],
                          wheel_vel, act_buf, &rl_control.torque_state,
                          &torque_out))
            {
                for (i = 0u; i < DM_MOTOR_NUM; i++)
                {
                    /* 测试限幅 10Nm (手动链路是 20Nm) */
                    tau_cmd[i] = clampf(torque_out.dm[i],
                        -SYSID_TRQ_LIMIT_NM, SYSID_TRQ_LIMIT_NM);
                }
                output_debug_dm_sent = (uint8_t)Dm_Send_Torque(tau_cmd);
            }
            else
            {
                output_debug_dm_sent = (uint8_t)Dm_Send_Zero();
            }
            output_debug_dji_sent = (uint8_t)Dji_All_Stop();
#else
            for (i = 0u; i < DM_MOTOR_NUM; i++)
            {
                tau_cmd[i] = 0.0f;
                sysid_pre[i] = 0.0f;
            }
            tau_cmd[run->target] = sysid_clamp_f(target,
                -SYSID_TRQ_LIMIT_NM, SYSID_TRQ_LIMIT_NM);
#if SYSID_EXCITE_BOTH
            /* 对侧腿的对应电机给同值 (0<->2, 1<->3) */
            tau_cmd[run->target ^ 2u] = tau_cmd[run->target];
#endif
            /* 预压: 沿腿力 → 两髋力矩 (baseline 不加, 保留纯零基线) */
            if (run->test_id != TID_BASELINE)
            {
                sysid_pre[DM_MOTOR_LEG_F_LFT] = pre_n
                    * leg_l.output.leg_jac[0][0];
                sysid_pre[DM_MOTOR_LEG_B_LFT] = pre_n
                    * leg_l.output.leg_jac[0][1];
                sysid_pre[DM_MOTOR_LEG_F_RGT] = pre_n
                    * leg_r.output.leg_jac[0][0];
                sysid_pre[DM_MOTOR_LEG_B_RGT] = pre_n
                    * leg_r.output.leg_jac[0][1];
                for (i = 0u; i < DM_MOTOR_NUM; i++)
                {
                    tau_cmd[i] = sysid_clamp_f(tau_cmd[i] + sysid_pre[i],
                        -SYSID_TRQ_LIMIT_NM, SYSID_TRQ_LIMIT_NM);
                }
            }
            output_debug_dm_sent  = (uint8_t)Dm_Send_Torque(tau_cmd);
            output_debug_dji_sent = (uint8_t)Dji_All_Stop();
#endif
        }
        else
        {
            int16_t raw = sysid_to_raw(target);
            for (i = 0u; i < 4u; i++) { whl_cur[i] = 0; }
            /* 两边一起给: 各自按自己的输出极性换算 */
            whl_cur[DJI_MOTOR_WHEEL_LFT] = (machine->dji_sign[DJI_MOTOR_WHEEL_LFT].out < 0)
                                           ? (int16_t)(-raw) : raw;
            whl_cur[DJI_MOTOR_WHEEL_RGT] = (machine->dji_sign[DJI_MOTOR_WHEEL_RGT].out < 0)
                                           ? (int16_t)(-raw) : raw;
            output_debug_dm_sent  = (uint8_t)Dm_Send_Zero();
            output_debug_dji_sent = (uint8_t)Dji_Send_Current(
                Can_Bus_Handle(machine->dji_bus),
                dji_motor_config[DJI_MOTOR_WHEEL_LFT].control_id,
                whl_cur);
            for (i = 0u; i < DM_MOTOR_NUM; i++)
            {
                tau_cmd[i] = 0.0f;
#if SYSID_MODE == SYSID_MODE_TORQUE
                sysid_pre[i] = 0.0f;
#endif
            }
            sysid_wheel_cmd_raw[DJI_MOTOR_WHEEL_LFT] = raw;   /* 供 VOFA 显示 */
            sysid_wheel_cmd_raw[DJI_MOTOR_WHEEL_RGT] = raw;
        }

        sysid_pop_tx(&leg_tx, &whl_tx);
        rx_ns = dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].rx_ns;

        memset(&snap, 0, sizeof(snap));
        snap.kind = (run->kind == SYSID_KIND_CMD_LEG)
                    ? SYSID_KIND_CMD_LEG : SYSID_KIND_CMD_WHEEL;
        snap.phase_or_event = sysid_phase_num(run, t);
        snap.t_cmd_ns = (run->kind == SYSID_KIND_CMD_LEG)
                        ? leg_tx : whl_tx;
        snap.t_rx_ns = rx_ns;
#if SYSID_MODE != SYSID_MODE_TORQUE
        /* 位置控制: 记录当前目标角 (左右腿同值) */
        snap.pose_tgt[0] = thigh_t;
        snap.pose_tgt[1] = shank_t;
#else
        snap.preload_n = (run->kind == SYSID_KIND_CMD_LEG
                          && run->test_id != TID_BASELINE) ? pre_n : 0.0f;
#endif
        for (i = 0u; i < DM_MOTOR_NUM; i++)
        {
            snap.tau_cmd[i]  = tau_cmd[i];
            /* 顶到限幅 = 被削平 */
            if (fabsf(tau_cmd[i]) >= SYSID_TRQ_LIMIT_NM)
            {
                snap.clamp_cnt++;
            }
        }
        sysid_fill_fb(&snap);
        (void)Sysid_Log_Push(&snap);

        tick_in_run++;
    }
    else
    {
        /* run 结束 */
        output_debug_dm_sent = 0u;
        output_debug_dji_sent = 0u;
        (void)Dm_Send_Zero();
        (void)Dji_All_Stop();
        sysid_pop_tx(&leg_tx, &whl_tx);
        rx_ns = dji_motor_feedback[DJI_MOTOR_WHEEL_LFT].rx_ns;
        sysid_push_marker(SYSID_EVENT_RUN_END, leg_tx, rx_ns);
        run_idx++;
        tick_in_run = 0u;
        run_active  = 0u;
    }
}

#endif /* SYSID_ENABLE */
