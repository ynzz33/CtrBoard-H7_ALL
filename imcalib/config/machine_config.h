/*
 * 文件职责：两机参数、标定与编译选择（machine_config.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：作者台架标定、机械数据和既有控制参数。
 * 输出/去向：向驱动/几何/IMU/RL/LQR提供同一只读machine配置和节拍宏。
 * 关键边界：极性/零点/量程/几何只能按作者实测修改；当前FDCAN时序以Core/Src/fdcan.c实际初始化为准。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef __MACHINE_CONFIG_H
#define __MACHINE_CONFIG_H

#include <stdint.h>

/* 机器编号 */
#define MACHINE_ID_BIG_WHEELLEG    0u
#define MACHINE_ID_SMALL_WHEELLEG  1u

#define MACHINE_NUM               2u

/* 电机数量 (须与 dm.c / dji.c 的枚举一致) */
#define MACHINE_LEG_NUM           4u
#define MACHINE_WHEEL_NUM         2u

/* 编译选机，参数与 K 表同步绑定。 */
#ifndef MACHINE_DEFAULT
#define MACHINE_DEFAULT           MACHINE_ID_BIG_WHEELLEG
#endif

#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
#define MACHINE_TIM6_PERIOD       999u    /* 1 kHz */
#define MACHINE_TICK_DT           0.001f
#define MACHINE_RL_CTRL_DIV       2u      /* RL实体PD/反馈采样/正常下发：500Hz */
#define MACHINE_POLICY_DIV        20u     /* actor推理50Hz；一个action保持10个2ms控制子步 */
#define MACHINE_DM_OFFLINE_MS     50u
#define MACHINE_VOFA_PORT         8u      /* UART8 */
#define MACHINE_FDCAN13_DATA_PRESCALER  1u
#define MACHINE_FDCAN13_DATA_SEG1       4u
#define MACHINE_FDCAN13_DATA_SEG2       1u
#elif MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
#define MACHINE_TIM6_PERIOD       999u    /* 1 kHz */
#define MACHINE_TICK_DT           0.001f
#define MACHINE_RL_CTRL_DIV       1u      /* 小机兼容 */
#define MACHINE_DM_OFFLINE_MS     10u
#define MACHINE_POLICY_DIV        10u     /* 策略节拍 1000/10 = 100 Hz */
#define MACHINE_VOFA_PORT         1u      /* UART1 */
#define MACHINE_FDCAN13_DATA_PRESCALER  3u
#define MACHINE_FDCAN13_DATA_SEG1       5u
#define MACHINE_FDCAN13_DATA_SEG2       2u
#else
#error "Unsupported MACHINE_DEFAULT"
#endif

#define MACHINE_LQR_DT            MACHINE_TICK_DT
#define MACHINE_RL_CTRL_DT        (MACHINE_TICK_DT * (float)MACHINE_RL_CTRL_DIV)
#define MACHINE_POLICY_DT         (MACHINE_TICK_DT * (float)MACHINE_POLICY_DIV)

/* 一路电机的极性 */
typedef struct {
    int8_t fb;      /* 反馈极性 */
    int8_t out;     /* 输出极性 */
} motor_sign_t;

/* IMU 安装: 机体三轴各取模块哪一路、乘什么符号 (输出 = 原始值 × 符号) */
typedef struct {
    uint8_t eul_src[3];   /* 欧拉角来源, 按机体 俯仰/横滚/偏航 序; 值 = 模块 0 Roll / 1 Pitch / 2 Yaw */
    int8_t  eul_sign[3];  /* 欧拉角符号, 同序 */
    uint8_t gyr_src[3];   /* 角速度来源, 按机体 X/Y/Z 序 */
    int8_t  gyr_sign[3];  /* 角速度符号, 输出 X/Y/Z */
    uint8_t acc_src[3];   /* 加速度来源, 按机体 X/Y/Z 序 */
    int8_t  acc_sign[3];  /* 加速度符号, 输出 X/Y/Z */
    uint8_t quat_src[3];  /* 四元数输出 X/Y/Z 分别取模块 X/Y/Z 哪一路 */
    int8_t  quat_sign[3]; /* 四元数输出 X/Y/Z 极性, 独立于 quat_src */
} imu_cfg_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output;
    float integral_limit;
} machine_pid_cfg_t;

typedef struct {
    float vel_max;              /* m/s */
    float yaw_max;              /* rad/s */
    float len_init;             /* m */
    float len_rate;             /* m/s */
} machine_rc_cfg_t;

/* RL39：缩放前裁剪，模型原生q/dq供电机PD使用。 */
typedef struct {
    float gyro_scale;          /* 机体角速度观测缩放 */
    float height_scale;        /* 目标高度命令观测缩放 */
    float joint_vel_scale;     /* 六路电机速度观测缩放 */
    float obs_clip;            /* 角速度/重力/角差/action的缩放前裁剪幅值 */
    float velocity_clip;       /* 电机速度缩放前裁剪幅值，rad/s */
} machine_rl_observation_cfg_t;
typedef struct {
    float pos_scale;           /* action→相对名义角偏移，rad/action */
    float wheel_vel_scale;     /* action→轮轴速度目标，rad/s/action */
    float p_gains[4];          /* 实体电机位置比例，Nm/rad */
    float d_gains[4];          /* 实体电机速度阻尼，Nm/(rad/s) */
    float wheel_gain[2];       /* 轮轴速度比例，Nm/(rad/s) */
    float position_limit;     /* q_reference两侧的目标范围，rad；不限制反馈或环绕角差 */
    float wheel_speed_limit;  /* 轮轴速度目标幅值，rad/s */
    float leg_torque_limit;   /* RL腿力矩限幅，Nm；与MIT编码刻度分开 */
    float wheel_torque_limit; /* RL轮力矩限幅，Nm；与公共满电流限幅分开 */
} machine_rl_torque_cfg_t;
/* 实体顺序LF前/后、RF前/后、左轮/右轮。
 * 电机反馈已包含驱动机械标定；motor_zero基于pos_continuous_rad，不再叠加旧虚拟关节零位。
 * 配对映射将公共驱动坐标转换至RL坐标，不能照网络action顺序填写。 */
typedef struct {
    int8_t motor_sign[6];      /* 固件↔模型配对方向，输入角/速与输出力矩共用 */
    float motor_zero[4];       /* 固件标定连续角至模型零位的额外偏移，rad */
    float nominal_pos[4];      /* q_nominal：训练默认站立原生角，观测减此值、目标加此值 */
    float reference_pos[4];    /* q_reference：XML的qpos0，目标限幅中心，非实机零偏 */
    uint8_t configured;        /* 本机是否启用RL接口；小机当前未接入 */
    machine_rl_observation_cfg_t observation;
    machine_rl_torque_cfg_t torque;
    uint32_t action_timeout_ms; /* 已发布动作停止更新的故障期限 */
    uint32_t ready_timeout_ms; /* 实际投入后动作持续不可用的故障期限 */
} machine_rl_cfg_t;

typedef struct {
    float leg_trim[2];
    float pitch_trim;
    float pos_target;
    float pos_arm_vel;
    float lpf_alpha[2];             /* pitch/yaw角速 */
    machine_pid_cfg_t leg_len[2];
    machine_pid_cfg_t roll;
    float support_force[2];
    uint8_t yaw_hold;
    uint8_t yaw_rate_hold;
    uint8_t pos_hold;
    uint8_t wheel_enable;
    uint8_t hip_enable;
    uint8_t len_pid_enable;
} machine_lqr_cfg_t;

typedef struct {
    float len_min;
    float len_max;
    float bar_sum_sq;
    float bar_prod2;
    float anchor_sum_sq;
    float anchor_prod2;
    float phase;
    float numerator_scale;
    float denominator_scale;
    float force_n;
} machine_spring_cfg_t;

extern const machine_spring_cfg_t machine_spring_leg3;

typedef struct {
    float length;
    float offset;
    float radius;
} machine_gravity_node_t;

typedef struct {
    float mass;
    float gravity;
    const machine_gravity_node_t *nodes;
    uint8_t count;
} machine_gravity_cfg_t;

/* 一台机器的全部参数 */
typedef struct {
    const char *name;
    machine_rc_cfg_t rc;        /* 公共量程 */
    uint8_t     lqr_configured;     /* 此机器有匹配的 LQR 增益表 */
    machine_lqr_cfg_t lqr;
    const machine_spring_cfg_t *spring;
    const machine_gravity_cfg_t *gravity;
    /* 轮: 型号(0=M2006, 1=M3508) + 总传动比 + 满限幅力矩(Nm) */
    uint8_t     dji_type;
    float       dji_gear_ratio;
    float       dji_trq_clamp;
    float       wheel_r;
    /* 腿: MIT 三个满量程 + 满限幅力矩(Nm) */
    float       dm_pos_max;
    float       dm_vel_max;
    float       dm_trq_max;
    float       dm_trq_clamp;
    /* 极性: 腿 4 台 (前左/后左/前右/后右), 轮 2 个 (左/右) */
    motor_sign_t dm_sign[MACHINE_LEG_NUM];
    motor_sign_t dji_sign[MACHINE_WHEEL_NUM];
    /* 总线号: 1=FDCAN1 2=FDCAN2 3=FDCAN3 */
    uint8_t     dm_bus[MACHINE_LEG_NUM];
    uint8_t     dji_bus;
    /* 电机零点: 4 台腿 (前左/后左/前右/后右), 在 dm.c 解码时叠加 */
    float       dm_zero[MACHINE_LEG_NUM];
    /* 腿几何: 杆长 + 腿长工作区间 + 腿摆角零位 (左/右) */
    float       leg_lu;
    float       leg_lg;
    float       leg_len_min;
    float       leg_len_max;
    float       leg_off_phi0[2];
    /* IMU 安装极性 */
    imu_cfg_t   imu;
    /* RL 默认配置 */
    machine_rl_cfg_t rl;
} machine_cfg_t;

extern const machine_cfg_t machine_table[MACHINE_NUM];
extern const machine_cfg_t *const machine; /* 编译期选定的机器 */


#endif
