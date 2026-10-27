/*
 * 文件职责：两机参数、标定与编译选择（machine_config.c）。
 * 输入/来源：作者台架标定、机械数据和既有控制参数。
 * 输出/去向：向驱动/几何/IMU/RL/LQR提供同一只读machine配置和节拍宏。
 * 关键边界：极性/零点/量程/几何只能按作者实测修改；当前FDCAN时序以Core/Src/fdcan.c实际初始化为准。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "machine_config.h"

/* Leg3 原模型常量 */
const machine_spring_cfg_t machine_spring_leg3 = {
    0.05F, 0.45F, 0.1066F, 0.105F, 0.0473512858F,
    0.0206288453F, 0.213803F, 0.0103144227F, 0.0525F, 150.0F
};

/* 现有有效质心节点 */
static const machine_gravity_node_t gravity_nodes_big[] = {
    {0.15f, 0.14696847451069225f, 0.06629277864141764f},
    {0.16f, 0.15233205965915383f, 0.06716142047336403f},
    {0.17f, 0.15785793169809365f, 0.06810379284592012f},
    {0.18f, 0.16353393531619057f, 0.06910968094268703f},
    {0.19f, 0.16932790703247946f, 0.07018361703417686f},
    {0.20f, 0.17523986903670066f, 0.07130926798109767f},
    {0.21f, 0.18125438063671734f, 0.07249103737704407f},
    {0.22f, 0.18735804573062775f, 0.07371456640312009f},
    {0.23f, 0.1935440737919919f, 0.07498472177717272f},
    {0.24f, 0.19981164355462372f, 0.07629346564418214f},
    {0.25f, 0.20613804355334314f, 0.07763950669601141f},
    {0.26f, 0.2125254867068889f, 0.07902583438344704f},
    {0.27f, 0.21897323717751446f, 0.08044798692322885f},
    {0.28f, 0.22546613138118995f, 0.08189613177678175f},
    {0.29f, 0.23200655335571885f, 0.08337530089900726f},
    {0.30f, 0.23859429875837357f, 0.08488368158839482f},
    {0.31f, 0.2452153170175142f, 0.08641615416112892f},
};

static const machine_gravity_cfg_t gravity_big = {
    .mass = 1.408f,
    .gravity = 9.81f,
    .nodes = gravity_nodes_big,
    .count = sizeof(gravity_nodes_big) / sizeof(gravity_nodes_big[0]),
};

/* 两份电机配置表: 换机器改 machine_config.h 的 MACHINE_DEFAULT */
const machine_cfg_t machine_table[MACHINE_NUM] = {
    [MACHINE_ID_BIG_WHEELLEG] = {
        .name           = "big_wheelleg",             /* 大轮腿 */
        .rc = {
            .vel_max = 4.0f,
            .yaw_max = 7.0f,
            .len_init = 0.15f,
            .len_rate = 1.3f,
        },
        .spring = &machine_spring_leg3,               /* 气弹簧表 */
        .gravity = &gravity_big,
        .dji_type       = 1u,                          /* M3508 + C620 */
        .dji_gear_ratio = 15.3333333f,                       /* 转子→轮子总减速比 */
        .dji_trq_clamp  = (0.30f * 20.0f) * (15.3333333f / 19.2f), /* 满电流 */
        .wheel_r        = 0.0525f,                     /* 作者确认 */
        .dm_pos_max     = 3.14159f,                    /* DM-J8009P: 上位机 ±π */
        .dm_vel_max     = 45.0f,
        .dm_trq_max     = 54.0f,                       /* MIT 刻度, 勿改 */
        .dm_trq_clamp   = 54.0f,                       /* 满量程 */
        /* 极性: 前左/后左/前右/后右 */
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{-1, -1}, {1, 1}},
        /* 总线: 腿 4 台全在 FDCAN1, 轮在 FDCAN3 */
        .dm_bus         = {1, 1, 1, 1},
        .dji_bus        = 3,
        /* 电机零点: 前左/后左/前右/后右 (作者标定) */
        .dm_zero        = {0.476998f, 1.974491f,0.476998f, 1.974491f },
        /* 腿几何: 杆长 0.21/0.25; 腿长区间为实测工作区间 */
        .leg_lu         = 0.21f,
        .leg_lg         = 0.25f,
        .leg_len_min    = 0.13f,
        .leg_len_max    = 0.34f,
        .leg_off_phi0   = {-0.0f, -0.0f},
        /* IMU 轴映射待台架核对 */
        .imu = {
            //roll pitch yaw
            .eul_src   = {0,1, 2},
            .eul_sign  = {-1, 1, 1},
            .gyr_src   = {1,0, 2},
            .gyr_sign  = {-1, 1, 1},
            .acc_src   = {1,0, 2},
            .acc_sign  = {-1, -1, -1},
            .quat_src  = {1, 0, 2},    /* 四元数 X/Y 交换；轴向待烧录后复核 */
            .quat_sign = {-1, 1, 1},  /* quat_src 后独立修正训练极性 */
        },
        .lqr = {
            .leg_trim = {-0.0f, -0.0f},             /* 摆角目标 rad */
            .pitch_trim = 0.0f,                      /* 俯仰目标 rad */
            .pos_target = 0.1f,                      /* 位移目标 m */
            .pos_arm_vel = 0.1f,                     /* 松杆低速门槛 */
            /* 1kHz低通顺序：pitch角速度 / yaw角速度 / roll角速度 / 高度变化速度；越小越平滑、延迟越大 */
            .lpf_alpha = {0.6f, 0.6f, 0.01f, 0.15f},
            .roll_height_bias_common_max = 100.0f,
            .roll_height_bias_diff_max = 20.0f,
            .turn_comp = {20.534f, 0.42551f, 0.07113f, 0.3f}, /* 当前大机模型；当前系数0.2、方向+1 */
            /* PID 顺序: Kp / Ki / Kd / 输出限幅 / 积分限幅; 输出单位 N */
            .leg_len = {{1900.0f, 0.0f, 150000.0f, 5000.0f, 0.0f},
                        {1900.0f, 0.0f, 150000.0f, 5000.0f, 0.0f}}, /* 左右腿长 */
            .roll = {600.0f, 0.0f, 100.0f, 5000.0f, 0.0f}, /* 横滚补偿 */
            .support_force = {20.534f * 9.81f * 0.5f, 20.534f * 9.81f * 0.5f}, /* 支撑前馈 N */
            .yaw_hold = 1u,                          /* 偏航角环 */
            .yaw_rate_hold = 1u,                     /* 偏航角速环 */
            .pos_hold = 1u,                          /* 位移环 */
            .wheel_enable = 1u,                      /* 轮矩输出 */
            .leg_swing_enable = 1u,                        /* 摆腿矩输出 */
        },
        /* 公共驱动保留LQR标定；RL配对转换复现作者提供的可用RL方向与零点。 */
        .rl = {
            .motor_sign = {1, 1, -1, -1, 1, -1}, /* 四腿两轮顺序；右腿/右轮转换到RL坐标 */
            .motor_zero = {0.0f, 0.0f, 0.0f, 0.0f},
            /* 版本化XML与训练model.py微蹲闭链姿态：四腿顺序LF前/后、RF前/后。 */
            .nominal_pos = {-0.604720554f, 0.608324173f, 0.604720554f, -0.608324173f},
            .position_limit_center = {-0.664720554f, 0.667594582f, 0.664720554f, -0.667594582f},
            .observation = {0.5f, 5.0f, 0.1f, 100.0f, 200.0f}, /* gyro/height/velocity缩放及两种裁剪 */
            .torque = {
                .pos_scale = 0.5f,
                .wheel_vel_scale = 10.0f,
                .p_gains = {60.0f, 60.0f, 60.0f, 60.0f},
                .d_gains = {2.0f, 2.0f, 2.0f, 2.0f},
                .wheel_gain = {0.1f, 0.1f},
                .position_limit = 3.14f,
                .wheel_speed_limit = 150.0f,
                .leg_torque_limit = 40.0f, /* 训练电机饱和；公共DM限幅与MIT刻度保持原值 */
                .wheel_torque_limit = 3.9f,
            },
            .action_timeout_ms = 100u,
            .ready_timeout_ms = 100u,
        },
    },
    [MACHINE_ID_SMALL_WHEELLEG] = {
        .name           = "small_wheelleg",
        .rc = {
            .vel_max = 3.0f,
            .yaw_max = 5.0f,
            .len_init = 0.14f,
            .len_rate = 0.3f,
        },
        /* 小轮腿目前只使用 LQR。TODO: 训练小轮腿 RL 模型后再配置关节映射并启用。 */
        .spring = 0,
        .lqr = {
            .leg_trim = {-0.04f, -0.04f},
            .pitch_trim = 0.0f, .pos_target = 0.10f,
            .pos_arm_vel = 0.0f,
            .lpf_alpha = {0.3f, 0.3f, 0.0f, 0.0f}, /* pitch/yaw/roll角速度/高度速度；小机未接入四态，后两项保持0 */
            .leg_len = {{2500.0f, 0.0f, 10000.0f, 5000.0f, 0.0f},
                        {2500.0f, 0.0f, 10000.0f, 5000.0f, 0.0f}},
            .roll = {200.0f, 0.0f, 50.0f, 5000.0f, 0.0f},
            .support_force = {8.0f, 8.0f},
            .turn_comp = {0.0f, 0.6918f, 0.0901f, 0.0f}, /* 小机本轮不补偿；轮距/高度偏置沿用小机模型 */
            .yaw_hold = 1u, .yaw_rate_hold = 1u,
            .pos_hold = 1u, .wheel_enable = 1u, .leg_swing_enable = 1u,
        },
        .dji_type       = 0u,                          /* M2006 */
        .dji_gear_ratio = 36.0f,
        .dji_trq_clamp  = 1.8f,                        /* 满限幅 = 0.18 Nm/A × 10A */
        .wheel_r        = 0.04f,                       /* Leg2_v1 WHEEL_R */
        .dm_pos_max     = 3.14159f,                    /* DM-J4310 */
        .dm_vel_max     = 30.0f,
        .dm_trq_max     = 10.0f,
        .dm_trq_clamp   = 10.0f,
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{1, 1}, {-1, -1}},
        /* 总线: 左腿 FDCAN1, 右腿 FDCAN3, 轮 FDCAN2 */
        .dm_bus         = {1, 1, 3, 3},
        .dji_bus        = 2,
        /* 电机零点: 前左/后左/前右/后右 (本机原值) */
        .dm_zero        = {-0.056f, 0.296f, 0.024f, 0.316f},
        /* 腿几何 (本机原值) */
        .leg_lu         = 0.13087f,
        .leg_lg         = 0.15240f,
        .leg_len_min    = 0.13f,
        .leg_len_max    = 0.23f,
        .leg_off_phi0   = {-0.0f, -0.0f},
        /* IMU */
        .imu = {
            .eul_src   = {1, 0, 2},
            .eul_sign  = {-1,-1, -1},
            .gyr_src   = {0, 1, 2},
            .gyr_sign  = {-1,-1, -1},
            .acc_src   = {0, 1, 2},
            .acc_sign  = {-1, 1, -1},
            .quat_src  = {0, 1, 2},
            .quat_sign = {-1, 1, -1},
        },
        /* RL 关节映射: 模型对应大机器, 小机器未配置 */
        .rl = {0},
    },
};

const machine_cfg_t *const machine = &machine_table[MACHINE_DEFAULT];
