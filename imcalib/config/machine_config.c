/*
 * 集中保存大小机的几何、驱动映射、补偿表、遥控和LQR/RL参数。
 * 编译机型及节拍/功能开关在头文件，具体数值在machine_table；物理标定值须由作者台架确认。
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
        .name = "big_wheelleg",
        /* 作者批准AUTO首轮：LHX目标，控制参数与自起同源，实机越阶待验证。 */
        .stair = {
            .mode = STAIR_MODE_AUTO,
            .detection = {
                .confirmed = 1u,
                .request_min = 0.1f,
                .yaw_request_max = 0.3f, .yaw_rate_max = 1.2f,
                .length_min = 0.27f, /* 两腿实际长度均达到门槛，无上限/轮速限制 */
                .angle_min = 0.1f, .acceleration_min = 80.0f, /* CSV试用值，尚非验收门槛 */
                .acceleration_tau = 0.02f, .confirm_time = 0.02f, .impact_window = 0.35f,
            },
            .process = {
                .confirmed = 1u,
                .swing_angle = {1.3f, 1.3f}, /* 作者选择的首轮连续几何角rad */
                .retract_length = {0.17f, 0.17f},
                .stall_angle_progress = 0.01f, .stall_length_progress = 0.002f,
                .angle_tolerance = 0.1f, .length_tolerance = 0.02f, .confirm_time = 0.05f,
                .swing_timeout_ms = 1500u, .retract_timeout_ms = 1500u,
                .total_timeout_ms = 3000u, .stall_timeout_ms = 300u, .wait_ms = 300u,
            },
        },
        /* 遥控：死区为DR16计数，速度m/s、偏航rad/s、腿长m */
        .rc = {
            .deadband_vel = 10u,
            .deadband_yaw = 20u,
            .deadband_len = 20u,
            .vel_max = 3.0f,
            .yaw_max = 5.0f,
            .len_init = 0.13f,
            .len_rate = 1.3f,
        },
        /* IMU安装映射；只迁移，不改标定 */
        .imu = {
            .eul_src   = {0,1, 2},
            .eul_sign  = {-1, 1, 1},
            .gyr_src   = {1,0, 2},
            .gyr_sign  = {-1, 1, 1},
            .acc_src   = {1,0, 2},
            .acc_sign  = {-1, -1, -1},
            .quat_src  = {1, 0, 2},
            .quat_sign = {-1, 1, 1},
        },
        /* 十态平衡及腿长/roll控制 */
        .lqr = {
            /* 向心前馈：mass kg、track/height_offset m、gain倍率 */
            .turn_comp = {25.544f, 0.42551f , 0.07113f, 0.2f},
            .motion_guard = {MACHINE_MOTION_GUARD_ENABLE, 0.70f, 3.0f}, /* 试用包线；放宽rad/s² */
            /* 十态姿态参考rad及yaw反馈开关 */
            .reference = {
                .leg_trim = {-0.0f, -0.0f},
                .pitch_trim = 0.0f,
                .yaw_hold = 1u,
                .yaw_rate_hold = 1u,
            }, /* 十态姿态参考及偏航反馈开关 */
            /* 停车位置保持：距离m、速度m/s、角度rad */
            .position_hold = {
                .pos_target = 0.0f,
                .pos_reset_dist = 3.5f,
                .pos_arm_vel = 0.15f,
                .pos_leg_tol = 0.15f,
                .pos_hold = 1u,
            }, /* 停车位置保持；m、m/s、rad */
            /* 1kHz低通α：pitch/yaw/roll角速度、高度速度 */
            .filter = {
                .lpf_alpha = {0.6f, 0.6f, 0.01f, 0.30f},
            }, /* 状态低通，顺序见结构体 */
            /* 四态：轴向力/偏置N，参考加速度m/s² */
            .roll_height = {
                .leg_force_min = -10.0f,
                .length_ref_accel_max = 1.0f,
                .roll_height_bias_common_max = 150.0f,
                .roll_height_bias_diff_max = 40.0f,
                .support_force = {25.544f * 9.81f * 0.5f, 25.544f * 9.81f * 0.5f},
            }, /* 四态轴向/高度控制；N、m/s² */
            /* 原PD：左右腿长度输出N，roll差动力N，Kd原换算口径不变 */
            .pd = {
                .leg_len = {
                { .kp = 2200.0f, .ki = 0.0f, .kd = 25000.0f, .max_output = 5000.0f, .integral_limit = 0.0f },
                { .kp = 2200.0f, .ki = 0.0f, .kd = 25000.0f, .max_output = 5000.0f, .integral_limit = 0.0f },
            },
                .roll = { .kp = 600.0f, .ki = 0.0f, .kd = 100.0f, .max_output = 5000.0f, .integral_limit = 0.0f },
            }, /* 原腿长/roll PD；参数口径保持 */
            /* 十态轮/腿摆矩输出开关 */
            .output = {
                .wheel_enable = 1u,
                .leg_swing_enable = 1u,
            }, /* 十态输出通道选择 */
        }, /* lqr */
        /* RL观测、动作、恢复及新鲜度 */
        .rl = {
            .motor_sign = {1, 1, -1, -1, 1, -1},
            .motor_zero = {0.0f, 0.0f, 0.0f, 0.0f},
            .nominal_pos = {-0.604720554f, 0.608324173f, 0.604720554f, -0.608324173f},
            .position_limit_center = {-0.664720554f, 0.667594582f, 0.664720554f, -0.667594582f},
            .observation = { .gyro_scale = 0.5f, .height_scale = 5.0f, .joint_vel_scale = 0.1f, .obs_clip = 100.0f, .velocity_clip = 200.0f },
            .torque = {
                .pos_scale = 0.5f,
                .wheel_vel_scale = 10.0f,
                .p_gains = {60.0f, 60.0f, 60.0f, 60.0f},
                .d_gains = {2.0f, 2.0f, 2.0f, 2.0f},
                .wheel_gain = {0.1f, 0.1f},
                .position_limit = 3.14f,
                .wheel_speed_limit = 100.0f,
                .leg_torque_limit = 40.0f,
                .wheel_torque_limit = 3.9f,
            },
            /* 恢复策略：公共起立角范围rad；RL恢复高度m、时间ms */
            .recovery = {
                .fallen_angle_deg = 40.0f,
                .recovery_height = 0.13f,
                .recovery_hold_ms = 200u,
                .recovery_stable_ms = 500u,
                .recovery_timeout_ms = 400u,
            }, /* 公共起立/恢复过程 */
            /* RL动作新鲜度/就绪期限，ms */
            .timing = {
                .action_timeout_ms = 100u,
                .ready_timeout_ms = 100u,
            }, /* RL动作新鲜度期限，ms */
        }, /* rl */
        /* 轮驱动：型号、传动、限矩Nm及实机标定 */
        .wheel = {
            .dji_type = 1u,
            .dji_gear_ratio = 15.3333333f,
            .dji_trq_clamp = (0.30f * 20.0f) * (15.3333333f / 19.2f),
            .wheel_r = 0.0525f,
            .dji_sign = {{-1, -1}, {1, 1}},
            .dji_bus = 3,
        }, /* 轮驱动：型号、传动、限矩、总线和标定 */
        /* 髋驱动：MIT刻度和限矩分开，左前/左后/右前/右后 */
        .hip = {
            .dm_pos_max = 3.14159f,
            .dm_vel_max = 45.0f,
            .dm_trq_max = 54.0f,
            .dm_trq_clamp = 54.0f,
            .dm_sign = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
            .dm_bus = {1, 1, 1, 1},
            .dm_zero = {0.476998f, 1.974491f,0.476998f, 1.974491f },
        }, /* 髋驱动：MIT刻度、限矩、总线和标定；不改物理值 */
        /* 机构：杆长/工作范围m，角偏置rad */
        .leg = {
            .leg_lu = 0.21f,
            .leg_lg = 0.25f,
            .leg_len_min = MACHINE_BIG_LEG_LEN_MIN,
            .leg_len_max = MACHINE_BIG_LEG_LEN_MAX,
            .leg_off_phi0 = {-0.0f, -0.0f},
        }, /* 机构几何及机械工作范围 */
        /* 公共弹簧/腿自重物理补偿 */
        .compensation = {
            .spring = &machine_spring_leg3,
            .gravity = &gravity_big,
        }, /* 公共物理补偿模型 */
        /* 恢复策略：公共起立角范围rad；RL恢复高度m、时间ms */
        .recovery = {
            .fallen_support_angle_range = 0.6f,
        }, /* 公共起立/恢复过程 */
    },
    [MACHINE_ID_SMALL_WHEELLEG] = {
        .name = "small_wheelleg",
        .stair = { .mode = STAIR_MODE_OFF }, /* 小机不开放台阶入口 */
        /* 遥控：死区为DR16计数，速度m/s、偏航rad/s、腿长m */
        .rc = {
            .deadband_vel = 10u,
            .deadband_yaw = 20u,
            .deadband_len = 20u,
            .vel_max = 3.0f,
            .yaw_max = 5.0f,
            .len_init = 0.14f,
            .len_rate = 0.3f,
        },
        /* 十态平衡及腿长/roll控制 */
        .lqr = {
            /* 向心前馈：mass kg、track/height_offset m、gain倍率 */
            .turn_comp = {0.0f, 0.6918f, 0.0901f, 0.0f},
            .motion_guard = {0u, 0.375f, 3.0f}, /* 小机暂不启用 */
            /* 十态姿态参考rad及yaw反馈开关 */
            .reference = {
                .leg_trim = {-0.04f, -0.04f},
                .pitch_trim = 0.0f,
                .yaw_hold = 1u,
                .yaw_rate_hold = 1u,
            }, /* 十态姿态参考及偏航反馈开关 */
            /* 停车位置保持：距离m、速度m/s、角度rad */
            .position_hold = {
                .pos_target = 0.10f,
                .pos_reset_dist = 3.5f,
                .pos_arm_vel = 0.0f,
                .pos_leg_tol = 0.3f,
                .pos_hold = 1u,
            }, /* 停车位置保持；m、m/s、rad */
            /* 1kHz低通α：pitch/yaw/roll角速度、高度速度 */
            .filter = {
                .lpf_alpha = {0.3f, 0.3f, 0.0f, 0.0f},
            }, /* 状态低通，顺序见结构体 */
            /* 四态：轴向力/偏置N，参考加速度m/s² */
            .roll_height = {
                .leg_force_min = -10.0f,
                .length_ref_accel_max = 1.0f,
                .support_force = {8.0f, 8.0f},
            }, /* 四态轴向/高度控制；N、m/s² */
            /* 原PD：左右腿长度输出N，roll差动力N，Kd原换算口径不变 */
            .pd = {
                .leg_len = {
                { .kp = 2500.0f, .ki = 0.0f, .kd = 10000.0f, .max_output = 5000.0f, .integral_limit = 0.0f },
                { .kp = 2500.0f, .ki = 0.0f, .kd = 10000.0f, .max_output = 5000.0f, .integral_limit = 0.0f },
            },
                .roll = { .kp = 200.0f, .ki = 0.0f, .kd = 50.0f, .max_output = 5000.0f, .integral_limit = 0.0f },
            }, /* 原腿长/roll PD；参数口径保持 */
            /* 十态轮/腿摆矩输出开关 */
            .output = {
                .wheel_enable = 1u,
                .leg_swing_enable = 1u,
            }, /* 十态输出通道选择 */
        }, /* lqr */
        /* IMU安装映射；只迁移，不改标定 */
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
        /* RL观测、动作、恢复及新鲜度 */
        .rl = {
            /* 恢复策略：公共起立角范围rad；RL恢复高度m、时间ms */
            .recovery = {
                .fallen_angle_deg = 40.0f,
                .recovery_height = 0.13f,
                .recovery_hold_ms = 200u,
                .recovery_stable_ms = 500u,
                .recovery_timeout_ms = 400u,
            }, /* 公共起立/恢复过程 */ /* RL动作新鲜度期限，ms */
        }, /* rl */
        /* 轮驱动：型号、传动、限矩Nm及实机标定 */
        .wheel = {
            .dji_type = 0u,
            .dji_gear_ratio = 36.0f,
            .dji_trq_clamp = 1.8f,
            .wheel_r = 0.04f,
            .dji_sign = {{1, 1}, {-1, -1}},
            .dji_bus = 2,
        }, /* 轮驱动：型号、传动、限矩、总线和标定 */
        /* 髋驱动：MIT刻度和限矩分开，左前/左后/右前/右后 */
        .hip = {
            .dm_pos_max = 3.14159f,
            .dm_vel_max = 30.0f,
            .dm_trq_max = 10.0f,
            .dm_trq_clamp = 10.0f,
            .dm_sign = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
            .dm_bus = {1, 1, 3, 3},
            .dm_zero = {-0.056f, 0.296f, 0.024f, 0.316f},
        }, /* 髋驱动：MIT刻度、限矩、总线和标定；不改物理值 */
        /* 机构：杆长/工作范围m，角偏置rad */
        .leg = {
            .leg_lu = 0.13087f,
            .leg_lg = 0.15240f,
            .leg_len_min = MACHINE_SMALL_LEG_LEN_MIN,
            .leg_len_max = MACHINE_SMALL_LEG_LEN_MAX,
            .leg_off_phi0 = {-0.0f, -0.0f},
        }, /* 机构几何及机械工作范围 */
        /* 公共弹簧/腿自重物理补偿 */
        .compensation = {
            .spring = 0,
        }, /* 公共物理补偿模型 */
        /* 恢复策略：公共起立角范围rad；RL恢复高度m、时间ms */
        .recovery = {
            .fallen_support_angle_range = 0.6f,
        }, /* 公共起立/恢复过程 */
    },
};
const machine_cfg_t *const machine = &machine_table[MACHINE_DEFAULT];
