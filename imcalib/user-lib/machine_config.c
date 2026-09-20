#include "machine_config.h"

/* 两份电机配置表: 换机器改 machine_config.h 的 MACHINE_DEFAULT */
const machine_cfg_t machine_table[MACHINE_NUM] = {
    [MACHINE_ID_CHUANLIANTUI] = {
        .name           = "chuanliantui",
        .dji_type       = 1u,                          /* M3508 + C620 */
        .dji_gear_ratio = 15.5f,                       /* 转子→轮子总减速比 */
        .dji_trq_clamp  = 4.8f,                        /* 15.5 箱比下的输出轴物理上限 */
        .wheel_r        = 0.04f,                       /* 占位，待实测 */
        .dm_pos_max     = 3.14159f,                    /* DM-J8009P: 上位机 ±π */
        .dm_vel_max     = 45.0f,
        .dm_trq_max     = 54.0f,                       /* MIT 刻度, 勿改 */
        .dm_trq_clamp   = 20.0f,                       /* 满限幅 (作者许可力矩) */
        /* 极性: 前左/后左/前右/后右 */
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{1, 1}, {-1, -1}},
        /* 总线: 腿 4 台全在 FDCAN1, 轮在 FDCAN3 */
        .dm_bus         = {1, 1, 1, 1},
        .dji_bus        = 3,
        /* 电机零点: 前左/后左/前右/后右 (作者标定) */
        .dm_zero        = {0.476998f, 1.974491f,0.476998f, 1.974491f },
        /* 腿几何: 杆长 0.21/0.25; 腿长区间为实测工作区间 */
        .leg_lu         = 0.21f,
        .leg_lg         = 0.25f,
        .leg_len_min    = 0.14f,
        .leg_len_max    = 0.34f,
        .leg_off_phi0   = {-0.13f, -0.07f},
    },
    [MACHINE_ID_LOCAL] = {
        .name           = "local-m2006-j4310",
        .dji_type       = 0u,                          /* M2006 */
        .dji_gear_ratio = 36.0f,
        .dji_trq_clamp  = 1.8f,                        /* 满限幅 = 0.18 Nm/A × 10A */
        .wheel_r        = 0.03f,                       /* Leg2_v1 建模值 */
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
        .dm_zero        = {-0.03f, -0.04f, -0.038f, -0.023f},
        /* 腿几何 (本机原值) */
        .leg_lu         = 0.13087f,
        .leg_lg         = 0.15240f,
        .leg_len_min    = 0.10f,
        .leg_len_max    = 0.20f,
        .leg_off_phi0   = {-0.13f, -0.07f},
    },
};

const machine_cfg_t *machine = &machine_table[MACHINE_DEFAULT];

/* 换机器 */
void Machine_Select(uint8_t id)
{
    if (id < MACHINE_NUM)
    {
        machine = &machine_table[id];
    }
}

/* 当前机器号 */
uint8_t Machine_Id(void)
{
    return (uint8_t)(machine - machine_table);
}
