#include "machine_config.h"

/* 机器① 自制减速箱比: 待机械确认后改这一行 */
#define CJL_WHEEL_BOX_RATIO     1.0f

/* 两份电机配置表: 换机器改 machine_config.h 的 MACHINE_DEFAULT */
const machine_cfg_t machine_table[MACHINE_NUM] = {
    [MACHINE_ID_CHUANLIANTUI] = {
        .name           = "chuanliantui",
        .dji_type       = 1u,                          /* M3508 + C620 */
        .dji_gear_ratio = (19.2f * CJL_WHEEL_BOX_RATIO),
        .dji_trq_clamp  = 1.5f,                        /* 安全测试值 (满限幅 6.0) */
        .dm_pos_max     = 3.14159f,                    /* DM-J8009P: 上位机 ±π */
        .dm_vel_max     = 45.0f,
        .dm_trq_max     = 54.0f,                       /* MIT 刻度, 勿改 */
        .dm_trq_clamp   = 3.0f,                        /* 安全测试值 (满限幅 20.0) */
        /* 极性: 前左/后左/前右/后右 */
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{1, 1}, {-1, -1}},
        /* 总线: 腿 4 台全在 FDCAN1, 轮在 FDCAN3 */
        .dm_bus         = {1, 1, 1, 1},
        .dji_bus        = 3,
        /* 电机零点: 前左/后左/前右/后右 (照抄参考固件) */
        .dm_zero        = {0.476998f, 1.974491f,0.476998f, 1.974491f },
        /* 腿几何: 杆长 0.21/0.25; 腿长区间待台架给实测工作区间, 现为理论极限 */
        .leg_lu         = 0.21f,
        .leg_lg         = 0.25f,
        .leg_len_min    = 0.04f,
        .leg_len_max    = 0.46f,
        .leg_off_phi0   = {-0.13f, -0.07f},
    },
    [MACHINE_ID_LOCAL] = {
        .name           = "local-m2006-j4310",
        .dji_type       = 0u,                          /* M2006 */
        .dji_gear_ratio = 36.0f,
        .dji_trq_clamp  = 1.8f,                        /* 满限幅 = 0.18 Nm/A × 10A */
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
