#include "machine_config.h"

/* 机器① 自制减速箱比: 待机械确认后改这一行 */
#define CJL_WHEEL_BOX_RATIO     1.0f

/* 两份电机配置表: 换机器改 machine_config.h 的 MACHINE_DEFAULT */
const machine_cfg_t machine_table[MACHINE_NUM] = {
    [MACHINE_ID_CHUANLIANTUI] = {
        .name           = "chuanliantui",
        .dji_type       = 1u,                          /* M3508 + C620 */
        .dji_gear_ratio = (19.2f * CJL_WHEEL_BOX_RATIO),
        .dji_trq_clamp  = 6.0f,                        /* 满限幅 = 0.30 Nm/A × 20A */
        .dm_pos_max     = 12.5f,                       /* DM-J8009P MIT 预设 */
        .dm_vel_max     = 45.0f,
        .dm_trq_max     = 54.0f,
        .dm_trq_clamp   = 20.0f,                       /* 签字许可力矩 */
        /* 极性: 腿 前左/后左/前右/后右, 轮 左/右 (按实机安装填) */
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{1, 1}, {-1, -1}},
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
