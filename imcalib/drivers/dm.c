/*
 * 文件职责：达妙髋电机MIT协议与状态/使能管理（dm.c）。
 * 输入/来源：CAN反馈、机器ID/极性/零点/MIT量程与四髋力矩请求。
 * 输出/去向：解析物理角/速/矩和接收时间，维护在线/故障/使能，量化MIT命令并发送。
 * 关键边界：所有实体极性/零点在驱动边界统一处理；控制层传物理量，不再重复乘符号。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "dm.h"
#include "machine_config.h"
#include "mono_ns.h"
#include <string.h>
#include <math.h>

/* 编译期检查: 极性表长度对齐 */
typedef char dm_num_check[(MACHINE_LEG_NUM == DM_MOTOR_NUM) ? 1 : -1];

/* 固定参: 只存报文 ID (总线/极性见 machine_config.c) */
const dm_motor_config_t dm_motor_config[DM_MOTOR_NUM] = {
    [DM_MOTOR_LEG_F_LFT] = {
        .feedback_id = 0x11u,
        .control_id = 0x01u,
    },
    [DM_MOTOR_LEG_B_LFT] = {
        .feedback_id = 0x13u,
        .control_id = 0x03u,
    },
    [DM_MOTOR_LEG_F_RGT] = {
        .feedback_id = 0x12u,
        .control_id = 0x02u,
    },
    [DM_MOTOR_LEG_B_RGT] = {
        .feedback_id = 0x14u,
        .control_id = 0x04u,
    },
};

/* 反馈值 */
dm_motor_feedback_t dm_motor_feedback[DM_MOTOR_NUM];

/* MIT解码 */
float Dm_Uint_To_Float(uint16_t value, float min, float max, uint8_t bits)
{
    uint32_t max_raw = (1UL << bits) - 1UL;
    return (float)value * (max - min) / (float)max_raw + min;
}

/* MIT编码 */
uint16_t Dm_Float_To_Uint(float value, float min, float max, uint8_t bits)
{
    uint32_t max_raw = (1UL << bits) - 1UL;
    float scaled;

    if (value < min) { value = min; }
    if (value > max) { value = max; }
    scaled = (value - min) / (max - min) * (float)max_raw;
    return (uint16_t)(uint32_t)scaled;
}

/* 计圈数 */
static void Dm_Update_Angle(uint8_t index)
{
    static uint16_t last_angle[DM_MOTOR_NUM];
    static int32_t turns[DM_MOTOR_NUM];
    static uint32_t inited;
    dm_motor_feedback_t *feedback = &dm_motor_feedback[index];
    int32_t delta;

    if (!(inited & (1UL << index)))
    {
        last_angle[index] = feedback->angle_raw;
        feedback->angle_total = feedback->angle_raw;
        inited |= 1UL << index;
        return;
    }

    delta = (int32_t)feedback->angle_raw - (int32_t)last_angle[index];
    if (delta > (DM_ANGLE_CPR / 2L))
    {
        turns[index]--;
    }
    else if (delta < -(DM_ANGLE_CPR / 2L))
    {
        turns[index]++;
    }

    last_angle[index] = feedback->angle_raw;
    feedback->angle_total = turns[index] * DM_ANGLE_CPR + feedback->angle_raw;
}

/* 中断收 */
static void Dm_Read(void *ctx, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    dm_motor_feedback_t *feedback = (dm_motor_feedback_t *)ctx;

    if (feedback == NULL || data == NULL || dlc < 8u)
    {
        return;
    }
    (void)id;


    for (uint8_t i = 0; i < 8u; i++)
    {
        feedback->raw_data[i] = data[i];
    }
    feedback->last_rx_tick = HAL_GetTick();
    feedback->last_rx_ns = Mono_Ns_Get();
    feedback->rx_seen = 1u;
    feedback->raw_pending = 1u;
}

/* 注册表 */
void Dm_Init(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_config_t *config = &dm_motor_config[i];
        FDCAN_HandleTypeDef *handle = Can_Bus_Handle(machine->dm_bus[i]);

        if (handle == NULL
            || config->feedback_id > 0x7FFu || config->control_id > 0x7FFu)
        {
            continue;
        }
        /* 反馈 ID 兼容两种 Master ID: 0x10+ID (表里值) 与 0+ID (等同控制 ID) */
        Can_Bus_Register(handle, config->feedback_id, Dm_Read,
                         &dm_motor_feedback[i]);
        if (config->control_id != config->feedback_id)
        {
            Can_Bus_Register(handle, config->control_id, Dm_Read,
                             &dm_motor_feedback[i]);
        }
    }
}

/* 解码值 */
/*
 * CAN协议反馈先解量程，再按机器表零点/极性转物理量，保存真实接收时刻。
 * 状态、故障和在线由驱动维护；未更新的报文不能冒充新力反馈。
 */
void Dm_Parse(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_feedback_t *feedback = &dm_motor_feedback[i];
        uint8_t raw_data[8];
        uint32_t primask;
        uint64_t rx_ns;

        if (!feedback->raw_pending)
        {
            continue;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        memcpy(raw_data, (const void *)feedback->raw_data, sizeof(raw_data));
        rx_ns = feedback->last_rx_ns;
        feedback->raw_pending = 0u;
        __set_PRIMASK(primask);

        feedback->err_raw    = raw_data[0] >> 4;
        feedback->motor_id   = raw_data[0] & 0x0Fu;
        feedback->angle_raw  = ((uint16_t)raw_data[1] << 8) | raw_data[2];
        feedback->vel_raw    = ((uint16_t)raw_data[3] << 4) | (raw_data[4] >> 4);
        feedback->trq_raw    = ((uint16_t)(raw_data[4] & 0x0Fu) << 8) | raw_data[5];
        feedback->temp_mos   = raw_data[6];
        feedback->temp_rotor = raw_data[7];
        if (machine->dm_sign[i].fb < 0)
        {
            feedback->angle_raw = (uint16_t)(DM_ANGLE_CPR - 1L
                - feedback->angle_raw);
            feedback->vel_raw = (uint16_t)(DM_MIT_FIELD_MAX
                - feedback->vel_raw);
            feedback->trq_raw = (uint16_t)(DM_MIT_FIELD_MAX
                - feedback->trq_raw);
        }
        feedback->pos_rad = Dm_Uint_To_Float(feedback->angle_raw,
            -machine->dm_pos_max, machine->dm_pos_max, 16u);
        feedback->pos_zero_rad = feedback->pos_rad + machine->dm_zero[i];
        feedback->vel_rad_s = Dm_Uint_To_Float(feedback->vel_raw,
            -machine->dm_vel_max, machine->dm_vel_max, 12u);
        feedback->trq_nm = Dm_Uint_To_Float(feedback->trq_raw,
            -machine->dm_trq_max, machine->dm_trq_max, 12u);
        Dm_Update_Angle(i);
        feedback->parsed_rx_ns = rx_ns;
        feedback->parsed_seen = 1u;
    }
}

/* 查在线 */
bool Dm_Is_Online(uint8_t index)
{
    if (index >= DM_MOTOR_NUM)
    {
        return false;
    }
    return dm_motor_feedback[index].rx_seen
        && (HAL_GetTick() - dm_motor_feedback[index].last_rx_tick)
           <= DM_OFFLINE_MS;
}

/* 查使能 */
bool Dm_Is_Enabled(uint8_t index)
{
    if (index >= DM_MOTOR_NUM)
    {
        return false;
    }
    /* 离线时 err_raw 是旧值, 状态未知, 不当作使能 */
    return Dm_Is_Online(index) && dm_motor_feedback[index].err_raw == 1u;
}

/* 查故障 */
bool Dm_Has_Fault(uint8_t index)
{
    uint8_t err;

    if (index >= DM_MOTOR_NUM)
    {
        return false;
    }
    err = dm_motor_feedback[index].err_raw;
    return err >= 0x08u && err <= 0x0Eu;
}

/* 状态命令重发 */
static void Dm_Command_Watchdog(uint8_t state, uint8_t command,
                                uint32_t last_tick[DM_MOTOR_NUM])
{
    uint32_t now;
    uint8_t i;

    now = HAL_GetTick();
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (Dm_Is_Online(i) && dm_motor_feedback[i].err_raw == state
            && now - last_tick[i] >= 100u)
        {
            (void)Dm_Send_Command(i, command);
            last_tick[i] = now;
        }
    }
}

/* 使能重发 */
void Dm_Enable_Watchdog(void)
{
    static uint32_t last_enable_tick[DM_MOTOR_NUM];

    Dm_Command_Watchdog(0u, DM_CMD_ENABLE, last_enable_tick);
}

/* 失能重发 */
void Dm_Disable_Watchdog(void)
{
    static uint32_t last_disable_tick[DM_MOTOR_NUM];

    Dm_Command_Watchdog(1u, DM_CMD_DISABLE, last_disable_tick);
}

/* 发MIT */
HAL_StatusTypeDef Dm_Mit_Control(uint8_t index, uint16_t angle_raw,
                                 uint16_t vel_raw, uint16_t kp_raw,
                                 uint16_t kd_raw, uint16_t trq_raw)
{
    const dm_motor_config_t *config;
    uint8_t data[8];

    if (index >= DM_MOTOR_NUM)
    {
        return HAL_ERROR;
    }
    if (vel_raw > DM_MIT_FIELD_MAX || kp_raw > DM_MIT_FIELD_MAX
        || kd_raw > DM_MIT_FIELD_MAX || trq_raw > DM_MIT_FIELD_MAX)
    {
        return HAL_ERROR;
    }

    config = &dm_motor_config[index];

    data[0] = (uint8_t)(angle_raw >> 8);
    data[1] = (uint8_t)angle_raw;
    data[2] = (uint8_t)(vel_raw >> 4);
    data[3] = (uint8_t)((vel_raw << 4) | (kp_raw >> 8));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4);
    data[6] = (uint8_t)((kd_raw << 4) | (trq_raw >> 8));
    data[7] = (uint8_t)trq_raw;

    return Can_Bus_Transmit(Can_Bus_Handle(machine->dm_bus[index]),
                            config->control_id, data, sizeof(data));
}

/* 发命令 */
HAL_StatusTypeDef Dm_Send_Command(uint8_t index, uint8_t command)
{
    const dm_motor_config_t *config;
    uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu,
                       0xFFu, 0xFFu, 0xFFu, 0x00u};

    if (index >= DM_MOTOR_NUM)
    {
        return HAL_ERROR;
    }
    if (command != DM_CMD_CLEAR_ERROR && command != DM_CMD_ENABLE
        && command != DM_CMD_DISABLE && command != DM_CMD_SET_ZERO)
    {
        return HAL_ERROR;
    }

    config = &dm_motor_config[index];
    data[7] = command;
    return Can_Bus_Transmit(Can_Bus_Handle(machine->dm_bus[index]),
                            config->control_id, data, sizeof(data));
}

/* 广播状态命令 */
static HAL_StatusTypeDef Dm_All_Command(uint8_t command)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t i;

    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (Dm_Send_Command(i, command) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}

/* 全部使能 */
HAL_StatusTypeDef Dm_All_Enable(void)
{
    return Dm_All_Command(DM_CMD_ENABLE);
}

/* 全部失能 */
HAL_StatusTypeDef Dm_All_Disable(void)
{
    return Dm_All_Command(DM_CMD_DISABLE);
}

/* 全部零力矩 */
HAL_StatusTypeDef Dm_Send_Zero(void)
{
    const uint16_t zero_trq_raw = Dm_Float_To_Uint(0.0f,
        -machine->dm_trq_max, machine->dm_trq_max, 12u);
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (Dm_Mit_Control(i, dm_motor_feedback[i].angle_raw,
                           0u, 0u, 0u, zero_trq_raw) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}

/* 全部力矩 */
/*
 * 接收四髋物理力矩，在MIT发送边界按既有输出标定量化并发帧。
 * 控制器不需要再次乘电机极性；返回状态只表示底层发送调用结果。
 */
HAL_StatusTypeDef Dm_Send_Torque(const float torque[DM_MOTOR_NUM])
{
    HAL_StatusTypeDef status = HAL_OK;
    float command_torque;

    if (torque == NULL)
    {
        return HAL_ERROR;
    }
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        command_torque = torque[i];
        if (machine->dm_sign[i].out < 0)
        {
            command_torque = -command_torque;
        }
        uint16_t trq_raw = Dm_Float_To_Uint(command_torque,
            -machine->dm_trq_max, machine->dm_trq_max, 12u);
        if (Dm_Mit_Control(i, DM_MIT_FIELD_MAX,
                           DM_MIT_FIELD_MAX, 0u, 0u, trq_raw) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}
