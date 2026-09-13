#include "dm.h"
#include <string.h>

/* 固定参 */
const dm_motor_config_t dm_motor_config[DM_MOTOR_NUM] = {
    [DM_MOTOR_LEG_F_LFT] = {
        .handle = &hfdcan1,
        .type = DM_MOTOR_J4310,
        .feedback_id = 0x11u,
        .control_id = 0x01u,
        .feedback_sign = 1,
    },
    [DM_MOTOR_LEG_B_LFT] = {
        .handle = &hfdcan1,
        .type = DM_MOTOR_J4310,
        .feedback_id = 0x13u,
        .control_id = 0x03u,
        .feedback_sign = 1,
    },
    [DM_MOTOR_LEG_F_RGT] = {
        .handle = &hfdcan3,
        .type = DM_MOTOR_J4310,
        .feedback_id = 0x12u,
        .control_id = 0x02u,
        .feedback_sign = -1,
    },
    [DM_MOTOR_LEG_B_RGT] = {
        .handle = &hfdcan3,
        .type = DM_MOTOR_J4310,
        .feedback_id = 0x14u,
        .control_id = 0x04u,
        .feedback_sign = -1,
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

    if (value < min) value = min;
    if (value > max) value = max;
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
    if (delta > (DM_ANGLE_CPR / 2L)) turns[index]--;
    else if (delta < -(DM_ANGLE_CPR / 2L)) turns[index]++;

    last_angle[index] = feedback->angle_raw;
    feedback->angle_total = turns[index] * DM_ANGLE_CPR + feedback->angle_raw;
}

/* 中断收 */
static void Dm_Read(void *ctx, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    dm_motor_feedback_t *feedback = (dm_motor_feedback_t *)ctx;

    if (feedback == NULL || data == NULL || dlc < 8u) return;
    (void)id;

    for (uint8_t i = 0; i < 8u; i++)
        feedback->raw_data[i] = data[i];
    feedback->last_rx_tick = HAL_GetTick();
    feedback->raw_pending = 1u;
}

/* 注册表 */
void Dm_Init(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_config_t *config = &dm_motor_config[i];

        if (config->handle == NULL || config->type >= DM_MOTOR_TYPE_NUM
            || config->feedback_id > 0x7FFu || config->control_id > 0x7FFu)
            continue;

        Can_Bus_Register(config->handle, config->feedback_id, Dm_Read,
                         &dm_motor_feedback[i]);
    }
}

/* 解码值 */
void Dm_Parse(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_config_t *config = &dm_motor_config[i];
        dm_motor_feedback_t *feedback = &dm_motor_feedback[i];
        uint8_t raw_data[8];
        uint32_t primask;

        if (!feedback->raw_pending) continue;

        primask = __get_PRIMASK();
        __disable_irq();
        memcpy(raw_data, (const void *)feedback->raw_data, sizeof(raw_data));
        feedback->raw_pending = 0u;
        __set_PRIMASK(primask);

        feedback->err_raw = raw_data[0] >> 4;
        feedback->motor_id = raw_data[0] & 0x0Fu;
        feedback->angle_raw = ((uint16_t)raw_data[1] << 8) | raw_data[2];
        feedback->vel_raw = ((uint16_t)raw_data[3] << 4) | (raw_data[4] >> 4);
        feedback->trq_raw = ((uint16_t)(raw_data[4] & 0x0Fu) << 8) | raw_data[5];
        feedback->temp_mos = raw_data[6];
        feedback->temp_rotor = raw_data[7];
        if (config->feedback_sign < 0)
        {
            feedback->angle_raw = (uint16_t)(DM_ANGLE_CPR - 1L
                - feedback->angle_raw);
            feedback->vel_raw = (uint16_t)(DM_MIT_FIELD_MAX
                - feedback->vel_raw);
            feedback->trq_raw = (uint16_t)(DM_MIT_FIELD_MAX
                - feedback->trq_raw);
        }
        feedback->pos_rad = Dm_Uint_To_Float(feedback->angle_raw,
            DM_MIT_POS_MIN, DM_MIT_POS_MAX, 16u);
        feedback->vel_rad_s = Dm_Uint_To_Float(feedback->vel_raw,
            DM_MIT_VEL_MIN, DM_MIT_VEL_MAX, 12u);
        feedback->trq_nm = Dm_Uint_To_Float(feedback->trq_raw,
            DM_MIT_TRQ_MIN, DM_MIT_TRQ_MAX, 12u);
        Dm_Update_Angle(i);
    }
}

/* 查在线 */
bool Dm_Is_Online(uint8_t index)
{
    if (index >= DM_MOTOR_NUM) return false;
    return (HAL_GetTick() - dm_motor_feedback[index].last_rx_tick)
           <= DM_OFFLINE_MS;
}

/* 发MIT */
HAL_StatusTypeDef Dm_Mit_Control(uint8_t index, uint16_t angle_raw,
                                 uint16_t vel_raw, uint16_t kp_raw,
                                 uint16_t kd_raw, uint16_t trq_raw)
{
    const dm_motor_config_t *config;
    uint8_t data[8];

    if (index >= DM_MOTOR_NUM) return HAL_ERROR;
    if (vel_raw > DM_MIT_FIELD_MAX || kp_raw > DM_MIT_FIELD_MAX
        || kd_raw > DM_MIT_FIELD_MAX || trq_raw > DM_MIT_FIELD_MAX)
        return HAL_ERROR;

    config = &dm_motor_config[index];

    data[0] = (uint8_t)(angle_raw >> 8);
    data[1] = (uint8_t)angle_raw;
    data[2] = (uint8_t)(vel_raw >> 4);
    data[3] = (uint8_t)((vel_raw << 4) | (kp_raw >> 8));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4);
    data[6] = (uint8_t)((kd_raw << 4) | (trq_raw >> 8));
    data[7] = (uint8_t)trq_raw;

    return Can_Bus_Transmit(config->handle, config->control_id, data, sizeof(data));
}

/* 发命令 */
HAL_StatusTypeDef Dm_Send_Command(uint8_t index, uint8_t command)
{
    const dm_motor_config_t *config;
    uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu,
                       0xFFu, 0xFFu, 0xFFu, 0x00u};

    if (index >= DM_MOTOR_NUM) return HAL_ERROR;
    if (command != DM_CMD_CLEAR_ERROR && command != DM_CMD_ENABLE
        && command != DM_CMD_DISABLE && command != DM_CMD_SET_ZERO)
        return HAL_ERROR;

    config = &dm_motor_config[index];
    data[7] = command;
    return Can_Bus_Transmit(config->handle, config->control_id, data, sizeof(data));
}

/* 全部使能 */
HAL_StatusTypeDef Dm_All_Enable(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (Dm_Send_Command(i, DM_CMD_ENABLE) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}

/* 全部失能 */
HAL_StatusTypeDef Dm_All_Disable(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (Dm_Send_Command(i, DM_CMD_DISABLE) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}

/* 全部零力矩 */
HAL_StatusTypeDef Dm_Send_Zero(void)
{
    const uint16_t zero_trq_raw = Dm_Float_To_Uint(0.0f,
        DM_MIT_TRQ_MIN, DM_MIT_TRQ_MAX, 12u);
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
HAL_StatusTypeDef Dm_Send_Torque(const float torque[DM_MOTOR_NUM])
{
    HAL_StatusTypeDef status = HAL_OK;

    if (torque == NULL)
    {
        return HAL_ERROR;
    }
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        uint16_t trq_raw = Dm_Float_To_Uint(torque[i],
            DM_MIT_TRQ_MIN, DM_MIT_TRQ_MAX, 12u);
        if (Dm_Mit_Control(i, dm_motor_feedback[i].angle_raw,
                           0u, 0u, 0u, trq_raw) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}
