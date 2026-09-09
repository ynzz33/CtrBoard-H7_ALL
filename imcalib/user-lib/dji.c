#include "dji.h"

/* 固定参 */
const dji_motor_config_t dji_motor_config[DJI_MOTOR_NUM] = {
    [DJI_MOTOR_WHEEL_LFT] = {
        .handle = &hfdcan2,
        .type = DJI_M2006,
        .motor_id = 1u,
        .feedback_id = 0x201u,
        .control_id = 0x200u,
    },
    [DJI_MOTOR_WHEEL_RGT] = {
        .handle = &hfdcan2,
        .type = DJI_M2006,
        .motor_id = 2u,
        .feedback_id = 0x202u,
        .control_id = 0x200u,
    },
};

/* 反馈值 */
dji_motor_feedback_t dji_motor_feedback[DJI_MOTOR_NUM];

/* 转换角度 */
float Dji_Encoder_To_Rad(int32_t encoder_count)
{
    return (float)encoder_count * DJI_RAD_PER_COUNT;
}

/* 转换速度 */
float Dji_Rpm_To_Rad_S(int16_t rpm)
{
    return (float)rpm * DJI_RPM_TO_RAD_S;
}

/* 更新物理量 */
static void Dji_Update_Physical(void)
{
    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        dji_motor_feedback_t *feedback = &dji_motor_feedback[i];
        feedback->angle_rad = Dji_Encoder_To_Rad((int32_t)feedback->angle_raw);
        feedback->angle_total_rad = Dji_Encoder_To_Rad(feedback->angle_total);
        feedback->vel_rad_s = Dji_Rpm_To_Rad_S(feedback->vel_raw);
    }
}

/* 中断收 */
static void Dji_Read(void *ctx, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    dji_motor_feedback_t *feedback = (dji_motor_feedback_t *)ctx;

    if (feedback == NULL || data == NULL || dlc < 8u) return;
    (void)id;

    for (uint8_t i = 0; i < 8u; i++)
        feedback->raw_data[i] = data[i];
    feedback->raw_pending = 1u;
    feedback->last_rx_tick = HAL_GetTick();
}

/* 注册表 */
void Dji_Init(void)
{
    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_config_t *config = &dji_motor_config[i];
        dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

        if (config->handle == NULL || config->motor_id < 1u
            || config->motor_id > 8u || config->feedback_id > 0x7FFu)
            continue;

        Can_Bus_Register(config->handle, config->feedback_id, Dji_Read, feedback);
    }
}

/* 解码值 */
void Dji_Parse(void)
{
    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        dji_motor_feedback_t *feedback = &dji_motor_feedback[i];
        uint8_t raw_data[8];
        uint32_t primask;

        if (!feedback->raw_pending) continue;

        primask = __get_PRIMASK();
        __disable_irq();
        for (uint8_t j = 0; j < 8u; j++)
            raw_data[j] = feedback->raw_data[j];
        feedback->raw_pending = 0u;
        __set_PRIMASK(primask);

        feedback->angle_raw = DJI_FB_ANGLE(raw_data);
        feedback->vel_raw = DJI_FB_VEL(raw_data);
        feedback->current_raw = DJI_FB_CURRENT(raw_data);
        feedback->temp_raw = DJI_FB_TEMP(raw_data);
        feedback->angle_pending = 1u;
    }
    Dji_Circle_Calculate();
    Dji_Update_Physical();
}

/* 多圈值 */
void Dji_Circle_Calculate(void)
{
    static uint16_t last_angle[DJI_MOTOR_NUM];
    static int32_t turns[DJI_MOTOR_NUM];
    static uint32_t inited;

    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        dji_motor_feedback_t *feedback = &dji_motor_feedback[i];
        int32_t delta;

        if (!feedback->angle_pending) continue;
        feedback->angle_pending = 0u;

        if (!(inited & (1UL << i)))
        {
            last_angle[i] = feedback->angle_raw;
            inited |= (1UL << i);
            feedback->angle_total = feedback->angle_raw;
            continue;
        }

        delta = (int32_t)feedback->angle_raw - (int32_t)last_angle[i];
        if (delta > (DJI_ANGLE_CPR / 2L)) turns[i]--;
        else if (delta < -(DJI_ANGLE_CPR / 2L)) turns[i]++;

        last_angle[i] = feedback->angle_raw;
        feedback->angle_total = turns[i] * DJI_ANGLE_CPR + feedback->angle_raw;
    }
}

/* 发电流 */
HAL_StatusTypeDef Dji_Send_Current(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const int16_t current_raw[4])
{
    uint8_t data[8];

    if (hfdcan == NULL || current_raw == NULL || can_id > 0x7FFu)
        return HAL_ERROR;

    for (uint8_t i = 0; i < 4u; i++)
    {
        int16_t raw = current_raw[i];

        data[2u * i] = (uint8_t)(raw >> 8);
        data[2u * i + 1u] = (uint8_t)raw;
    }
    return Can_Bus_Transmit(hfdcan, can_id, data, sizeof(data));
}

/* 全停机 */
HAL_StatusTypeDef Dji_All_Stop(void)
{
    const int16_t zero_current[4] = {0};
    const dji_motor_config_t *config = &dji_motor_config[DJI_MOTOR_WHEEL_LFT];

    return Dji_Send_Current(config->handle, config->control_id, zero_current);
}

/* 查在线 */
bool Dji_Is_Online(uint8_t index)
{
    if (index >= DJI_MOTOR_NUM) return false;
    return (HAL_GetTick() - dji_motor_feedback[index].last_rx_tick)
           <= DJI_OFFLINE_MS;
}
