/*
 * 把公共FLIGHT结果接入LQR：选择空中腿摆控制、轴向释放及轮矩处理。
 * 落地时处理腿长目标与控制历史；检测本体在air_detection，本文件执行控制响应。
 */
#include "air_process.h"
#include "robot_state.h"
#include "air_detection.h"
#include "control_condition.h"

/* 把公共FLIGHT接入LQR空中控制/释放标志，并在落地交接时处理腿长目标和控制历史。 */
void Air_Process_Update(control_frame_t *frame)
{
    uint8_t active;
    uint8_t was_active;
    uint8_t precontact;

    was_active = lqr_state.air_control;
    active = (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_FLIGHT && air_param.enabled && air_param.control_enabled && air_detection.flight);
    /* 压缩候选已开始计时：保持AIR选路，只提前接入腿长环吸收落地冲击。 */
    precontact = (uint8_t)(active && air_param.compression_enabled && (air_detection.landing[0].elapsed > 0.0f || air_detection.landing[1].elapsed > 0.0f));
    frame->landed = (uint8_t)(frame->landed && control_condition.valid && air_param.control_enabled && lqr_debug.air_release_enable);
    if (frame->landed)
    {
        taskENTER_CRITICAL();
        rc_command.leg_length_target = 0.2f;
        frame->rc.leg_length_target = rc_command.leg_length_target;
        taskEXIT_CRITICAL();
    }
    if (active != was_active)
    {
        lqr_state.pos = 0.0f;
        lqr_state.pos_armed = 0u;
        lqr_state.yaw_target = lqr_state.x[LQR_X_PHI];
    }
    lqr_state.air_control = active;
    lqr_state.air_release = (uint8_t)(active && lqr_debug.air_release_enable && !precontact);
    lqr_state.air_hip_max = air_param.hip_max;
    air_detection.applied = active;
}
