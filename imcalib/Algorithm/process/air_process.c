/*
 * 文件职责：原AIR后处理的独立执行层实现。
 * 消费本拍三态与落地沿，保留原公共腿长写回临界区、位移/偏航复位及AIR赋值。
 * 不修改检测历史或处理公式；临界区和共享命令写回留在任务层，不进入算法层。
 */
#include "air_process.h"
#include "robot_state.h"
#include "air_detection.h"
#include "control_condition.h"

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
        rc_command.leg_length_target = 0.25f;
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
