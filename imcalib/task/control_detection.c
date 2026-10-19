/*
 * 文件职责：组装检测输入并按原顺序调用AIR观测、触地观测和三态选择。
 * 输入为执行拍快照及现有公共上下文；输出为判断、真实落地沿和进入FALLEN事件。
 * 数学算法归air_detection/control_condition；本文件不调用任何恢复或控制处理。
 */
#include "control_detection.h"
#include "air_detection.h"
#include "control_condition.h"
#include "standup.h"
#include <string.h>

uint8_t Control_Detection_Update(control_frame_t *frame)
{
    air_input_t input;
    uint8_t was_flight;
    control_condition_type_t previous;
    uint8_t request;
    uint8_t zero_effort;
    uint8_t normal_running;

    memset(&input, 0, sizeof(input));
    input.imu = &frame->imu;
    input.leg[0] = &frame->left;
    input.leg[1] = &frame->right;
    input.compensation = &frame->compensation;
    input.joint_ready = frame->joint_ready;
    for (uint8_t side = 0u; side < 2u; side++)
    {
        for (uint8_t i = 0u; i < 2u; i++)
        {
            input.torque[side][i] = frame->joint_torque[side * 2u + i];
        }
    }
    was_flight = air_detection.flight;
    Air_Detection_Update(&air_detection, &input);
    frame->landed = (uint8_t)(was_flight && !air_detection.flight && air_detection.valid);

    previous = control_condition.condition;
    request = (uint8_t)(frame->rc.online && frame->rc.mode.rc_enable && frame->rc.mode.strategy == CTRL_STRATEGY_LQR && frame->rc.mode.engage);
    zero_effort = (uint8_t)(!request || !frame->drive || !control_condition.started);
    normal_running = (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_NORMAL && control_condition.normal_ready);
    Control_Condition_Observe(&control_condition, &frame->imu, &frame->left, &frame->right, frame->height, frame->height_valid, zero_effort, normal_running);
    Control_Condition_Select(&control_condition, &air_detection, request, frame->drive, frame->fallen, (uint8_t)(standup_control.phase == STANDUP_DONE));
    return (uint8_t)(control_condition.valid && control_condition.condition == CONTROL_FALLEN && previous != CONTROL_FALLEN);
}
