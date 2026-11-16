#ifndef JOINT_SYSID_H
#define JOINT_SYSID_H

#include "robot_control.h"
#include "torque_output.h"

void Joint_Sysid_Update_Mode(const control_frame_t *frame, ctrl_strategy_t strategy);
void Joint_Sysid_Update(const control_frame_t *frame, torque_output_t *torque);
void Joint_Sysid_Record(const control_frame_t *frame,
                        const torque_output_t *requested,
                        const torque_output_t *applied);

#endif
