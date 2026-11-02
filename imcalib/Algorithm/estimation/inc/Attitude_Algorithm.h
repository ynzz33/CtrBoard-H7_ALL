/*
 * 检查映射后的IMU数值，归一化四元数、将欧拉角转rad并求竖直运动加速度m/s²。
 * 加速度计算按当前机型投影并扣重力；HI229字节解析和机器取轴在调用方完成。
 */

#ifndef ATTITUDE_ALGORITHM_H
#define ATTITUDE_ALGORITHM_H

#include <stdbool.h>
#include "imu_state.h"

void Attitude_Init(imu_state_t *state);
bool Attitude_Update(imu_state_t *state); /* 发布前校验 */
bool Attitude_Accel_Vertical(const imu_state_t *state, float *acceleration);

#endif
