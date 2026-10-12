/*
 * 文件职责：姿态归一化与竖直加速度计算（Attitude_Algorithm.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：已由task_imu按机器表映射的四元数、欧拉角和加速度。
 * 输出/去向：输出归一化姿态、rad角度和世界竖直加速度，供LQR/离地估计使用。
 * 关键边界：本文件不解析HI229字节；两机竖直映射沿用源码，不在此重新标定符号。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef ATTITUDE_ALGORITHM_H
#define ATTITUDE_ALGORITHM_H

#include <stdbool.h>
#include "imu_state.h"

void Attitude_Init(imu_state_t *state);
bool Attitude_Update(imu_state_t *state);
bool Attitude_Accel_Vertical(const imu_state_t *state, float *acceleration);

#endif
