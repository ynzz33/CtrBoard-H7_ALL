/*
 * 用四元数重力朝向为欧拉俯仰选择正立/倒置分支，得到[-pi,pi)范围的pitch_world。
 * 返回false表示无法计算；输出用于恢复姿态及对应补偿，不替换LQR原欧拉俯仰。
 */

// 世界俯仰头文件
// 版本：v1.2
#ifndef PITCH_WORLD_H
#define PITCH_WORLD_H

#include <stdbool.h>

bool Pitch_World_Calc(float pitch, const float quat[4], float *pitch_world);

#endif
