/*
 * 文件职责：利用重力方向区分正常/倒置的俯仰补角（pitch_world.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：已由Attitude_Update验证的欧拉pitch和四元数。
 * 输出/去向：返回[-pi,pi)内pitch_world及成功标志，供自起/跌倒相关处理。
 * 关键边界：先按最大分量缩放再归一化；重力z决定补角；不重写LQR原pitch或IMU零点。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

// 世界俯仰头文件
// 版本：v1.2
#ifndef PITCH_WORLD_H
#define PITCH_WORLD_H

#include <stdbool.h>

bool Pitch_World_Calc(float pitch, const float quat[4], float *pitch_world);

#endif
