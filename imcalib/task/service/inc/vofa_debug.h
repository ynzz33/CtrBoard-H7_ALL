/*
 * 在短临界区复制控制诊断到浮点通道数组，退出临界区后交给Vofa_Send。
 * 有效通道赋值以当前编译分支为准；注释掉的旧布局保留作人工切换参考。
 */

#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

void Vofa_Debug_Send(void);

#endif
