/*
 * 文件职责：普通VOFA状态快照与39通道打包（vofa_debug.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：共享反馈/控制/离地/补偿诊断及驱动记录。
 * 输出/去向：直接构造普通39通道JustFloat帧，由后台循环与DMA忙状态安排发送。
 * 关键边界：只读观测，不推进检测或修改目标；异步来源同帧不等于同一物理采样时刻，通道表见md/file-guide.md#vofa。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

void Vofa_Debug_Send(void);

#endif
