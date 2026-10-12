/*
 * 文件职责：DR16遥控协议校验与在线管理（dr16.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：uart_idle交出的接收字节和时间戳。
 * 输出/去向：解析通道/拨杆/鼠标键盘、缓存有效帧并提供协议快照/在线状态。
 * 关键边界：驱动不决定LQR/RL模式，也不直接控制电机；统一指令转换由rc_command处理。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef __DR16_H
#define __DR16_H

#include "main.h"
#include "rc_types.h"
#include <stdbool.h>

#define DR16_FRAME_LEN   18
#define DR16_OFFLINE_MS  50u




extern dr16_t dr16;

void        DR16_Init(void);
void        DR16_Process(void);
bool        DR16_Online(void);
int16_t     DR16_Deadline(int16_t input, uint16_t deadline);
dr16_t      DR16_Snapshot(void);

#define DR16_SW_LEFT_UP     (dr16.s1 == DR16_SW_UP)
#define DR16_SW_LEFT_MID    (dr16.s1 == DR16_SW_MID)
#define DR16_SW_LEFT_DOWN   (dr16.s1 == DR16_SW_DOWN)
#define DR16_SW_RIGHT_UP    (dr16.s2 == DR16_SW_UP)
#define DR16_SW_RIGHT_MID   (dr16.s2 == DR16_SW_MID)
#define DR16_SW_RIGHT_DOWN  (dr16.s2 == DR16_SW_DOWN)

#endif
