/*
 * 文件职责：默认任务后台入口（background_service.c）。
 * 输入/来源：FreeRTOS defaultTask周期调用。
 * 输出/去向：调用普通VOFA发送与WS2812灯效。
 * 关键边界：通信/控制计算不放在此低优先级后台；USB初始化仍由defaultTask入口执行。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "robot_control.h"
#include "vofa_debug.h"
#include "jetson_uart.h"
#include "ws2812.h"
#include "dm.h"

void background_task_body(void)
{
    Vofa_Debug_Send();
    Jetson_Mirror_Send();
    WS2812_RainbowBlink();    
    // for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    // {
    //     Dm_Send_Command(i, DM_CMD_SET_ZERO);
    // }
}
 