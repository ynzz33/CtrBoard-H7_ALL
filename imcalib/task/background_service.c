/*
 * defaultTask每轮调用：发送VOFA诊断、更新WS2812灯效。
 * 此处运行于后台，控制任务通过共享快照提供观测数据。
 */

#include "robot_control.h"
#include "latency_usb.h"
#include "machine_config.h"
#include "vofa_debug.h"
#include "ws2812.h"
#include "dm.h"

/* 后台发送当前VOFA布局并更新灯效；DMA忙时由发送函数跳过。 */
void background_task_body(void)
{
    Vofa_Debug_Send();
#if MACHINE_LATENCY_CAPTURE_ENABLE
    Latency_Usb_Pump();
#endif
    WS2812_RainbowBlink();    
    // for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    // {
    //     Dm_Send_Command(i, DM_CMD_SET_ZERO);
    // }
}

