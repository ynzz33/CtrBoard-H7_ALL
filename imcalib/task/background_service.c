#include "robot_control.h"
#include "vofa_debug.h"
#include "ws2812.h"

void background_task_body(void)
{
    Vofa_Debug_Tick();
    WS2812_RainbowBlink();
}
