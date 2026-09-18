#include "sysid_mode.h"
#include "robot_control.h"

/* 测试模式: 暂时保持零力矩, 激励序列与采集待接入 */
void Sysid_Mode_Run(void)
{
    output_debug_dm_sent = 0u;
    output_debug_dji_sent = 0u;
    (void)Dm_Send_Zero();
    (void)Dji_All_Stop();
}
