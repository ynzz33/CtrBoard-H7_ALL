> 2026-10-11：作者已要求清理未使用的VOFA传输标识与切换逻辑，当前VOFA仅经机器配置选定的串口DMA发送。下文USB VOFA流程为历史说明；USB设备基础代码保留，不再接入当前Vofa_Send。现状见vofa-debug.md。

# USB CDC普通VOFA发送（MC02H7）

2026-10-09：独立策略追踪、USB关节试验及宿主策略协议已移除。保留普通VOFA经UART或USB CDC发送，USB接收不再解析控制命令。字段见[vofa-debug.md](vofa-debug.md)。

USB_OTG_HS使用PA11/PA12和内置Full Speed PHY，未切外部ULPI PHY。`Vofa_send.c`是非阻塞发送入口；`vofa_transport.requested/active`上电均选UART。电机失能且上一帧发送完成后，写requested=0切USB、写1回UART；其他值不切换。USB未枚举或忙时跳过本次普通帧，异步发送完成前不覆写帧缓冲。

USB控制器内部DMA沿用DISABLE，以FIFO中断发送。两套链接配置保留专用16KiB非缓存区，VOFA发送缓冲仍在DMA可访问且32字节对齐的AXI SRAM；UART DMA路径继续执行DCache清理。此次不改变USB初始化、内存布局、串口或普通VOFA打包。

历史上机记录仅证明当时USB普通帧可持续接收。此次删除协议后的固件仍需重新烧录核对收包、通道和控制周期，未将编译结果视为实机验收。
