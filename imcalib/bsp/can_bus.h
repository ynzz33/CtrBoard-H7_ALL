/*
 * 文件职责：FDCAN公共注册、接收分发与发送服务（can_bus.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：CubeMX FDCAN句柄、注册ID及驱动收发请求。
 * 输出/去向：HAL中断读取帧并派发协议回调；统一构造发送帧、统计接收和错误。
 * 关键边界：本层不解释DM/DJI物理协议或策略意图；HAL_OK仅表明发送入队调用成功。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "main.h"
#include "fdcan.h"
#include "motor_types.h"
#include <stdbool.h>

#define CAN_BUS_NUM         3
#define CAN_BUS_ROUTE_MAX   8
#define CAN_BUS_DEAD_MS     100
#define CAN_BUS_REINIT_MS   100

/* 回调类型 */
typedef void (*can_parse_fn_t)(void *ctx, uint32_t can_id,
                                const uint8_t *data, uint8_t len);

/* 总线状态 */
typedef enum {
    CAN_BUS_STATE_ACTIVE = 0,
    CAN_BUS_STATE_DEAD,
    CAN_BUS_STATE_RECOVERING, /* 本轮启动成功，等待新反馈 */
} can_bus_state_t;

/* 电机固定参 (DM / DJI 共用): 报文 ID (总线按机器选) */



/* 路由条目 */
typedef struct {
    uint32_t       can_id;
    can_parse_fn_t parse;
    void          *ctx;
} can_route_t;

/* 每条总线运行时 */
typedef struct {
    can_route_t           route[CAN_BUS_ROUTE_MAX];
    volatile uint32_t     route_cnt;
    FDCAN_TxHeaderTypeDef tx_template;
    volatile can_bus_state_t state;
    volatile uint32_t     alive_cnt;
    volatile uint32_t     last_rx_id;    /* 最近收到帧的 CAN ID */
    volatile uint32_t     last_tx_tick;
    uint32_t              alive_prev;
    uint32_t              dead_since;
    uint32_t              reinit_tick;
    uint32_t              recover_attempts;
    uint32_t              last_error; /* 最近一次HAL失败的错误，成功后保留 */
    uint8_t               failed_step; /* 0通过/1停止/2初始化/3滤波/4启动/5通知 */
} can_bus_t;

void    Can_Bus_Init(void);
bool    Can_Bus_Register(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                         can_parse_fn_t fn, void *ctx);
HAL_StatusTypeDef Can_Bus_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const uint8_t *data, uint8_t len);
/* 通信任务推进恢复；本轮启动后收到新帧才判在线，RECOVERING仍返回false。 */
bool    Can_Bus_Online(bool expect_traffic);
/* 按总线号取句柄 (1=FDCAN1 2=FDCAN2 3=FDCAN3) */
FDCAN_HandleTypeDef *Can_Bus_Handle(uint8_t bus);
/* 收帧计数 / 最近一帧 ID (调试用) */
uint32_t Can_Bus_Rx_Count(uint8_t bus);
uint32_t Can_Bus_Last_Rx_Id(uint8_t bus);


#endif
