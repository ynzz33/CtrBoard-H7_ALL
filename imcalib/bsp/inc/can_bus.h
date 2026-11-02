/*
 * 管理三路FDCAN的ID回调、发送队列与故障恢复；接收中断按ID派发到电机驱动缓存。
 * bus-off恢复后等待新RX再恢复ACTIVE；共享硬复位会影响三路，发送HAL_OK不是总线执行回执。
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
#define CAN_BUS_HARD_RESET_MS 1000u /* 全部已登记总线离线时，共享硬件复位的最小间隔 */

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
    uint32_t              hardware_resets; /* 运行期共享FDCAN硬件复位次数 */
    uint32_t              last_error; /* 最近一次HAL失败的错误，成功后保留 */
    uint8_t               failed_step; /* 0通过/1停止/2初始化/3滤波/4启动/5通知 */
} can_bus_t;

void    Can_Bus_Init(void);
/* 共享外设复位：启动时须在任何MX_FDCANx_Init之前调用；运行期由CAN层统一协调。 */
void    Can_Bus_Reset_Hardware(void);
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
