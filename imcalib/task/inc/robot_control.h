/*
 * 文件职责：共享控制状态定义与全机控制初始化（robot_control.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：机器表、各算法初始化接口、TIM6分频节拍。
 * 输出/去向：初始化几何/LQR/RL/自起/离地与信号量，提供动作清零及任务接口。
 * 关键边界：共享变量在这里定义；策略仲裁和发送在task_actuation；不在初始化中重标定物理量。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "rc_command.h"
#include "feedback_snapshot.h"
#include "machine_config.h"
#include "rl_policy.h"
#include "leg_compensation.h"

/* 执行拍快照 */
typedef struct {
    imu_state_t imu;
    leg_state_t left;
    leg_state_t right;
    rc_command_t rc;
    float height;
    uint8_t height_valid;
    uint8_t fallen;             /* 本拍公共姿态倒地；RL还单独判断低高度。 */
    float wheel_vel[2];
    float motor_pos[4];         /* 本拍DM零点后连续角，rad；RL首次选分支 */
    float motor_vel[4];         /* 本拍固件电机速度，rad/s */
    rl_motor_state_t rl_motor;  /* 当前RL控制子步的模型坐标反馈，供采样与PD共用 */
    uint8_t motor_valid;        /* 源电机组在线且无故障，与虚拟腿几何valid分开 */
    float joint_torque[4];
    leg_compensation_values_t compensation;
    uint8_t joint_ready;
    uint8_t drive_permit; /* 本拍反馈、电机和总输出许可满足 */
    uint8_t nonfallen_drive_permit; /* drive_permit且公共姿态未倒地，不代表三态NORMAL */
    uint8_t recovery_permit; /* 本拍LQR地面恢复执行许可，不是恢复阶段 */
    uint8_t landed;
} control_frame_t;

#ifndef CONTROL_TIME_VOFA_ENABLE
#define CONTROL_TIME_VOFA_ENABLE 0u
#endif

typedef struct {
    uint32_t sequence;
    uint32_t period_us;
    uint32_t min_period_us;
    uint32_t max_period_us;
    uint32_t run_us;
    uint32_t output_sequence;
    uint32_t output_period_us;
} control_time_debug_t;

#define FAULT_NONE    0u
#define FAULT_IMU     0x01u
#define FAULT_RC      0x02u
#define FAULT_MOTOR   0x04u
#define FAULT_CAN     0x08u
#define FAULT_ACTION  0x10u

extern osSemaphoreId ctrl_tick_sem_handle;
extern osSemaphoreId policy_tick_sem_handle;

void Robot_Control_Init(void);
uint8_t Robot_Control_Enable_Allowed(void);
void Action_State_Clear(void);
/* TIM6 节拍分频: 每 MACHINE_POLICY_DIV 拍释放策略节拍 (ISR 调用) */
void Policy_Tick_Div(void);
void imu_task_init(void);
void imu_task_body(void);
void ctrl_task_init(void);
void ctrl_task_body(void);
void output_task_init(void);
void output_task_body(void);
uint8_t output_task_lqr_engaged(void);
uint8_t output_task_rl_engaged(void);
void comm_task_body(void);
void background_task_body(void);

#endif
