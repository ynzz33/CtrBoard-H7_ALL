/*
 * 定义跨任务共享对象；上电创建执行/推理信号量，装入腿几何并清控制器历史。
 * TIM6经Policy_Tick_Div唤醒策略任务；Action_State_Clear使旧动作及在途推理失效。
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
    imu_state_t imu; /* 本拍IMU测量及有效性，由已发布反馈复制。 */
    leg_state_t left; /* 本拍左腿几何、速度和力映射。 */
    leg_state_t right; /* 本拍右腿几何、速度和力映射。 */
    rc_command_t rc; /* 本拍遥控策略、使能/投入与目标。 */
    float height; /* 两腿髋轮投影高度均值m，不是机身绝对离地高度。 */
    uint8_t height_valid; /* height是否由有效双腿几何求得。 */
    uint8_t fallen;             /* 本拍公共FALLEN归属；RL独立判断倾角与低高度。 */
    float wheel_vel[2]; /* 左右轮轴速度rad/s，顺序左/右。 */
    float motor_pos[4];         /* 本拍DM零点后连续角，rad；RL首次选分支 */
    float motor_vel[4];         /* 本拍固件电机速度，rad/s */
    rl_motor_state_t rl_motor;  /* 当前RL控制子步的模型坐标反馈，供采样与PD共用 */
    uint8_t motor_valid;        /* 源电机组在线且无故障，与虚拟腿几何valid分开 */
    float joint_torque[4]; /* 四髋物理反馈力矩N·m，顺序同DM索引。 */
    leg_compensation_values_t compensation; /* 本拍模型缓存，供AIR估力和补偿叠加分别读取。 */
    uint8_t joint_ready; /* 四髋均在线且报告已使能。 */
    uint8_t drive_permit; /* 本拍反馈、电机和总输出许可满足 */
    uint8_t nonfallen_drive_permit; /* drive_permit且公共姿态未倒地，不代表三态NORMAL */
    uint8_t recovery_permit; /* 本拍LQR地面恢复执行许可，不是恢复阶段 */
    uint8_t landed; /* 本拍有效AIR检测由离地变接地的事件。 */
} control_frame_t;

#ifndef CONTROL_TIME_VOFA_ENABLE
#define CONTROL_TIME_VOFA_ENABLE 0u
#endif

typedef struct {
    uint32_t sequence; /* 时序统计执行次数。 */
    uint32_t period_us; /* 相邻output_task_body开始时刻间隔us。 */
    uint32_t min_period_us; /* 已记录的最小执行间隔us。 */
    uint32_t max_period_us; /* 已记录的最大执行间隔us。 */
    uint32_t run_us; /* 本拍执行函数耗时us。 */
    uint32_t output_sequence; /* 调用最终输出层的次数，不是成功CAN帧数。 */
    uint32_t output_period_us; /* 相邻发送调用所在执行拍的起点间隔us。 */
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
