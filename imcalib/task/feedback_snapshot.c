/*
 * 文件职责：跨任务反馈发布/读取（feedback_snapshot.c）。
 * 输入/来源：imu/comm任务更新后的IMU、电机/腿和遥控结果。
 * 输出/去向：临界区发布，执行/策略任务一次读取同批结构。
 * 关键边界：不同设备仍是异步采样；DM原始接收时刻随电机反馈保留。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#include "feedback_snapshot.h"
#include "robot_state.h"
#include "mono_ns.h"

static feedback_meta_t imu_meta;
static feedback_meta_t motor_meta;
/*
 * 在同一临界区复制IMU、电机、双腿和遥控。
 * 避免读到半更新结构；不同设备仍是异步采样。
 */
uint8_t Robot_Feedback_Read(robot_feedback_t *out)
{
    taskENTER_CRITICAL();
    out->imu = imu_state;
    out->motor = motor_state;
    out->left = leg_l;
    out->right = leg_r;
    out->rc = rc_command;
    out->height = robot_state.height;
    out->height_valid = robot_state.height_valid;
    out->imu_meta = imu_meta;
    out->motor_meta = motor_meta;
    out->captured_ns = Mono_Ns_Get();
    taskEXIT_CRITICAL();
    return 1u;
}

void Robot_Feedback_Publish_Imu(const imu_state_t *next)
{
    taskENTER_CRITICAL();
    imu_state = *next;
    imu_meta.published_ns = Mono_Ns_Get();
    imu_meta.seen = 1u;
    taskEXIT_CRITICAL();
}

/*
 * 把电机状态和由同一批数据解算的左右腿一起发布。
 * 每台电机的CAN接收时刻随motor_state.dm.parsed_rx_ns一起发布，此处不再另存样本。
 */
void Robot_Feedback_Publish_Motor(const motor_state_t *next,
    const leg_state_t *left, const leg_state_t *right)
{
    float height;
    uint8_t height_valid;

    height_valid = Leg_Height_Estimate(left, right, &height);
    taskENTER_CRITICAL();
    motor_state = *next;
    leg_l = *left;
    leg_r = *right;
    robot_state.height = height;
    robot_state.height_valid = height_valid;
    motor_meta.published_ns = Mono_Ns_Get();
    motor_meta.seen = 1u;
    taskEXIT_CRITICAL();
}

void Robot_Feedback_Publish_Rc(const rc_command_t *next)
{
    taskENTER_CRITICAL();
    rc_command = *next;
    taskEXIT_CRITICAL();
}
