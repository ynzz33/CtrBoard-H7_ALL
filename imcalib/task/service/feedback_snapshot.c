/*
 * IMU任务发布姿态，通信任务成批发布电机/双腿/投影高度并发布遥控命令。
 * 读取时在临界区复制这些最新结构，防止半更新；设备实际采样时刻仍然不同。
 */

#include "feedback_snapshot.h"
#include "robot_state.h"
#include "mono_ns.h"

static feedback_meta_t imu_meta;
static feedback_meta_t motor_meta;
/* 在一个临界区复制最新IMU、电机、左右腿、遥控和高度，返回1。
 * captured_ns是复制时刻；各设备采样异步，published_ns是软件发布时间。 */
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

/* 原子替换IMU共享状态并记录发布时间ns；seen表示已有发布，不保证姿态有效。 */
void Robot_Feedback_Publish_Imu(const imu_state_t *next)
{
    taskENTER_CRITICAL();
    imu_state = *next;
    imu_meta.published_ns = Mono_Ns_Get();
    imu_meta.seen = 1u;
    taskEXIT_CRITICAL();
}

/* 先由双腿计算投影高度，再在同一临界区发布电机、双腿和高度及其有效性。 */
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

/* 在临界区整体发布遥控目标，避免使能/策略/数值来自不同次更新。 */
void Robot_Feedback_Publish_Rc(const rc_command_t *next)
{
    taskENTER_CRITICAL();
    rc_command = *next;
    taskEXIT_CRITICAL();
}
