/*
 * 文件职责：跨任务反馈发布/读取与元数据（feedback_snapshot.c）。
 * 输入/来源：imu/comm任务更新后的IMU、电机/腿和遥控结果。
 * 输出/去向：临界区发布，执行/策略任务一次读取结构与接收/发布时间戳。
 * 关键边界：发布时刻不等于传感器采样或CAN接收时刻；sequence表示软件发布，旧样本不能伪装成新反馈。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 * TODO: 死代码 —— 本文件 metadata / Feedback_Published 与 robot_feedback_t 的
 *   *_meta / *_sample / captured_ns 全链无消费者，详见 Robot_Feedback_Read 内标记。
 */

#include "feedback_snapshot.h"
#include "robot_state.h"
#include "mono_ns.h"

static struct {
    feedback_meta_t imu;
    feedback_meta_t motor;
    feedback_meta_t rc;
    feedback_sample_t imu_sample;
    feedback_sample_t rc_sample;
} metadata;

static void Feedback_Published(feedback_meta_t *meta)
{
    meta->sequence++;
    meta->published_ns = Mono_Ns_Get();
    meta->seen = 1u;
}

/*
 * 在同一临界区复制IMU、电机、双腿、遥控和各类接收/发布元数据。
 * 避免读到半更新结构；不同设备仍是异步采样，captured_ns只表示此次软件快照时刻。
 */
uint8_t Robot_Feedback_Read(robot_feedback_t *out)
{
    if (out == NULL)
    {
        return 0u;
    }
    taskENTER_CRITICAL();
    out->imu = imu_state;
    out->motor = motor_state;
    out->left = leg_l;
    out->right = leg_r;
    out->rc = rc_command;
    /*
     * ============================ TODO: 死代码 ============================
     * 下面 6 行搬的是"样本来源/数据龄"元数据，从设立至今【全工程没有任何读者】。
     * 死链清单：robot_feedback_t 的 {imu,motor,rc}_meta / {imu,rc}_sample / captured_ns
     *   + 本文件 metadata 结构体、Feedback_Published()
     *   + Publish_Imu / Publish_Rc 里的 if (sample.seen) 块
     *   + feedback_meta_t / feedback_sample_t 两个类型。
     * 2026-10-11 作者决定【暂不删除】：这是刻意预留的"新样本/数据龄判别"能力
     * （md/module-boundaries.md「公共反馈快照」一节），不是被淘汰的残留；
     * 开销≈0（6 次小结构体赋值）。将来若仍无人用，删除需一并改
     * feedback_snapshot.c/h、task_imu.c、task_comm.c 四个文件。
     * ====================================================================
     */
    out->imu_meta = metadata.imu;
    out->motor_meta = metadata.motor;
    out->rc_meta = metadata.rc;
    out->imu_sample = metadata.imu_sample;
    out->rc_sample = metadata.rc_sample;
    out->captured_ns = Mono_Ns_Get();
    taskEXIT_CRITICAL();
    return 1u;
}

void Robot_Feedback_Publish_Imu(const imu_state_t *next, feedback_sample_t sample)
{
    taskENTER_CRITICAL();
    imu_state = *next;
    if (sample.seen)
    {
        metadata.imu_sample = sample;
    }
    Feedback_Published(&metadata.imu);
    taskEXIT_CRITICAL();
}

/*
 * 把电机状态和由同一批数据解算的左右腿一起发布。
 * 每台电机的CAN接收时刻随motor_state.dm.parsed_rx_ns一起发布，此处不再另存样本。
 */
void Robot_Feedback_Publish_Motor(const motor_state_t *next,
    const leg_state_t *left, const leg_state_t *right)
{
    taskENTER_CRITICAL();
    motor_state = *next;
    leg_l = *left;
    leg_r = *right;
    Feedback_Published(&metadata.motor);
    taskEXIT_CRITICAL();
}

void Robot_Feedback_Publish_Rc(const rc_command_t *next,
    uint8_t rc_enable, feedback_sample_t sample)
{
    taskENTER_CRITICAL();
    rc_command = *next;
    robot_state.rc_enable = rc_enable;
    if (sample.seen)
    {
        metadata.rc_sample = sample;
    }
    Feedback_Published(&metadata.rc);
    taskEXIT_CRITICAL();
}
