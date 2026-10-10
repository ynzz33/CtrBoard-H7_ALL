#include "feedback_snapshot.h"
#include "robot_state.h"
#include "mono_ns.h"

#include <string.h>

static struct {
    feedback_meta_t imu;
    feedback_meta_t motor;
    feedback_meta_t rc;
    feedback_sample_t imu_sample;
    feedback_sample_t motor_sample[DM_MOTOR_NUM + DJI_MOTOR_NUM];
    feedback_sample_t rc_sample;
} metadata;

static void Feedback_Published(feedback_meta_t *meta)
{
    meta->sequence++;
    meta->published_ns = Mono_Ns_Get();
    meta->seen = 1u;
}

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
    out->imu_meta = metadata.imu;
    out->motor_meta = metadata.motor;
    out->rc_meta = metadata.rc;
    out->imu_sample = metadata.imu_sample;
    memcpy(out->motor_sample, metadata.motor_sample, sizeof(out->motor_sample));
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

void Robot_Feedback_Publish_Motor(const motor_state_t *next,
    const leg_state_t *left, const leg_state_t *right,
    const feedback_sample_t sample[DM_MOTOR_NUM + DJI_MOTOR_NUM])
{
    taskENTER_CRITICAL();
    motor_state = *next;
    leg_l = *left;
    leg_r = *right;
    memcpy(metadata.motor_sample, sample, sizeof(metadata.motor_sample));
    Feedback_Published(&metadata.motor);
    taskEXIT_CRITICAL();
}

void Robot_Feedback_Publish_Rc(const rc_command_t *next,
    uint8_t rc_enable, uint8_t mode, feedback_sample_t sample)
{
    taskENTER_CRITICAL();
    rc_command = *next;
    robot_state.rc_enable = rc_enable;
    input_command.mode = mode;
    if (sample.seen)
    {
        metadata.rc_sample = sample;
    }
    Feedback_Published(&metadata.rc);
    taskEXIT_CRITICAL();
}
