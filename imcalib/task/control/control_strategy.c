/*
 * 先更新RL状态与公共倒地/AIR选路，再为本拍输出计算一种姿态引用的补偿。
 * 返回本拍策略与刚进入FALLEN事件；AIR估力独立计算实体项；输出缓存只供后续力矩叠加。
 */
#include "control_strategy.h"
#include "robot_state.h"
#include "air_detection.h"
#include "control_detection.h"
#include "control_condition.h"
#include "fallen_process.h"
#include "stair_process.h"
#include <string.h>
#include <math.h>

/* RL只更新自身终止状态，不改写LQR公共倒地/AIR选路。 */
static void Control_RL_Recovery_Update(const control_frame_t *frame)
{
    rl_recovery_state_t recovery = rl_control.recovery;
    rl_recovery_phase_t previous = recovery.phase;
    float gravity[3];
    uint8_t valid;
    uint8_t request = (uint8_t)(frame->rc.online && frame->rc.mode.rc_enable && frame->rc.mode.strategy == CTRL_STRATEGY_RL && frame->rc.mode.engage);
    valid = (uint8_t)(frame->drive_permit && frame->imu.online && RL_Observation_Project_Gravity(frame->imu.quat, gravity));
    RL_Recovery_Update(&recovery, request, valid, valid ? gravity[2] : 0.0f);
    taskENTER_CRITICAL();
    rl_control.recovery = recovery;
    if (recovery.phase != previous)
    {
        RL_Torque_Reset(&rl_control.torque_state);
        Action_State_Clear();
        RL_Observation_Reset(&rl_control.observation); /* 投入/退出/失败不能继承上一段历史。 */
    }
    taskEXIT_CRITICAL();
}

/* 公共选路完成后，按本拍输出路径只请求一种重力引用。
 * AIR在自身估力内部计算实体项，不再向输出模型追加普通重力请求。 */
void Control_Model_Prepare(control_frame_t *frame)
{
    uint8_t enabled = 0u;
    ctrl_strategy_t strategy = frame->rc.mode.strategy;

    if (frame->drive_permit && frame->rc.mode.rc_enable && frame->rc.mode.engage)
    {
        if (strategy == CTRL_STRATEGY_LQR)
        {
            if (Stair_Process_Owns_Output(&stair_process) && !control_condition.valid)
            {
                enabled = LEG_COMP_GRAVITY_WORLD;
            }
            else if (control_condition.valid && control_condition.condition == CONTROL_FALLEN &&
                fallen_process_control.lifecycle.enabled)
            {
                enabled = LEG_COMP_GRAVITY_WORLD;
            }
            else if (control_condition.valid && control_condition.condition != CONTROL_FALLEN)
            {
                enabled = LEG_COMP_GRAVITY;
            }
            if (GAS_SPRING_COMP_ENABLE)
            {
                enabled |= LEG_COMP_SPRING;
            }
        }
        else if (strategy == CTRL_STRATEGY_COMPENSATION && frame->nonfallen_drive_permit)
        {
            enabled = LEG_COMP_SPRING | LEG_COMP_GRAVITY_WORLD;
        }
    }
    Leg_Compensation_Prepare(&frame->compensation, &frame->left, &frame->right, &frame->imu, enabled);
}

/* 执行拍计算前依次更新RL状态、公共选路和所选输出补偿。
 * 返回使能后的策略，entered_fallen输出本拍首次转入FALLEN的事件，供控制器复位。 */
ctrl_strategy_t Control_Strategy_Prepare(control_frame_t *frame,
                                         uint8_t *entered_fallen)
{
    Control_RL_Recovery_Update(frame);
    *entered_fallen = Control_Detection_Update(frame);
    Control_Model_Prepare(frame);
    return frame->rc.mode.rc_enable ? frame->rc.mode.strategy : CTRL_STRATEGY_DISABLE;
}
