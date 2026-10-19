/*
 * 文件职责：执行拍的策略确认与前置模型准备。
 * 输入为同拍反馈快照和上一拍LQR投入状态；输出为本拍策略及进入FALLEN事件。
 * 不修改LQR/RL控制器历史，不生成力矩，不下发电机。
 */
#include "control_strategy.h"
#include "robot_state.h"
#include "air_detection.h"
#include "control_detection.h"
#include "standup.h"
#include <string.h>

/* 恢复仅改变RL许可/观测，不动LQR离地检测。实际投入/退出清历史，模式切换只撤销旧动作就绪。 */
static void Control_RL_Recovery_Update(const control_frame_t *frame)
{
    rl_recovery_state_t recovery;
    rl_recovery_phase_t previous;
    uint8_t request;

    recovery = rl_control.recovery;
    previous = recovery.phase;
    request = (uint8_t)(frame->rc.online && frame->rc.mode.rc_enable && frame->rc.mode.strategy == CTRL_STRATEGY_RL && frame->rc.mode.engage);
    RL_Recovery_Update(&recovery, request, (uint8_t)(frame->drive && frame->height_valid), frame->fallen, frame->height);
    taskENTER_CRITICAL();
    rl_control.recovery = recovery;
    if (recovery.phase != previous)
    {
        if (previous == RL_RECOVERY_IDLE || recovery.phase == RL_RECOVERY_IDLE || recovery.phase == RL_RECOVERY_FAILED)
        {
            RL_Torque_Reset(&rl_control.torque_state);
            Action_State_Clear();
            memset(rl_control.observation.last_action, 0, sizeof(rl_control.observation.last_action));
        }
        else
        {
            /* 保留训练的原始action历史；拒绝跨模式的在途推理，等待新模式首帧。 */
            action_state.rl_ready = 0u;
            action_state.reset_count++;
        }
    }
    taskEXIT_CRITICAL();
}

/* 仅生成本拍模型缓存；是否把补偿加到候选力矩仍由后续控制路径决定。 */
static void Control_Model_Prepare(control_frame_t *frame, uint8_t lqr_running)
{
    uint8_t enabled = 0u;
    ctrl_strategy_t strategy = frame->rc.mode.strategy;

    if (frame->drive && frame->rc.mode.rc_enable && frame->rc.mode.engage)
    {
        if (strategy == CTRL_STRATEGY_LQR)
        {
            enabled = (uint8_t)((frame->normal ? LEG_COMP_GRAVITY : 0u) | (standup_control.enabled ? LEG_COMP_GRAVITY_WORLD : 0u));
            if (GAS_SPRING_COMP_ENABLE || (air_param.enabled && lqr_running && frame->normal && (!standup_control.enabled || standup_control.phase == STANDUP_DONE)))
            {
                enabled |= LEG_COMP_SPRING;
            }
        }
        else if (strategy == CTRL_STRATEGY_COMPENSATION && frame->normal)
        {
            enabled = LEG_COMP_SPRING | LEG_COMP_GRAVITY_WORLD;
        }
    }
    /* 公共观察器需要实体模型，不等待LQR投入或自起完成；命令补偿接入保持原样。 */
    if (air_param.enabled && frame->imu.online && frame->left.output.valid && frame->right.output.valid && frame->joint_ready)
    {
        enabled |= LEG_COMP_SPRING | LEG_COMP_GRAVITY;
    }
    Leg_Compensation_Prepare(&frame->compensation, &frame->left, &frame->right, &frame->imu, enabled);
}

ctrl_strategy_t Control_Strategy_Prepare(control_frame_t *frame,
                                         uint8_t lqr_running,
                                         uint8_t *entered_fallen)
{
    Control_RL_Recovery_Update(frame);
    Control_Model_Prepare(frame, lqr_running);
    *entered_fallen = Control_Detection_Update(frame);
    return frame->rc.mode.rc_enable ? frame->rc.mode.strategy : CTRL_STRATEGY_DISABLE;
}
