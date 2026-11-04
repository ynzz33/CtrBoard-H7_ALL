/*
 * 每个策略节拍复制最新反馈、遥控、RL执行状态和上一裁剪后动作，组装网络输入并推理。
 * 发布六路裁剪后action前重查模式和reset_count，防止旧控制段的推理覆盖新动作。
 */
#include "robot_control.h"
#include "robot_state.h"
#include "latency_state.h"
#include "output_capture.h"
#include "mono_ns.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include <string.h>
/* 初始化已编译CubeAI网络并检查观测/action尺寸；不在这里开启电机出力。 */
void ctrl_task_init(void)
{
    (void)RL_Policy_Init(&rl_control.policy);
}
/* 复制反馈和last_action→构建观测→推理→发布裁剪后action。
 * reset_count前后不一致或模式已退出则丢弃结果；遥控选择RL可推理，出力另由执行层许可。 */
void ctrl_task_body(void)
{
#if MACHINE_LATENCY_CAPTURE_ENABLE
    latency_record_t trace={0};
    uint64_t build_ns;
    trace.kind=LATENCY_POLICY;trace.begin_ns=Mono_Ns_Get();trace.value[3]=UINT32_MAX;
    trace.value[7]=Latency_Tick_Read(1u).sequence;
#endif
    rl_observation_state_t observation;
    rl_observation_sample_t sample;
    rl_recovery_state_t recovery;
    rc_command_t command;
    float action[RL_ACTION_SIZE] = {0.0f};
    uint32_t reset_count; /* 本次推理所属动作段编号；发布前不一致说明结果已过期。 */
    uint8_t selected;
    uint8_t sampled; /* 是否成功复制执行任务的有效反馈样本。 */
    uint8_t warmup; /* 首帧先发布零动作，下一10ms策略拍才运行网络。 */
    uint8_t built; /* 本次网络观测是否成功构建且全部有限。 */
    /* 样本缓存由高优先级执行任务推进，先在临界区复制，避免读取半更新帧。 */
    taskENTER_CRITICAL();
    selected = (uint8_t)(rc_command.online && rc_command.mode.strategy == CTRL_STRATEGY_RL && rc_command.mode.engage && rl_control.recovery.phase == RL_RECOVERY_NORMAL);
    reset_count = action_state.reset_count;
    if (selected)
    {
#if MACHINE_LATENCY_CAPTURE_ENABLE
        trace.value[0]=trace_sample.sequence;
        trace.value[1]=(uint32_t)(Mono_Ns_Get()-trace.begin_ns);
        trace.value[4]=trace_policy_sequence;
        trace.value[5]=(uint32_t)rc_command.mode.strategy;
        trace.value[6]=(uint32_t)rl_control.recovery.phase;
        trace.reset_count=reset_count;
#endif
        recovery = rl_control.recovery;
        command = rc_command;
        sampled = RL_Observation_Read_Sample(&rl_control.observation, &sample);
        memcpy(observation.last_action, rl_control.observation.last_action, sizeof(observation.last_action));
        memcpy(observation.history, rl_control.observation.history, sizeof(observation.history));
        observation.history_initialized = rl_control.observation.history_initialized;
        observation.warmup_steps = rl_control.observation.warmup_steps;
    }
    else
    {
        sampled = 0u;
        RL_Observation_Reset(&rl_control.observation);
    }
    taskEXIT_CRITICAL();
    if (!selected)
    {
        Action_State_Clear();
        #if MACHINE_LATENCY_CAPTURE_ENABLE
        if(trace.end_ns==0u) { trace.end_ns=Mono_Ns_Get(); }
        Latency_Capture_Push(&trace);
#endif
        return;
    }
    /* 第一份零动作必须实际经过一个策略周期的PD子步，再采第二帧运行网络。 */
    if (observation.history_initialized && observation.warmup_steps < MACHINE_POLICY_DIV / MACHINE_RL_CTRL_DIV)
    {
#if MACHINE_LATENCY_CAPTURE_ENABLE
        trace.end_ns=Mono_Ns_Get();Latency_Capture_Push(&trace);
#endif
        return;
    }
#if MACHINE_LATENCY_CAPTURE_ENABLE
    build_ns=Mono_Ns_Get();
#endif
    warmup = (uint8_t)!observation.history_initialized;
    built = RL_Observation_Build(&observation, &machine->rl, &sample, &command, &recovery, sampled);
#if MACHINE_LATENCY_CAPTURE_ENABLE
    trace.value[2]=(uint32_t)((Mono_Ns_Get()-build_ns)/1000u);
    trace.flags=built ? 2u : 1u;
#endif
    taskENTER_CRITICAL();
    if (reset_count != action_state.reset_count)
    {
#if MACHINE_LATENCY_CAPTURE_ENABLE
        trace.flags=3u;
#endif
        taskEXIT_CRITICAL();
        #if MACHINE_LATENCY_CAPTURE_ENABLE
        if(trace.end_ns==0u) { trace.end_ns=Mono_Ns_Get(); }
        Latency_Capture_Push(&trace);
#endif
        return;
    }
    memcpy(rl_control.observation.obs, observation.obs, sizeof(observation.obs));
    memcpy(rl_control.observation.history, observation.history, sizeof(observation.history));
    rl_control.observation.history_initialized = built ? observation.history_initialized : 0u;
    rl_control.observation.valid = built;
    taskEXIT_CRITICAL();
    if (!built || !rl_control.policy.ready || (!warmup && !RL_Policy_Run(&rl_control.policy, &observation, action)))
    {
        taskENTER_CRITICAL();
        if (reset_count == action_state.reset_count)
        {
            RL_Observation_Reset(&rl_control.observation);
            Action_State_Clear();
        }
        taskEXIT_CRITICAL();
        #if MACHINE_LATENCY_CAPTURE_ENABLE
        if(trace.end_ns==0u) { trace.end_ns=Mono_Ns_Get(); }
        Latency_Capture_Push(&trace);
#endif
        return;
    }
#if MACHINE_LATENCY_CAPTURE_ENABLE
    trace.value[3]=warmup ? UINT32_MAX : rl_control.policy.run_us;
#endif
    /* 动作保持左右两组的网络顺序；首帧为零动作热步，500Hz PD仍正常求解。
     * 推理期间可能退出RL，发布前重查模式，防止新动作覆盖退出时的清零。 */
    taskENTER_CRITICAL();
    if (reset_count == action_state.reset_count && rc_command.online && rc_command.mode.strategy == CTRL_STRATEGY_RL && rc_command.mode.engage && rl_control.recovery.phase == RL_RECOVERY_NORMAL)
    {
        RL_Observation_Set_Last_Action(&rl_control.observation, action);
        memcpy(action_state.a, action, sizeof(action_state.a));
        action_state.last_ok_tick = HAL_GetTick();
        action_state.updated = 1u;
        action_state.rl_ready = 1u;
#if MACHINE_LATENCY_CAPTURE_ENABLE
        trace_policy_published_ns=Mono_Ns_Get();
        trace.correlation=++trace_policy_sequence;trace.flags=4u;
        trace.value[3]=warmup ? UINT32_MAX : rl_control.policy.run_us;trace.end_ns=trace_policy_published_ns;
#endif
    }
    /* 不匹配时直接丢弃；不能用旧推理的失败路径清掉新控制段。 */
    taskEXIT_CRITICAL();
#if MACHINE_LATENCY_CAPTURE_ENABLE
    if(trace.flags!=4u) { trace.flags=3u; }
    if(trace.end_ns==0u) { trace.end_ns=Mono_Ns_Get(); }
    Latency_Capture_Push(&trace);
#endif
}
