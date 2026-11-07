/* 十态异步MPC消息桥；仅actuation发布/消费，单个后台任务独占求解上下文。 */
#ifndef TASK_BALANCE_MPC_H
#define TASK_BALANCE_MPC_H
#include "balance_mpc.h"
typedef struct {
    uint32_t age_us,solve_us,model_us,fallback_count,model_version,epoch;
    uint32_t worker_period_us,worker_deadline_count;
    uint16_t iterations,node;
    float primal,dual;
    uint8_t backend,reason,ready,constraint_active;
    mpc_status_t status;
} balance_mpc_debug_t;
extern balance_mpc_debug_t balance_mpc_debug;
void Balance_MPC_Task_Ready(uint8_t ready);
void balance_mpc_task_body(void);
void Balance_MPC_Invalidate(void);
/* 来自上一执行拍最终提交检查；wheel为模型轮矩，Tp为映射摆矩（u_hip=-Tp）。 */
void Balance_MPC_Set_Applied(const float wheel[2],const float Tp[2],uint8_t valid);
/* 返回1表示本拍使用MPC，输出为模型Nm；0保留调用方原LQR候选。 */
uint8_t Balance_MPC_Control(const lqr_state_t *state,const leg_state_t *left,const leg_state_t *right,
    const float force[2],const leg_compensation_values_t *values,const lqr_runtime_config_t *config,
    uint8_t normal_permit,uint64_t state_time_us,float selected[4]);
#endif
