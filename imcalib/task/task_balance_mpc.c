/* 异步十态消息所有权与时效门控，不复用RL的观测/动作缓冲。
 * 1kHz路径只复制快照、检验当拍约束和消费节点；矩阵更新/求解全部归后台。 */
#include "task_balance_mpc.h"
#include "machine_config.h"
#include "mono_ns.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
static balance_mpc_input_t mailbox;
static balance_mpc_solution_t published;
static balance_mpc_solution_t active_plan; /* actuation独占，不因尚未到激活时间的新结果丢旧序列。 */
static balance_mpc_context_t worker;
static float applied_input[4]; /* 上一执行拍确认提交、未被最终限矩改变的模型输入。 */
static uint8_t applied_valid;
static volatile uint32_t epoch=1u,sample;
static volatile uint8_t active,ready;
static uint32_t processed_sample,adopted_sample;
static uint64_t previous_worker_start;
static uint32_t previous_worker_epoch;
balance_mpc_debug_t balance_mpc_debug;

void Balance_MPC_Task_Ready(uint8_t value)
{
    ready=value;balance_mpc_debug.ready=value;
}
void Balance_MPC_Invalidate(void)
{
    applied_valid=0u;
    taskENTER_CRITICAL();
    if(active){active=0u;epoch++;published.valid=0u;}
    taskEXIT_CRITICAL();
    adopted_sample=0u;active_plan.valid=0u;balance_mpc_debug.backend=0u;
}
void Balance_MPC_Set_Applied(const float wheel[2],const float Tp[2],uint8_t valid)
{
    if(!valid){Balance_MPC_Invalidate();return;}
    applied_input[0]=wheel[0];applied_input[1]=wheel[1];
    applied_input[2]=-Tp[0];applied_input[3]=-Tp[1];
    applied_valid=1u;
}
void balance_mpc_task_body(void)
{
    balance_mpc_input_t in;
    taskENTER_CRITICAL();
    uint8_t run=(uint8_t)(active && ready && mailbox.epoch==epoch && mailbox.sample!=processed_sample);
    if(run)in=mailbox;
    taskEXIT_CRITICAL();
    if(!run)return;
    uint64_t begin=Mono_Ns_Get()/1000u;
    balance_mpc_solution_t result;
    Balance_MPC_Solve(&worker,&in,&result);
    processed_sample=in.sample;
    taskENTER_CRITICAL();
    if(active && epoch==in.epoch)
    {
        published=result;
        balance_mpc_debug.worker_period_us=previous_worker_epoch==in.epoch ? (uint32_t)(begin-previous_worker_start) : 0u;
        if(result.solve_us>(uint32_t)(BALANCE_MPC_TS*1000000.0f+0.5f)*BALANCE_MPC_SOLVE_DIV){balance_mpc_debug.worker_deadline_count++;}
    }
    taskEXIT_CRITICAL();
    previous_worker_start=begin;previous_worker_epoch=in.epoch;
}
static uint8_t Balance_MPC_Fallback(uint8_t reason)
{
    balance_mpc_debug.backend=2u;balance_mpc_debug.reason=reason;balance_mpc_debug.fallback_count++;
    return 0u;
}
/* 兜底优先沿用上一拍计划的首节点：输出仍是同一条MPC轨迹，不会像重新做单步反馈
 * 那样与规划结果跳开。只有计划本身失效时才退到同模型末节点K单步反馈。 */
static uint8_t Balance_MPC_Hold(const balance_mpc_input_t *in,uint8_t reason,float selected[4])
{
    if(active_plan.valid && active_plan.epoch==in->epoch)
    {
        for(unsigned i=0u;i<4u;i++)
        {
            float value=active_plan.input[i];
            if(!isfinite(value)){return Balance_MPC_Fallback(reason);}
            selected[i]=clampf(value,in->lower[i],in->upper[i]);
        }
        balance_mpc_debug.backend=2u;balance_mpc_debug.reason=reason;balance_mpc_debug.fallback_count++;
        return 1u;
    }
    for(unsigned i=0u;i<4u;i++)
    {
        float sum=0.0f;
        for(unsigned j=0u;j<10u;j++)sum+=worker.K[((BALANCE_MPC_N-1u)*4u+i)*10u+j]*(in->state[j]-in->x_eq[j]);
        sum=-sum*balance_mpc_input_scale[i]+in->u_eq[i];
        if(!isfinite(sum)){return Balance_MPC_Fallback(reason);}
        selected[i]=clampf(sum,in->lower[i],in->upper[i]);
    }
    balance_mpc_debug.backend=2u;balance_mpc_debug.reason=reason;balance_mpc_debug.fallback_count++;
    return 1u;
}
uint8_t Balance_MPC_Control(const lqr_state_t *st,const leg_state_t *left,const leg_state_t *right,
    const float force[2],const leg_compensation_values_t *values,const lqr_runtime_config_t *config,
    uint8_t normal_permit,uint64_t state_time_us,float selected[4])
{
    memcpy(selected,st->u,4u*sizeof(float));
    if(!MACHINE_BALANCE_USE_MPC || !normal_permit || st->air_control || st->air_release || !st->valid || !st->gain_valid)
    {
        Balance_MPC_Invalidate();balance_mpc_debug.reason=0u;return 0u;
    }
    if(MACHINE_DEFAULT!=MACHINE_ID_BIG_WHEELLEG || !isfinite(st->len[0]) || !isfinite(st->len[1]))
    {
        Balance_MPC_Invalidate();return Balance_MPC_Fallback(1u); /* 非有限测量不能用于模型调度。 */
    }
    if(!ready)return Balance_MPC_Fallback(3u);
    balance_mpc_input_t in={0};
    mpc_status_t bounds=Balance_MPC_Bounds(left,right,force,values,config,in.lower,in.upper);
    if(bounds!=MPC_OK)
    {
        Balance_MPC_Invalidate();balance_mpc_debug.status=bounds;return Balance_MPC_Fallback(4u);
    }
    const lqr_gain_desc_t *gain=LQR_Gain_Info();
    memcpy(in.state,st->x,sizeof(in.state));memcpy(in.u_eq,gain->u_eq,sizeof(in.u_eq));
    memcpy(in.x_eq,st->operating_point,sizeof(in.x_eq));
    /* 与现役LQR一致：仅夹模型参数，保持实测状态、几何和输出约束原值。 */
    for(unsigned i=0u;i<2u;i++)
    {
        in.length[i]=clampf(st->len[i],balance_mpc_grid[0],balance_mpc_grid[BALANCE_MPC_GRID_N-1]);
    }
    memcpy(in.applied,applied_input,sizeof(in.applied));
    for(unsigned i=0;i<10;i++)in.reference[i]=st->target[i]+gain->x_eq[i];
    /* yaw局部原点与原LQR同一最短角误差；不缩放绝对测量角。 */
    in.state[LQR_X_PHI]=0.0f;in.x_eq[LQR_X_PHI]=0.0f;
    in.reference[LQR_X_PHI]=st->control_reference_scale*Angle_Wrap_Pi(in.reference[LQR_X_PHI]-st->x[LQR_X_PHI]);
    in.reference[LQR_X_DPHI]*=st->control_reference_scale;
    /* 窗口内速度参考保持当前目标，不按命令值继续爬升；否则超速时MPC仍认为未达参考而不刹车。 */
    in.slope[0]=0.0f;in.slope[1]=in.reference[LQR_X_DPHI];
    in.time_us=state_time_us;
    taskENTER_CRITICAL();
    if(!active){epoch++;active=1u;published.valid=0u;}
    in.epoch=epoch;in.sample=++sample;mailbox=in;
    balance_mpc_solution_t result=published;
    taskEXIT_CRITICAL();
    balance_mpc_debug.epoch=in.epoch;
    balance_mpc_debug.status=result.status;balance_mpc_debug.iterations=result.iterations;
    balance_mpc_debug.primal=result.primal;balance_mpc_debug.dual=result.dual;
    balance_mpc_debug.solve_us=result.solve_us;balance_mpc_debug.model_us=result.model_us;
    balance_mpc_debug.model_version=result.model_version;
    uint64_t now=Mono_Ns_Get()/1000u;
    uint32_t node_us=(uint32_t)(BALANCE_MPC_TS*1000000.0f+0.5f);
    if(result.epoch==in.epoch && result.sample!=adopted_sample && result.sample!=0u)
    {
        if(!result.valid){active_plan.valid=0u;return Balance_MPC_Hold(&in,5u,selected);}
        if(now>=result.time_us)
        {
            if(now-result.time_us>=node_us)return Balance_MPC_Hold(&in,6u,selected);
            active_plan=result;adopted_sample=result.sample;
        }
    }
    if(!active_plan.valid || active_plan.epoch!=in.epoch)return Balance_MPC_Hold(&in,5u,selected);
    result=active_plan;
    /* 成功出力时诊断对应实际消费的计划，而非尚未激活的后台新结果。 */
    balance_mpc_debug.status=result.status;balance_mpc_debug.iterations=result.iterations;
    balance_mpc_debug.primal=result.primal;balance_mpc_debug.dual=result.dual;
    balance_mpc_debug.solve_us=result.solve_us;balance_mpc_debug.model_us=result.model_us;
    balance_mpc_debug.model_version=result.model_version;
    uint64_t age=now-result.time_us;
    balance_mpc_debug.age_us=age>UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    if(age>=(uint64_t)node_us*BALANCE_MPC_N)return Balance_MPC_Hold(&in,6u,selected);
    unsigned node=(unsigned)(age/node_us);
    /* 腿长漂移超过模型门槛：不再丢弃计划，照常消费当前节点，只把漂移计入诊断。
     * 后台每4ms按新腿长重建模型，5mm左右的慢漂移由重建吸收，不必中断轨迹。 */
    if(fabsf(result.length[0]-in.length[0])>BALANCE_MPC_MODEL_DELTA || fabsf(result.length[1]-in.length[1])>BALANCE_MPC_MODEL_DELTA){balance_mpc_debug.reason=7u;}
    float candidate[4];
    for(unsigned i=0;i<4;i++)
    {
        float value=result.input[node*4u+i];
        if(!isfinite(value))return Balance_MPC_Hold(&in,8u,selected);
        candidate[i]=clampf(value,in.lower[i],in.upper[i]);
    }
    memcpy(selected,candidate,sizeof(candidate));
    balance_mpc_debug.node=(uint16_t)node;balance_mpc_debug.backend=1u;balance_mpc_debug.reason=0u;
    balance_mpc_debug.constraint_active=result.constraint_active;
    return 1u;
}
