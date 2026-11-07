/* 单线程消息桥回归，临界区由主机空桩替代；不证明RTOS抢占或实际延迟。 */
#include "task_balance_mpc.h"
#ifdef NDEBUG
#error "Host regression requires active assertions"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
static machine_cfg_t config;
const machine_cfg_t *const machine=&config;
static lqr_gain_desc_t gain;
const lqr_gain_desc_t *LQR_Gain_Info(void){return &gain;}
static uint64_t now_us;
static unsigned invalidate_on_clock;
uint64_t Mono_Ns_Get(void)
{
    if(invalidate_on_clock){invalidate_on_clock=0;Balance_MPC_Invalidate();}
    return now_us*1000u;
}
static float host_applied[4];
static uint8_t Host_Control(const lqr_state_t *st,const leg_state_t *left,const leg_state_t *right,
    const float force[2],const leg_compensation_values_t *values,const lqr_runtime_config_t *config,
    uint8_t permit,uint64_t time,float selected[4])
{
    float Tp[2]={-host_applied[2],-host_applied[3]};
    Balance_MPC_Set_Applied(host_applied,Tp,1u);
    uint8_t used=Balance_MPC_Control(st,left,right,force,values,config,permit,time,selected);
    if(permit){memcpy(host_applied,selected,sizeof(host_applied));}
    else{memset(host_applied,0,sizeof(host_applied));}
    return used;
}
int main(void)
{
    lqr_state_t st={0};leg_state_t left={0},right={0};leg_compensation_values_t values={0};lqr_runtime_config_t ctl={0};
    float F[2]={100,100},selected[4];
    const uint64_t node_us=(uint64_t)(BALANCE_MPC_TS*1000000.0f+0.5f);
    const uint64_t delay_us=node_us*BALANCE_MPC_SOLVE_DIV;
    st.valid=st.gain_valid=st.pos_armed=1;st.control_reference_scale=1;
    st.len[0]=.23f;st.len[1]=.25f;st.x[LQR_X_THB]=.03f;
    ctl.pos_hold=ctl.yaw_hold=ctl.yaw_rate_hold=ctl.wheel_enable=ctl.leg_swing_enable=1;
    ctl.trq_max_wheel=4.79f;ctl.trq_max_hip=54;
    left.output.force_valid=right.output.force_valid=1;
    for(unsigned j=0;j<2;j++)
    {
        left.output.force_map[j][0]=right.output.force_map[j][0]=j==0u?.1f:-.1f;
        left.output.force_map[j][1]=right.output.force_map[j][1]=1;
    }
    /* 具体电机界54Nm，F映射为+10/-10Nm，两个髋交集是[-44,44]。 */
    float lower[4],upper[4];
    assert(Balance_MPC_Bounds(&left,&right,F,&values,&ctl,lower,upper)==MPC_OK);
    assert(fabsf(lower[2]+44)<.0001f && fabsf(upper[2]-44)<.0001f);
    left.output.force_map[0][1]=-2;left.output.force_map[1][1]=-2;
    assert(Balance_MPC_Bounds(&left,&right,F,&values,&ctl,lower,upper)==MPC_OK);
    assert(fabsf(lower[2]+22)<.0001f && fabsf(upper[2]-22)<.0001f);
    left.output.force_map[0][1]=0;F[0]=600;
    assert(Balance_MPC_Bounds(&left,&right,F,&values,&ctl,lower,upper)==MPC_INFEASIBLE);
    F[0]=100;left.output.force_map[0][1]=left.output.force_map[1][1]=1;
    left.output.force_valid=0;
    assert(Balance_MPC_Bounds(&left,&right,F,&values,&ctl,lower,upper)==MPC_INVALID);
    left.output.force_valid=1;
    puts("ten-state physical motor bounds signs/zero/infeasible/invalid: PASS");
    Balance_MPC_Task_Ready(1);
    assert(!Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    balance_mpc_task_body();
    now_us=1000;assert(!Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    now_us=delay_us+500;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    assert(balance_mpc_debug.node==0 && balance_mpc_debug.status==MPC_OK);
    balance_mpc_task_body(); /* 新结果尚未激活时，继续消费旧计划。 */
    now_us=delay_us+2500;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    assert(balance_mpc_debug.node==1);
    puts("future activation, pending plan and age-indexed consumption: PASS");
    ctl.trq_max_wheel=.000001f;now_us=delay_us+2600;
    /* 约束收窄后计划输入超界：沿用计划首节点并用本拍界裁剪，不再退回独立LQR。 */
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));assert(balance_mpc_debug.reason==8||balance_mpc_debug.backend==1u);
    for(unsigned i=0;i<2;i++)assert(selected[i]<=.000001f+.00001f && selected[i]>=-.000001f-.00001f);
    ctl.trq_max_wheel=4.79f;st.len[0]=.239f;now_us=delay_us+2700;
    /* 腿长漂移只记诊断，计划继续消费。 */
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));st.len[0]=.23f;
    puts("current input bounds/model age reject result atomically: PASS");
    Balance_MPC_Invalidate();now_us=delay_us+4000;
    assert(!Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    invalidate_on_clock=1;balance_mpc_task_body();
    now_us=2u*delay_us+4500;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    assert(balance_mpc_debug.reason==5);
    puts("in-flight old epoch cannot publish after mode exit: PASS");
    Balance_MPC_Invalidate();now_us=2u*delay_us+6000;
    (void)Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected);balance_mpc_task_body();
    now_us=3u*delay_us+node_us+6500;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));assert(balance_mpc_debug.reason==6);
    puts("late first node rejected: PASS");
    /* 越域只夹模型调度；实测状态保持，下一拍同一夹值可继续消费。 */
    Balance_MPC_Invalidate();now_us=3u*delay_us+10000;st.len[0]=.34f;st.len[1]=.35f;
    (void)Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected);balance_mpc_task_body();
    now_us=4u*delay_us+10500;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    assert(st.len[0]==.34f && st.len[1]==.35f);
    Balance_MPC_Invalidate();now_us=4u*delay_us+12000;st.len[0]=.14f;st.len[1]=.13f;
    (void)Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected);balance_mpc_task_body();
    now_us=5u*delay_us+12500;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    assert(st.len[0]==.14f && st.len[1]==.13f);
    st.len[0]=NAN;assert(!Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));assert(balance_mpc_debug.reason==1);
    st.len[0]=.23f;st.len[1]=.25f;
    puts("LQR-equivalent upper/lower scheduling clamp, raw measurement and NaN rejection: PASS");
    ctl.pos_hold=0;assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));assert(balance_mpc_debug.backend==1u);
    ctl.pos_hold=1;
    Balance_MPC_Task_Ready(0);assert(!Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));assert(balance_mpc_debug.reason==3);
    puts("domain/mask/task-creation fallback: PASS");
    Balance_MPC_Task_Ready(1);Balance_MPC_Invalidate();now_us=5u*delay_us+15000;
    memset(st.x,0,sizeof(st.x));memset(st.target,0,sizeof(st.target));
    st.operating_point[LQR_X_THL]=.0105f;st.operating_point[LQR_X_THR]=.03595f;
    st.x[LQR_X_THL]=st.target[LQR_X_THL]=st.operating_point[LQR_X_THL];
    st.x[LQR_X_THR]=st.target[LQR_X_THR]=st.operating_point[LQR_X_THR];
    uint64_t origin=now_us;
    (void)Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected);balance_mpc_task_body();
    now_us=origin+delay_us+500;
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));
    for(unsigned i=0u;i<4u;i++){assert(fabsf(selected[i])<.0001f);}
    now_us=origin+delay_us+node_us*BALANCE_MPC_N;
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1,now_us,selected));assert(balance_mpc_debug.reason==6);
    puts("producer operating point copied, zero-error plan, exact horizon expiration: PASS");

    Balance_MPC_Invalidate();Balance_MPC_Task_Ready(1u);
    /* 已删除applied确认前置：首拍不再reason9回LQR，而是直接发布快照并用Kinf顶本拍。 */
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1u,now_us,selected));
    assert(balance_mpc_debug.reason==5u);
    float applied_wheel[2]={0.0f,0.0f},applied_Tp[2]={0.0f,0.0f};
    Balance_MPC_Set_Applied(applied_wheel,applied_Tp,1u);
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1u,now_us,selected));
    puts("unconfirmed/failed output invalidates plan; first committed LQR input primes prediction: PASS");
    /* Confirmed physical request must reach the delayed prediction with u_hip=-Tp. */
    Balance_MPC_Invalidate();applied_wheel[0]=.6f;applied_wheel[1]=-.3f;applied_Tp[0]=1.2f;applied_Tp[1]=-.7f;
    Balance_MPC_Set_Applied(applied_wheel,applied_Tp,1u);now_us+=10000u;
    balance_mpc_input_t expected={0};static balance_mpc_context_t expected_context;balance_mpc_solution_t expected_solution;
    memcpy(expected.state,st.x,sizeof(expected.state));memcpy(expected.reference,st.target,sizeof(expected.reference));
    memcpy(expected.x_eq,st.operating_point,sizeof(expected.x_eq));memcpy(expected.length,st.len,sizeof(expected.length));
    memcpy(expected.u_eq,gain.u_eq,sizeof(expected.u_eq));
    for(unsigned j=0;j<10u;j++){expected.reference[j]+=gain.x_eq[j];}
    expected.state[LQR_X_PHI]=expected.x_eq[LQR_X_PHI]=0;
    expected.reference[LQR_X_PHI]=st.control_reference_scale*Angle_Wrap_Pi(expected.reference[LQR_X_PHI]-st.x[LQR_X_PHI]);
    expected.reference[LQR_X_DPHI]*=st.control_reference_scale;
    expected.slope[0]=0.0f;expected.slope[1]=expected.reference[LQR_X_DPHI];
    expected.applied[0]=applied_wheel[0];expected.applied[1]=applied_wheel[1];expected.applied[2]=-applied_Tp[0];expected.applied[3]=-applied_Tp[1];
    assert(Balance_MPC_Bounds(&left,&right,F,&values,&ctl,expected.lower,expected.upper)==MPC_OK);
    expected.epoch=1u;expected.time_us=now_us;
    Balance_MPC_Solve(&expected_context,&expected,&expected_solution);assert(expected_solution.valid);
    assert(Host_Control(&st,&left,&right,F,&values,&ctl,1u,now_us,selected));balance_mpc_task_body();
    now_us+=delay_us+100u;assert(Balance_MPC_Control(&st,&left,&right,F,&values,&ctl,1u,now_us,selected));
    for(unsigned j=0;j<4u;j++){assert(fabsf(selected[j]-expected_solution.input[j])<.0001f);}
    puts("confirmed wheel/Tp input signs and delay prediction match direct solver: PASS");
    return 0;
}
