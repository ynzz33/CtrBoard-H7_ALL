/* 原10态LPV模型双线性插值，重新构建同模型有限因子，不插值旧K/Hinv。
 * 输入以1/sqrt(R)归一化；代价与物理Q/R等价。终端为插值物理DARE的1.01倍。 */
#include "balance_mpc.h"
#include "machine_config.h"
#include "mono_ns.h"
#include <math.h>
#include <string.h>
#if defined(__CC_ARM)
#pragma O2
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif
mpc_status_t Balance_MPC_Model(balance_mpc_context_t *c,const float length[2])
{
    const unsigned n=BALANCE_MPC_GRID_N;
    unsigned idx[2]={0,0};float fraction[2];
    for (unsigned side=0;side<2;side++)
    {
        if (!isfinite(length[side]) || length[side]<balance_mpc_grid[0] || length[side]>balance_mpc_grid[n-1]) return MPC_INVALID;
        while (idx[side]+1<n-1 && length[side]>balance_mpc_grid[idx[side]+1]) idx[side]++;
        fraction[side]=(length[side]-balance_mpc_grid[idx[side]])/(balance_mpc_grid[idx[side]+1]-balance_mpc_grid[idx[side]]);
    }
    float x=fraction[0],y=fraction[1];
    float w[4]={(1-x)*(1-y),x*(1-y),(1-x)*y,x*y};
    const float *v[4]={balance_mpc_grid_data+(idx[0]*n+idx[1])*240u,
        balance_mpc_grid_data+((idx[0]+1)*n+idx[1])*240u,
        balance_mpc_grid_data+(idx[0]*n+idx[1]+1)*240u,
        balance_mpc_grid_data+((idx[0]+1)*n+idx[1]+1)*240u};
    for (unsigned i=0;i<240;i++)
    {
        float value=0;
        for (unsigned j=0;j<4;j++) value+=w[j]*v[j][i];
        if (i<100)c->A[i]=value;else if(i<140)c->B[i-100]=value;else c->terminal[i-140]=value;
    }
    /* 原模型的位置/yaw平移不变性，不让权重求和舍入生成伪常量项。 */
    for (unsigned i=0;i<10;i++) {c->A[i*10]=i==0u?1.0f:0.0f;c->A[i*10+2]=i==2u?1.0f:0.0f;}
    c->model=(mpc_model_t){10u,4u,BALANCE_MPC_N,c->A,c->B,balance_mpc_Q,c->P,c->K,c->Hinv,4.0f,c->Ct,1.7f};
    mpc_status_t status=MPC_Factor_Build(&c->model,balance_mpc_R,c->terminal,c->P,c->K,c->Hinv,c->scratch,220);
    c->model_valid=(uint8_t)(status==MPC_OK);
    if(c->model_valid){MPC_Dynamics_Cache(&c->model,c->Ct);memcpy(c->length,length,sizeof(c->length));c->model_version++;}
    return status;
}
mpc_status_t Balance_MPC_Bounds(const leg_state_t *left,const leg_state_t *right,
    const float force[2],const leg_compensation_values_t *values,
    const lqr_runtime_config_t *config,float lower[4],float upper[4])
{
    const leg_state_t *leg[2]={left,right};
    uint8_t spring=(uint8_t)(GAS_SPRING_COMP_ENABLE && machine->compensation.spring!=NULL);
    if((spring&&!values->spring.valid)||(machine->compensation.gravity!=NULL&&!values->gravity.valid))return MPC_INVALID;
    lower[0]=lower[1]=-config->trq_max_wheel;upper[0]=upper[1]=config->trq_max_wheel;
    for(unsigned side=0;side<2;side++)
    {
        if(!leg[side]->output.force_valid || !isfinite(force[side]))return MPC_INVALID;
        lower[side+2]=-config->trq_max_hip;upper[side+2]=config->trq_max_hip;
        for(unsigned j=0;j<2;j++)
        {
            unsigned index=side*2+j;
            float a=-leg[side]->output.force_map[j][1]; /* 模型u_hip=-Tp。 */
            float constant=leg[side]->output.force_map[j][0]*force[side];
            if(spring)constant+=values->spring.delta_dm[index];
            if(machine->compensation.gravity!=NULL)constant+=values->gravity.delta_dm[index];
            if(!isfinite(a)||!isfinite(constant))return MPC_INVALID;
            if(a==0.0f){if(fabsf(constant)>config->trq_max_hip)return MPC_INFEASIBLE;}
            else
            {
                float v0=(-config->trq_max_hip-constant)/a,v1=(config->trq_max_hip-constant)/a;
                if(!isfinite(v0)||!isfinite(v1))return MPC_INVALID;
                lower[side+2]=fmaxf(lower[side+2],fminf(v0,v1));upper[side+2]=fminf(upper[side+2],fmaxf(v0,v1));
            }
        }
        if(lower[side+2]>upper[side+2])return MPC_INFEASIBLE;
    }
    return MPC_OK;
}
void Balance_MPC_Solve(balance_mpc_context_t *c,const balance_mpc_input_t *in,balance_mpc_solution_t *out)
{
    uint32_t begin=(uint32_t)(Mono_Ns_Get()/1000u);
    memset(out,0,sizeof(*out));out->epoch=in->epoch;out->sample=in->sample;
    out->time_us=in->time_us+(uint32_t)(BALANCE_MPC_TS*1000000.0f+0.5f)*BALANCE_MPC_SOLVE_DIV;out->status=MPC_INVALID;
    mpc_workspace_t work={c->storage,BALANCE_MPC_WORK_SIZE};
    if(c->epoch!=in->epoch){MPC_Reset(&work);c->epoch=in->epoch;}
    if(!c->model_valid || fabsf(c->length[0]-in->length[0])>BALANCE_MPC_MODEL_DELTA || fabsf(c->length[1]-in->length[1])>BALANCE_MPC_MODEL_DELTA)
    {
        if(Balance_MPC_Model(c,in->length)!=MPC_OK)return;
        MPC_Reset(&work);
        out->model_us=(uint32_t)(Mono_Ns_Get()/1000u)-begin;
    }
    /* 上次计划是按旧快照排的；新快照已前进SOLVE_DIV+1个节点，先平移z/y再求解。
     * 只影响ADMM初值，收敛验收和最优解不变。 */
    (void)MPC_Shift_Warm_Start(&c->model,&work,BALANCE_MPC_WARM_SHIFT);
    /* 按250Hz求解间隔预测两个节点到计划激活时刻；仍用最近已选输入作ZOH。
     * 这是LPV/ZOH近似，不宣称精确补偿传感器/通信延迟。 */
    /* 与原LQR相同的工作点假设：delta_x=x-x_eq，delta_u=u-u_eq。
     * 操纵参考单独减工作点，绝不把0.7m位置目标当静态腿姿态修正。 */
    float predicted[10];
    for(unsigned i=0u;i<10u;i++)
    {
        if(!isfinite(in->x_eq[i])){return;}
        predicted[i]=in->state[i]-in->x_eq[i];
    }
    for(unsigned step=0;step<BALANCE_MPC_SOLVE_DIV;step++)
    {
        float next[10];
        for(unsigned i=0;i<10;i++)
        {
            next[i]=0;
            for(unsigned j=0;j<10;j++)next[i]+=c->A[i*10+j]*predicted[j];
            for(unsigned j=0;j<4;j++)next[i]+=c->B[i*4+j]*(in->applied[j]-in->u_eq[j])/balance_mpc_input_scale[j];
        }
        memcpy(predicted,next,sizeof(predicted));
    }
    for(unsigned k=0;k<=BALANCE_MPC_N;k++)
    {
        float reference[10];
        for(unsigned i=0u;i<10u;i++){reference[i]=in->reference[i]-in->x_eq[i];}
        reference[0]+=in->slope[0]*(float)(k+BALANCE_MPC_SOLVE_DIV)*BALANCE_MPC_TS;
        reference[2]+=in->slope[1]*(float)(k+BALANCE_MPC_SOLVE_DIV)*BALANCE_MPC_TS;
        const float *weight=k==BALANCE_MPC_N ? c->terminal : balance_mpc_Q;
        for(unsigned i=0;i<10;i++)
        {
            float value=0;for(unsigned j=0;j<10;j++)value-=weight[i*10+j]*reference[j];c->q[k*10+i]=value;
            if(k<BALANCE_MPC_N)
            {
                /* 偏差输入已扣u_eq，此处不能再次加入B*u_eq。 */
                c->affine[k*10+i]=0.0f;
            }
        }
        if(k<BALANCE_MPC_N)for(unsigned j=0;j<4;j++)
        {
            c->r[k*4+j]=0;
            c->lower[k*4+j]=(in->lower[j]-in->u_eq[j])/balance_mpc_input_scale[j];
            c->upper[k*4+j]=(in->upper[j]-in->u_eq[j])/balance_mpc_input_scale[j];
        }
    }
    /* 实测迭代p95=22、最大26，远低于100轮；上限降到40给4ms预算留余量，减少计划过期。 */
    mpc_problem_t problem={&c->model,predicted,c->q,c->r,c->affine,c->lower,c->upper,40u,0.001f,0.001f,1u,NULL,NULL};
    /* 唯一路径：带输入投影的ADMM，所有工况使用同一收敛验收。 */
    mpc_result_t result;
    out->status=MPC_Solve(&problem,&work,&result);
    out->iterations=result.iterations;out->primal=result.primal;out->dual=result.dual;
    out->model_version=c->model_version;memcpy(out->length,c->length,sizeof(out->length));
    if(out->status==MPC_OK)
    {
        for(unsigned k=0;k<BALANCE_MPC_N;k++)for(unsigned j=0;j<4;j++)
        {
            unsigned index=k*4u+j;
            out->input[index]=result.input[index]*balance_mpc_input_scale[j]+in->u_eq[j];
            if(fabsf((result.input[index]-c->lower[index])*balance_mpc_input_scale[j])<=.001f || fabsf((result.input[index]-c->upper[index])*balance_mpc_input_scale[j])<=.001f){out->constraint_active=1u;}
        }
        out->valid=1u;
    }
    else if(out->status==MPC_INVALID || out->status==MPC_INFEASIBLE)MPC_Reset(&work);
    /* 有限未收敛辅助量只作下次初值，绝不发布为可用动作。 */
    out->solve_us=(uint32_t)(Mono_Ns_Get()/1000u)-begin;
}
