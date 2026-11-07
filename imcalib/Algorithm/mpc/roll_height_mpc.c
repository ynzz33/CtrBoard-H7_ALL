/* 单写者四态适配：原补偿作为已知输入校正，首版冻结当拍F区间。 */
#include "roll_height_mpc.h"
#include "roll_height_balance.h"
#include "machine_config.h"
#include "mono_ns.h"
#include <math.h>
#include <string.h>
#include <float.h>
#if defined(__CC_ARM)
#pragma O2
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif

mpc_status_t Roll_Height_MPC_Bounds(const leg_state_t *left, const leg_state_t *right,
    const float Tp[2], const leg_compensation_values_t *values,
    const lqr_runtime_config_t *config, float lower[2], float upper[2])
{
    const leg_state_t *leg[2]={left,right};
    uint8_t spring=(uint8_t)(GAS_SPRING_COMP_ENABLE && machine->compensation.spring!=NULL);
    if ((spring && !values->spring.valid) || (machine->compensation.gravity!=NULL && !values->gravity.valid)) return MPC_INVALID;
    for (uint8_t side=0;side<2;side++)
    {
        if (!leg[side]->output.force_valid) return MPC_INVALID;
        lower[side]=machine->lqr.roll_height.leg_force_min;
        upper[side]=FLT_MAX;
        for (uint8_t j=0;j<2;j++)
        {
            float a=leg[side]->output.force_map[j][0];
            float c=leg[side]->output.force_map[j][1]*Tp[side];
            uint8_t index=side*2u+j;
            if (spring) c+=values->spring.delta_dm[index];
            if (machine->compensation.gravity!=NULL) c+=values->gravity.delta_dm[index];
            if (!isfinite(a) || !isfinite(c)) return MPC_INVALID;
            if (a==0.0f)
            {
                if (fabsf(c)>config->trq_max_hip) return MPC_INFEASIBLE;
            }
            else
            {
                float lo=(-config->trq_max_hip-c)/a;
                float hi=(config->trq_max_hip-c)/a;
                if (!isfinite(lo) || !isfinite(hi)) return MPC_INVALID;
                lower[side]=fmaxf(lower[side],fminf(lo,hi));
                upper[side]=fminf(upper[side],fmaxf(lo,hi));
            }
        }
        if (lower[side]>upper[side]) return MPC_INFEASIBLE;
    }
    return MPC_OK;
}

/* 原LQR保持1ms模型；以离散系统组合验证MPC多拍ZOH模型，重力仿射亦同时组合。 */
static uint8_t Roll_Height_MPC_Model(roll_height_mpc_t *ctx, uint16_t steps)
{
    const mpc_model_t *m=&roll_height_mpc_model;
    float A[16]={0},B[8]={0},f[4]={0},nextA[16],nextB[8],nextf[4],determinant;
    for (uint8_t i=0;i<4;i++) A[i*4u+i]=1.0f;
    for (uint16_t step=0;step<steps;step++)
    {
        for (uint8_t i=0;i<4;i++)
        {
            nextf[i]=roll_height_model.f[i];
            for (uint8_t k=0;k<4;k++) nextf[i]+=roll_height_model.A[i][k]*f[k];
            for (uint8_t j=0;j<4;j++)
            {
                nextA[i*4u+j]=0;
                for (uint8_t k=0;k<4;k++) nextA[i*4u+j]+=roll_height_model.A[i][k]*A[k*4u+j];
            }
            for (uint8_t j=0;j<2;j++)
            {
                nextB[i*2u+j]=roll_height_model.B[i][j];
                for (uint8_t k=0;k<4;k++) nextB[i*2u+j]+=roll_height_model.A[i][k]*B[k*2u+j];
            }
        }
        memcpy(A,nextA,sizeof(A));memcpy(B,nextB,sizeof(B));memcpy(f,nextf,sizeof(f));
    }
    for (uint8_t i=0;i<16;i++) if (fabsf(m->A[i]-A[i])>1e-6f) return 0u;
    for (uint8_t i=0;i<8;i++) if (fabsf(m->B[i]-B[i])>1e-6f) return 0u;
    memcpy(ctx->model_affine,f,sizeof(f));
    /* 由roll/高度速度方程解参考力，不对传感器做差分。 */
    determinant=m->B[2u]*m->B[7u]-m->B[3u]*m->B[6u];
    if(!isfinite(determinant) || determinant==0.0f){return 0u;}
    ctx->reference_gain[0]=m->B[7u]/determinant;ctx->reference_gain[1]=-m->B[3u]/determinant;
    ctx->reference_gain[2]=-m->B[6u]/determinant;ctx->reference_gain[3]=m->B[2u]/determinant;
    return 1u;
}

/* 输入是相对同一support工作点的delta-F；物理上下界仍由优化器执行。 */
uint8_t Roll_Height_MPC_Input_Reference(const roll_height_mpc_t *ctx,const float current[4],const float next[4],float input[2])
{
    const mpc_model_t *m=&roll_height_mpc_model;
    float residual[2];
    for(uint8_t side=0u;side<2u;side++)
    {
        uint8_t row=(uint8_t)(2u*side+1u);
        residual[side]=next[row]-ctx->model_affine[row];
        for(uint8_t j=0u;j<4u;j++){residual[side]-=m->A[row*4u+j]*current[j];}
        for(uint8_t j=0u;j<2u;j++){residual[side]-=m->B[row*2u+j]*machine->lqr.roll_height.support_force[j];}
    }
    for(uint8_t j=0u;j<2u;j++)
    {
        input[j]=ctx->reference_gain[j*2u]*residual[0]+ctx->reference_gain[j*2u+1u]*residual[1];
        if(!isfinite(input[j])){return 0u;}
    }
    return 1u;
}

/* 首拍与保持拍都按本拍Tp、几何及补偿复核，不使用旧区间代替当前区间。 */
static uint8_t Roll_Height_MPC_Torque(const float force[2], const float Tp[2],
    const leg_state_t *left, const leg_state_t *right,
    const leg_compensation_values_t *values, const lqr_runtime_config_t *config)
{
    for (uint8_t side=0;side<2;side++)
    {
        const leg_state_t *leg=side==0u ? left : right;
        if (!isfinite(force[side])) return 0u;
        for (uint8_t j=0;j<2;j++)
        {
            uint8_t index=side*2u+j;
            float tau=leg->output.force_map[j][0]*force[side]+leg->output.force_map[j][1]*Tp[side];
            if (GAS_SPRING_COMP_ENABLE && machine->compensation.spring!=NULL) tau+=values->spring.delta_dm[index];
            if (machine->compensation.gravity!=NULL) tau+=values->gravity.delta_dm[index];
            if (!isfinite(tau) || fabsf(tau)>config->trq_max_hip+0.0001f) return 0u;
        }
    }
    return 1u;
}

void Roll_Height_MPC_Update(struct roll_height_balance *state, const lqr_state_t *lqr,
    const leg_state_t *left, const leg_state_t *right,
    const leg_compensation_values_t *values, const lqr_runtime_config_t *config)
{
    roll_height_mpc_t *ctx=&state->mpc;
    const mpc_model_t *m=&roll_height_mpc_model;
    mpc_workspace_t work={ctx->storage,RH_MPC_WORK_SIZE};
    float error[4], Tp[2], affine[4];
    uint16_t div=machine->lqr.roll_height.mpc.solve_div;
    uint32_t begin=(uint32_t)(Mono_Ns_Get()/1000u);
    ctx->candidate[0]=0.0f;ctx->candidate[1]=0.0f;
    ctx->backend=0u;
    if (MACHINE_DEFAULT!=MACHINE_ID_BIG_WHEELLEG || m->nx!=4u || m->nu!=2u || m->horizon!=RH_MPC_N ||
        div==0u || fabsf(RH_MPC_TS-MACHINE_LQR_DT*(float)div)>1e-7f || fabsf(roll_height_model.ts-MACHINE_LQR_DT)>1e-7f)
    {
        ctx->backend=4u;ctx->result.status=MPC_INVALID;ctx->fallback_count++;ctx->hold_valid=0u;ctx->solve_countdown=0u;
        return;
    }
    if (!ctx->initialized)
    {
        ctx->hold_valid=0u;
        /* 上拍未提交时不复用保持值，但不把降频计数清零而反复触发冷求解。 */
        if (ctx->solve_countdown>0u) {ctx->result.status=MPC_INVALID;}
        if (!Roll_Height_MPC_Model(ctx,div))
        {ctx->backend=4u;ctx->result.status=MPC_INVALID;ctx->fallback_count++;return;}
        MPC_Reset(&work);ctx->initialized=1u;
    }
    Tp[0]=config->leg_swing_enable ? -lqr->u[LQR_U_BL] : 0.0f;
    Tp[1]=config->leg_swing_enable ? -lqr->u[LQR_U_BR] : 0.0f;
    mpc_status_t bounds_status=MPC_OK;
    if (MACHINE_MPC_INPUT_CONSTRAINT) bounds_status=Roll_Height_MPC_Bounds(left,right,Tp,values,config,ctx->lower,ctx->upper);
    if (bounds_status!=MPC_OK)
    {
        ctx->backend=bounds_status==MPC_INFEASIBLE ? 3u : 5u; ctx->result.status=bounds_status;ctx->fallback_count++;
        ctx->hold_valid=0u;ctx->solve_countdown=0u;
        MPC_Reset(&work);
        ctx->elapsed_us=(uint32_t)(Mono_Ns_Get()/1000u)-begin;
        return;
    }
    if (ctx->solve_countdown>0u)
    {
        ctx->solve_countdown--;
        if (ctx->hold_valid)
        {
            uint8_t valid=1u;
            for (uint8_t i=0;i<2;i++) if (MACHINE_MPC_INPUT_CONSTRAINT &&
                (ctx->hold_force[i]<ctx->lower[i] || ctx->hold_force[i]>ctx->upper[i])) valid=0u;
            if (MACHINE_MPC_INPUT_CONSTRAINT && !Roll_Height_MPC_Torque(ctx->hold_force,Tp,left,right,values,config)) valid=0u;
            if (valid)
            {
                memcpy(state->force,ctx->hold_force,sizeof(state->force));
                memcpy(ctx->candidate,ctx->hold_force,sizeof(ctx->candidate));
                ctx->backend=1u;
                return;
            }
            ctx->hold_valid=0u;ctx->solve_countdown=0u;
            ctx->result.status=MPC_INVALID;
        }
        ctx->backend=2u;ctx->fallback_count++;
        if (MACHINE_MPC_INPUT_CONSTRAINT) for (uint8_t i=0;i<2;i++) state->force[i]=clampf(state->force[i],ctx->lower[i],ctx->upper[i]);
        return;
    }
    ctx->solve_count++;ctx->solve_countdown=div-1u;
    memset(&ctx->result,0,sizeof(ctx->result));
    for (uint8_t i=0;i<4;i++) error[i]=state->x[i]-state->target[i];
    for (uint8_t i=0;i<4;i++)
    {
        affine[i]=ctx->model_affine[i]-state->target[i];
        for (uint8_t j=0;j<4;j++) affine[i]+=m->A[i*4u+j]*state->target[j];
        for (uint8_t j=0;j<2;j++) affine[i]+=m->B[i*2u+j]*machine->lqr.roll_height.support_force[j];
    }
    for (uint16_t k=0;k<=RH_MPC_N;k++)
    {
        const float *weight=k==RH_MPC_N ? m->P+RH_MPC_N*16u : m->Q;
        for (uint8_t i=0;i<4;i++)
        {
            ctx->q[k*4u+i]=0.0f;
            if (MACHINE_MPC_REFERENCE_PREDICT)
                for (uint8_t j=0;j<4;j++) ctx->q[k*4u+i]-=weight[i*4u+j]*(ctx->reference[k*4u+j]-state->target[j]);
            if (k<RH_MPC_N)
            {
                /* 误差模型保留Ad*r-r+Bd*support+fd；b已作为输入校正消去。 */
                ctx->affine[k*4u+i]=affine[i];
            }
        }
        if(k<RH_MPC_N)
        {
            float input_reference[2]={0.0f,0.0f};
            if(MACHINE_MPC_REFERENCE_PREDICT && !Roll_Height_MPC_Input_Reference(ctx,ctx->reference+k*4u,ctx->reference+(k+1u)*4u,input_reference))
            {
                ctx->backend=5u;ctx->result.status=MPC_INVALID;ctx->fallback_count++;ctx->hold_valid=0u;
                return;
            }
            for(uint8_t j=0u;j<2u;j++)
            {
                ctx->r[k*2u+j]=0.0f;
                for(uint8_t i=0u;i<2u;i++){ctx->r[k*2u+j]-=roll_height_mpc_model_input_weight[j*2u+i]*input_reference[i];}
                ctx->lo[k*2u+j]=ctx->lower[j]-state->lqr.base_force[j];
                ctx->hi[k*2u+j]=ctx->upper[j]-state->lqr.base_force[j];
            }
        }
    }
    const mpc_problem_t problem={m,error,ctx->q,ctx->r,ctx->affine,ctx->lo,ctx->hi,
        machine->lqr.roll_height.mpc.max_iter,machine->lqr.roll_height.mpc.primal_tolerance,
        machine->lqr.roll_height.mpc.dual_tolerance,MACHINE_MPC_INPUT_CONSTRAINT,
        NULL,NULL};
    if (MPC_Solve(&problem,&work,&ctx->result)==MPC_OK)
    {
        uint8_t torque_valid=1u;
        for (uint8_t i=0;i<2;i++) ctx->candidate[i]=state->lqr.base_force[i]+ctx->result.input[i];
        /* 按原映射/补偿的实际浮点顺序复核首拍，而非只检验区间坐标。 */
        if (MACHINE_MPC_INPUT_CONSTRAINT) torque_valid=Roll_Height_MPC_Torque(ctx->candidate,Tp,left,right,values,config);
        if (isfinite(ctx->candidate[0]) && isfinite(ctx->candidate[1]) && torque_valid)
        {
            memcpy(state->force,ctx->candidate,sizeof(state->force));
            if (!MACHINE_MPC_INPUT_CONSTRAINT)
            {
                float shift=fmaxf(0.0f,machine->lqr.roll_height.leg_force_min-fminf(state->force[0],state->force[1]));
                state->force[0]+=shift;state->force[1]+=shift;
            }
            ctx->backend=1u;
            ctx->hold_valid=1u;
            memcpy(ctx->hold_force,state->force,sizeof(ctx->hold_force));
        }
    }
    if (ctx->backend!=1u && ctx->result.status==MPC_OK) ctx->result.status=MPC_INVALID;
    if (ctx->backend!=1u)
    {
        ctx->backend=2u;ctx->fallback_count++;
        ctx->hold_valid=0u;
        if (MACHINE_MPC_INPUT_CONSTRAINT) for (uint8_t i=0;i<2;i++) state->force[i]=clampf(state->force[i],ctx->lower[i],ctx->upper[i]);
        MPC_Reset(&work);
    }
    ctx->elapsed_us=(uint32_t)(Mono_Ns_Get()/1000u)-begin;
}
