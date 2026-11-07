/* 同一有限时域ADMM模板，分别编译固定10/4、4/2及通用内核；无解析返回路径。 */
static mpc_status_t MPC_KERNEL_NAME(const mpc_problem_t *v,mpc_workspace_t *w,mpc_result_t *out)
{
    const mpc_model_t *m=v->model;
    const size_t n=MPC_NX,a=MPC_NU,N=m->horizon;
    const float alpha=m->relaxation==0.0f ? 1.0f : m->relaxation;
    memset(out,0,sizeof(*out));out->status=MPC_INVALID;
    if(!(alpha>0.0f && alpha<2.0f) || !n || !a || !N || w->capacity<MPC_Workspace_Size(m) || !v->max_iter){return out->status;}
    float *p=w->storage,*x=p+(N+1u)*n,*d=x+(N+1u)*n;
    float *u=d+N*a,*z=u+N*a,*y=z+N*a,*old=y+N*a;
    float *t=old+N*a,*g=t+n,*ru=g+a,*qb=ru+a,*gb=qb+N*n,*cache=gb+N*a;
    const float *Ct=m->Ct;
    if(Ct==NULL){MPC_Dynamics_Cache(m,cache);Ct=cache;}
    for(size_t i=0u;i<n;i++)
    {
        if(!MPC_Finite(v->state[i])){return out->status;}
    }
    for(size_t j=0u;j<N*a;j++)
    {
        if(v->constraints_enabled && !(v->input_min[j]<=v->input_max[j])){out->status=MPC_INFEASIBLE;return out->status;}
        if(!MPC_Finite(z[j]) || !MPC_Finite(y[j])){return out->status;}
        if(v->constraints_enabled){z[j]=MPC_Clip(z[j],v->input_min[j],v->input_max[j]);}
    }
    /* 每个新问题一次：Pf、q+(A-BK)'Pf、r+B'Pf。迭代不再重复矩阵×仿射项。 */
    for(size_t k=0u;k<N;k++)
    {
        for(size_t i=0u;i<n;i++){t[i]=MPC_Mac(0.0f,m->P+(k+1u)*n*n+i*n,v->affine+k*n,n,1u);}
        for(size_t i=0u;i<n;i++){qb[k*n+i]=MPC_Mac(v->linear_state[k*n+i],Ct+k*n*n+i*n,t,n,1u);}
        for(size_t i=0u;i<a;i++){gb[k*a+i]=MPC_Mac(v->linear_input[k*a+i],m->B+i,t,n,a);}
    }
    for(uint16_t iteration=0u;iteration<v->max_iter;iteration++)
    {
        if(v->should_stop!=NULL && v->should_stop(v->stop_context)){out->status=MPC_TIME_LIMIT;return out->status;}
        memcpy(p+N*n,v->linear_state+N*n,n*sizeof(float));
        for(size_t kk=N;kk>0u;kk--)
        {
            size_t k=kk-1u;
            const float *next=p+(k+1u)*n,*K=m->K+k*a*n;
            for(size_t i=0u;i<a;i++)
            {
                float penalty=m->rho*(y[k*a+i]-z[k*a+i]);
                ru[i]=v->linear_input[k*a+i]+penalty;
                g[i]=MPC_Mac(gb[k*a+i]+penalty,m->B+i,next,n,a);
            }
            for(size_t i=0u;i<a;i++){d[k*a+i]=MPC_Mac(0.0f,m->Hinv+k*a*a+i*a,g,a,1u);}
            for(size_t i=0u;i<n;i++)
            {
                float value=MPC_Mac(qb[k*n+i],Ct+k*n*n+i*n,next,n,1u);
                p[k*n+i]=value-MPC_Mac(0.0f,K+i,ru,a,n);
            }
        }
        memcpy(x,v->state,n*sizeof(float));
        for(size_t k=0u;k<N;k++)
        {
            for(size_t i=0u;i<a;i++){u[k*a+i]=-MPC_Mac(d[k*a+i],m->K+k*a*n+i*n,x+k*n,n,1u);}
            for(size_t i=0u;i<n;i++)
            {
                float value=MPC_Mac(v->affine[k*n+i],m->A+i*n,x+k*n,n,1u);
                x[(k+1u)*n+i]=MPC_Mac(value,m->B+i*a,u+k*a,a,1u);
            }
        }
        out->primal=0.0f;out->dual=0.0f;
        for(size_t j=0u;j<N*a;j++)
        {
            float relaxed=alpha*u[j]+(1.0f-alpha)*z[j];
            float value=relaxed+y[j],primal,dual;
            if(!MPC_Finite(value)){return out->status;}
            old[j]=z[j];z[j]=v->constraints_enabled ? MPC_Clip(value,v->input_min[j],v->input_max[j]) : value;
            y[j]+=relaxed-z[j];primal=MPC_Abs(u[j]-z[j]);dual=m->rho*MPC_Abs(z[j]-old[j]);
            if(primal>out->primal){out->primal=primal;}
            if(dual>out->dual){out->dual=dual;}
        }
        out->iterations=iteration+1u;
        if(v->should_stop!=NULL && v->should_stop(v->stop_context)){out->status=MPC_TIME_LIMIT;return out->status;}
        if(out->primal<=v->primal_tolerance && out->dual<=v->dual_tolerance)
        {
            out->input=z;out->status=MPC_OK;return out->status;
        }
    }
    out->status=MPC_ITERATION_LIMIT;return out->status;
}
