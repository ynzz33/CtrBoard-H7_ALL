/* 输入分裂ADMM：有限Riccati线性项反推、动力学前推、盒投影及残差。
 * 缓存由MATLAB产生；不分裂状态，因此不向Q/终端物理P添加rho。 */
#include "mpc_solver.h"
#include <math.h>
#include <float.h>
#include <string.h>
/* 仅优化数值内核，不改变整机编译级别；耗时仍以板端测量为准。 */
#if defined(__CC_ARM)
#pragma O2
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif
size_t MPC_Workspace_Size(const mpc_model_t *m)
{
    return MPC_WORKSPACE_FLOATS(m->nx,m->nu,m->horizon);
}
void MPC_Reset(mpc_workspace_t *w)
{
    memset(w->storage,0,w->capacity*sizeof(float));
}
/* 只移动z/y初值，不改变问题或验收；末尾重复上一节点，避免引入零输入。 */
mpc_status_t MPC_Shift_Warm_Start(const mpc_model_t *m,mpc_workspace_t *w,uint16_t shift)
{
    if(!m || !w || !shift || shift>=m->horizon || w->capacity<MPC_Workspace_Size(m))return MPC_INVALID;
    size_t N=m->horizon,a=m->nu;
    float *z=w->storage+2u*(N+1u)*m->nx+2u*N*a;
    float *y=z+N*a;
    size_t keep=(N-shift)*a;
    memmove(z,z+shift*a,keep*sizeof(float));
    memmove(y,y+shift*a,keep*sizeof(float));
    for(size_t i=keep;i<N*a;i++){z[i]=z[keep-1u];y[i]=y[keep-1u];}
    return MPC_OK;
}

#if defined(__CC_ARM)
#define MPC_INLINE static __forceinline
#elif defined(__GNUC__)
#define MPC_INLINE static inline __attribute__((always_inline))
#else
#define MPC_INLINE static inline
#endif
/* 固定维数内核使count/stride成为常量，展开累加保持顺序。 */
MPC_INLINE float MPC_Mac(float sum,const float *a,const float *b,size_t count,size_t stride)
{
#define MAC(i) sum+=a[(i)*stride]*b[(i)]
    if(count==10u)
    {
        MAC(0);MAC(1);MAC(2);MAC(3);MAC(4);MAC(5);MAC(6);MAC(7);MAC(8);MAC(9);
    }
    else if(count==4u)
    {
        MAC(0);MAC(1);MAC(2);MAC(3);
    }
    else if(count==2u)
    {
        MAC(0);MAC(1);
    }
    else
    {
        for(size_t i=0u;i<count;i++){MAC(i);}
    }
#undef MAC
    return sum;
}
/* IEEE浮点比较：NaN/Inf不满足双界，避免AC5在热循环调用有限性/绝对值库。 */
MPC_INLINE uint8_t MPC_Finite(float value)
{
    return (uint8_t)(value<=FLT_MAX && value>=-FLT_MAX);
}
MPC_INLINE float MPC_Abs(float value)
{
    return value<0.0f ? -value : value;
}
MPC_INLINE float MPC_Clip(float value,float lo,float hi)
{
    if(value<lo){return lo;}
    if(value>hi){return hi;}
    return value;
}
void MPC_Dynamics_Cache(const mpc_model_t *m,float *Ct)
{
    size_t n=m->nx,a=m->nu;
    for(size_t k=0u;k<m->horizon;k++)
    {
        for(size_t i=0u;i<n;i++)
        {
            for(size_t j=0u;j<n;j++)
            {
                float value=m->A[j*n+i];
                for(size_t t=0u;t<a;t++){value-=m->B[j*a+t]*m->K[k*a*n+t*n+i];}
                Ct[k*n*n+i*n+j]=value;
            }
        }
    }
}
#define MPC_KERNEL_NAME MPC_Kernel_10_4
#define MPC_NX 10u
#define MPC_NU 4u
#include "mpc_kernel.inc"
#undef MPC_KERNEL_NAME
#undef MPC_NX
#undef MPC_NU
#define MPC_KERNEL_NAME MPC_Kernel_4_2
#define MPC_NX 4u
#define MPC_NU 2u
#include "mpc_kernel.inc"
#undef MPC_KERNEL_NAME
#undef MPC_NX
#undef MPC_NU
#define MPC_KERNEL_NAME MPC_Kernel_Generic
#define MPC_NX v->model->nx
#define MPC_NU v->model->nu
#include "mpc_kernel.inc"
#undef MPC_KERNEL_NAME
#undef MPC_NX
#undef MPC_NU
mpc_status_t MPC_Solve(const mpc_problem_t *v,mpc_workspace_t *w,mpc_result_t *out)
{
    if(v->model->nx==10u && v->model->nu==4u){return MPC_Kernel_10_4(v,w,out);}
    if(v->model->nx==4u && v->model->nu==2u){return MPC_Kernel_4_2(v,w,out);}
    return MPC_Kernel_Generic(v,w,out);
}
