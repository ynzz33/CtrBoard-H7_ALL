/* 有限Riccati因子：只增广输入R+rho*I，终端物理P保持调用方定义。
 * Cholesky求逆用于小尺寸正定输入块；全程静态调用方存储，无VLA或硬件依赖。 */
#include "mpc_factor.h"
#include <math.h>
#include <string.h>
#if defined(__CC_ARM)
#pragma O2
#elif defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif
size_t MPC_Factor_Scratch_Size(const mpc_model_t *m)
{
    return m->nx*m->nx+2u*m->nx*m->nu+2u*m->nu*m->nu+2u*m->nu;
}
mpc_status_t MPC_Factor_Build(mpc_model_t *m, const float *R, const float *terminal,
    float *P, float *K, float *Hinv, float *scratch, size_t capacity)
{
    size_t n=m->nx,a=m->nu,N=m->horizon;
    if (!n || !a || !N || capacity<MPC_Factor_Scratch_Size(m)) return MPC_INVALID;
    float *PA=scratch,*PB=PA+n*n,*G=PB+n*a,*H=G+a*n,*L=H+a*a,*y=L+a*a,*x=y+a;
    memcpy(P+N*n*n,terminal,n*n*sizeof(float));
    for (size_t kk=N;kk>0;kk--)
    {
        size_t k=kk-1u;
        const float *next=P+(k+1u)*n*n;
        for (size_t i=0;i<n;i++)
        {
            for (size_t j=0;j<n;j++)
            {
                float sum=0;
                for (size_t t=0;t<n;t++) sum+=next[i*n+t]*m->A[t*n+j];
                PA[i*n+j]=sum;
            }
            for (size_t j=0;j<a;j++)
            {
                float sum=0;
                for (size_t t=0;t<n;t++) sum+=next[i*n+t]*m->B[t*a+j];
                PB[i*a+j]=sum;
            }
        }
        for (size_t i=0;i<a;i++)
        {
            for (size_t j=0;j<a;j++)
            {
                float sum=R[i*a+j]+(i==j ? m->rho : 0.0f);
                for (size_t t=0;t<n;t++) sum+=m->B[t*a+i]*PB[t*a+j];
                H[i*a+j]=sum;
            }
            for (size_t j=0;j<n;j++)
            {
                float sum=0;
                for (size_t t=0;t<n;t++) sum+=m->B[t*a+i]*PA[t*n+j];
                G[i*n+j]=sum;
            }
        }
        memset(L,0,a*a*sizeof(float));
        for (size_t i=0;i<a;i++) for (size_t j=0;j<=i;j++)
        {
            float sum=H[i*a+j];
            for (size_t t=0;t<j;t++) sum-=L[i*a+t]*L[j*a+t];
            if (i==j)
            {
                if (!(sum>0.0f) || !isfinite(sum)) return MPC_INVALID;
                L[i*a+j]=sqrtf(sum);
            }
            else L[i*a+j]=sum/L[j*a+j];
        }
        for (size_t col=0;col<a;col++)
        {
            for (size_t i=0;i<a;i++)
            {
                float sum=i==col ? 1.0f : 0.0f;
                for (size_t t=0;t<i;t++) sum-=L[i*a+t]*y[t];
                y[i]=sum/L[i*a+i];
            }
            for (size_t ii=a;ii>0;ii--)
            {
                size_t i=ii-1u;float sum=y[i];
                for (size_t t=i+1u;t<a;t++) sum-=L[t*a+i]*x[t];
                x[i]=sum/L[i*a+i];
                Hinv[k*a*a+i*a+col]=x[i];
            }
        }
        for (size_t i=0;i<a;i++) for (size_t j=0;j<n;j++)
        {
            float sum=0;
            for (size_t t=0;t<a;t++) sum+=Hinv[k*a*a+i*a+t]*G[t*n+j];
            K[k*a*n+i*n+j]=sum;
        }
        for (size_t i=0;i<n;i++) for (size_t j=0;j<n;j++)
        {
            float sum=m->Q[i*n+j];
            for (size_t t=0;t<n;t++) sum+=m->A[t*n+i]*PA[t*n+j];
            for (size_t t=0;t<a;t++) sum-=G[t*n+i]*K[k*a*n+t*n+j];
            if (!isfinite(sum)) return MPC_INVALID;
            P[k*n*n+i*n+j]=sum;
        }
        for (size_t i=0;i<n;i++) for (size_t j=0;j<i;j++)
        {
            float mean=0.5f*(P[k*n*n+i*n+j]+P[k*n*n+j*n+i]);
            P[k*n*n+i*n+j]=mean;P[k*n*n+j*n+i]=mean;
        }
    }
    m->P=P;m->K=K;m->Hinv=Hinv;
    return MPC_OK;
}
