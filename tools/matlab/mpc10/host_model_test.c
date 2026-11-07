/* 主机数学对拍：真实插值/因子/求解源；不模拟设备或声称板端耗时。 */
#include "balance_mpc.h"
#include <stdio.h>
#include <stdlib.h>
const machine_cfg_t *const machine=NULL; /* 数学求解不访问机型全局。 */
uint64_t Mono_Ns_Get(void){return 0;}
int main(int argc,char **argv)
{
    if(argc!=3 && argc!=5)return 2;
    FILE *f=fopen(argv[1],"rb");if(!f)return 3;
    float data[50]={0};size_t count=fread(data,4,50,f);if(count!=40 && count!=50)return 4;fclose(f);
    balance_mpc_input_t in={0};unsigned p=0;
    for(unsigned i=0;i<2;i++)in.length[i]=data[p++];
    for(unsigned i=0;i<10;i++)in.state[i]=data[p++];
    for(unsigned i=0;i<10;i++)in.reference[i]=data[p++];
    for(unsigned i=0;i<2;i++)in.slope[i]=data[p++];
    for(unsigned i=0;i<4;i++)in.lower[i]=data[p++];
    for(unsigned i=0;i<4;i++)in.upper[i]=data[p++];
    for(unsigned i=0;i<4;i++)in.u_eq[i]=data[p++];
    for(unsigned i=0;i<4;i++)in.applied[i]=data[p++];
    for(unsigned i=0;i<10;i++)in.x_eq[i]=data[p++];
    in.epoch=1;in.sample=1;
    static balance_mpc_context_t ctx;
    balance_mpc_solution_t out;
    /* 可选参数只用于离线扫描，生产适配层没有此覆盖路径。 */
    if(argc==5)
    {
        if(Balance_MPC_Model(&ctx,in.length)!=MPC_OK)return 6;
        ctx.model.rho=(float)atof(argv[3]);ctx.model.relaxation=(float)atof(argv[4]);
        if(MPC_Factor_Build(&ctx.model,balance_mpc_R,ctx.terminal,ctx.P,ctx.K,ctx.Hinv,ctx.scratch,220)!=MPC_OK)return 7;
        MPC_Dynamics_Cache(&ctx.model,ctx.Ct);
    }
    Balance_MPC_Solve(&ctx,&in,&out);
    f=fopen(argv[2],"wb");if(!f)return 5;
    float status=(float)out.status,iterations=(float)out.iterations;
    fwrite(&status,4,1,f);fwrite(&iterations,4,1,f);fwrite(out.input,4,BALANCE_MPC_N*4,f);
    fwrite(ctx.A,4,100,f);fwrite(ctx.B,4,40,f);fwrite(ctx.terminal,4,100,f);
    fwrite(ctx.K,4,BALANCE_MPC_N*40,f);fwrite(ctx.Hinv,4,BALANCE_MPC_N*16,f);
    fwrite(&out.primal,4,1,f);fwrite(&out.dual,4,1,f);fwrite(&ctx.model.rho,4,1,f);
    fclose(f);return 0;
}
