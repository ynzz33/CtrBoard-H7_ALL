/* Nominal model + known trim cancellation + constant residual load; no bench/calibration claim. */
#include "roll_height_balance.h"
#include "machine_config.h"
#ifdef NDEBUG
#error "Bias regression requires active assertions"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static machine_cfg_t cfg;
const machine_cfg_t *const machine=&cfg;
uint64_t Mono_Ns_Get(void){return 0u;}
int main(void)
{
    cfg.lqr.roll_height.support_force[0]=cfg.lqr.roll_height.support_force[1]=25.544f*9.81f*.5f;
    cfg.lqr.roll_height.leg_force_min=-10;cfg.lqr.roll_height.roll_height_bias_common_max=150;cfg.lqr.roll_height.roll_height_bias_diff_max=40;
    cfg.lqr.roll_height.mpc.max_iter=100;cfg.lqr.roll_height.mpc.primal_tolerance=.002f;cfg.lqr.roll_height.mpc.dual_tolerance=.0000002f;cfg.lqr.roll_height.mpc.solve_div=2;
    cfg.lqr.filter.lpf_alpha[2]=cfg.lqr.filter.lpf_alpha[3]=1;
    lqr_runtime_config_t ctl={0};ctl.trq_max_hip=54;ctl.leg_swing_enable=1;
    for(unsigned test=0;test<2u;test++)
    {
        static roll_height_balance_t state;Roll_Height_Reset(&state);
        leg_state_t leg[2]={0};lqr_state_t lqr={0};leg_compensation_values_t values={0};
        float x[4]={0,0,.22f,0},target[2]={.22f,.22f},rate[2]={0,0},disturbance=test?8.0f:-8.0f;
        for(unsigned j=0;j<2u;j++)
        {
            leg[j].output.valid=leg[j].output.force_valid=1;
            leg[j].output.force_map[0][0]=.1f;leg[j].output.force_map[1][0]=-.1f;
            leg[j].output.force_map[0][1]=leg[j].output.force_map[1][1]=1;
        }
        for(unsigned k=0;k<=RH_MPC_N;k++){state.mpc.reference[k*4+2]=.22f;}
        for(unsigned tick=0;tick<8000u;tick++)
        {
            for(unsigned j=0;j<2u;j++){leg[j].output.virtual_leg_length=x[2];leg[j].output.d_virtual_leg_length=x[3];}
            lqr.roll=x[0];lqr.roll_rate=x[1];
            Roll_Height_Begin_Tick(&state,1,tick!=0,tick!=0);
            assert(Roll_Height_Update(&state,&lqr,&leg[0],&leg[1],target,rate,0,&values,&ctl));
            assert(state.mpc.backend==1);
            float next[4];
            for(unsigned i=0;i<4u;i++)
            {
                next[i]=roll_height_model.f[i];
                for(unsigned j=0;j<4u;j++){next[i]+=roll_height_model.A[i][j]*x[j];}
                for(unsigned j=0;j<2u;j++){next[i]+=roll_height_model.B[i][j]*(state.force[j]-state.lqr.height_trim+disturbance);}
            }
            memcpy(x,next,sizeof(x));
        }
#if MACHINE_ROLL_HEIGHT_OBSERVER_ENABLE
        assert(fabsf(x[2]-.22f)<.0001f && fabsf(state.observer.bias[0]-disturbance)<.02f);
#else
        assert(fabsf(x[2]-.22f)>.007f && fabsf(x[2]-.22f)<.009f && state.observer.bias[0]==0);
#endif
        float saved=state.observer.bias[0];
        Roll_Height_Begin_Tick(&state,1,0,1);
        assert(Roll_Height_Update(&state,&lqr,&leg[0],&leg[1],target,rate,0,&values,&ctl));
        assert(state.observer.bias[0]==saved);
        printf("bias %+g N, final error %.9g m, estimate %.9g N: PASS\n",disturbance,x[2]-.22f,state.observer.bias[0]);
        Roll_Height_Begin_Tick(&state,0,0,0);assert(state.observer.bias[0]==0 && !state.observer.seeded);
    }
    return 0;
}
