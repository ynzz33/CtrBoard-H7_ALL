/* Actual four-state adapter + reference generator, ideal model only; no HAL or bench claim. */
#include "leg_balance.h"
#include "machine_config.h"
#ifdef NDEBUG
#error "Preview regression requires active assertions"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
static machine_cfg_t cfg;
const machine_cfg_t *const machine=&cfg;
uint64_t Mono_Ns_Get(void){return 0u;}
int main(void)
{
    cfg.rc.len_rate=1.3f;
    cfg.lqr.roll_height.length_ref_accel_max=1.0f;
    cfg.lqr.roll_height.leg_force_min=-10.0f;
    cfg.lqr.roll_height.support_force[0]=cfg.lqr.roll_height.support_force[1]=25.544f*9.81f*.5f;
    cfg.lqr.roll_height.mpc.max_iter=100;
    cfg.lqr.roll_height.mpc.primal_tolerance=.002f;
    cfg.lqr.roll_height.mpc.dual_tolerance=.0000002f;
    cfg.lqr.roll_height.mpc.solve_div=2;
    cfg.lqr.filter.lpf_alpha[2]=cfg.lqr.filter.lpf_alpha[3]=1.0f;
    cfg.lqr.turn_comp.track=1.0f;
    lqr_runtime_config_t control={0};control.trq_max_hip=54;control.leg_swing_enable=1;
    for(unsigned scenario=0;scenario<3u;scenario++)
    {
        static leg_balance_t lb;Leg_Balance_Init(&lb);
        leg_state_t leg[2]={0};lqr_state_t lqr={0};leg_compensation_values_t values={0};torque_output_t torque;
        for(unsigned j=0;j<2u;j++)
        {
            leg[j].output.valid=leg[j].output.force_valid=1;
            leg[j].output.force_map[0][0]=.1f;leg[j].output.force_map[0][1]=1.0f;
            leg[j].output.force_map[1][0]=-.1f;leg[j].output.force_map[1][1]=1.0f;
        }
        float x[4]={0,0,.22f,0};
        double height_sq=0,roll_sq=0,change_sq=0;float max_height=0,max_force=0,previous[2]={0};unsigned fallback=0,max_iter=0;
        for(unsigned tick=0;tick<3600u;tick++)
        {
            float goal=scenario==1u ? .22f : (tick<200u || tick>=1800u ? .22f : .28f);
            if(scenario==1u && tick<1800u){goal=.28f;}
            if(tick==0u && scenario==1u){x[2]=.28f;}
            if(scenario==2u && tick==900u){x[0]+=.04f;x[1]+=.15f;}
            lqr.roll=x[0];lqr.roll_rate=x[1];
            for(unsigned j=0;j<2u;j++)
            {
                leg[j].output.virtual_leg_length=x[2]/cosf(x[0]);
                leg[j].output.d_virtual_leg_length=(x[3]+sinf(x[0])*x[1]*leg[j].output.virtual_leg_length)/cosf(x[0]);
                lqr.leg_length_target[j]=goal;
            }
            Roll_Height_Begin_Tick(&lb.roll_height,1u,0u,tick!=0u);
            assert(Leg_Balance_Compute(&lb,&lqr,&leg[0],&leg[1],&values,&control,1u,.001f,&torque));
            if(lb.roll_height.mpc.backend!=1u){fallback++;}
            if(lb.roll_height.mpc.result.iterations>max_iter){max_iter=lb.roll_height.mpc.result.iterations;}
            float error=x[2]-lb.roll_height.target[2];height_sq+=error*error;roll_sq+=x[0]*x[0];
            if(fabsf(error)>max_height){max_height=fabsf(error);}
            for(unsigned j=0;j<2u;j++)
            {
                assert(isfinite(lb.F[j]) && lb.F[j]>=-10.0001f && lb.F[j]<=540.0001f);
                if(fabsf(lb.F[j])>max_force){max_force=fabsf(lb.F[j]);}
                if(tick){float delta=lb.F[j]-previous[j];change_sq+=delta*delta;}
                previous[j]=lb.F[j];
            }
            float next[4];
            for(unsigned i=0;i<4u;i++)
            {
                next[i]=roll_height_model.f[i];
                for(unsigned j=0;j<4u;j++){next[i]+=roll_height_model.A[i][j]*x[j];}
                for(unsigned j=0;j<2u;j++){next[i]+=roll_height_model.B[i][j]*lb.F[j];}
                assert(isfinite(next[i]));
            }
            memcpy(x,next,sizeof(x));
        }
        assert(fallback==0u && max_height<.03f && fabsf(x[2]-.22f)<.0001f && fabsf(x[0])<.0001f);
        printf("%u %.9g %.9g %.9g %.9g %.9g %u %u\n",scenario,sqrt(height_sq/3600),max_height,sqrt(roll_sq/3600),max_force,sqrt(change_sq/(2*3599)),fallback,max_iter);
    }
    return 0;
}
