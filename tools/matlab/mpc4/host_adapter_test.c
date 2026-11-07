#include "leg_balance.h"
#include "roll_height_balance.h"
#include "machine_config.h"
#ifdef NDEBUG
#error "Host regression requires active assertions"
#endif
#include <assert.h>
#include <stdio.h>
#include <math.h>
static machine_cfg_t cfg;
const machine_cfg_t *const machine=&cfg;
static uint64_t clock_step_ns,clock_ns;
uint64_t Mono_Ns_Get(void){clock_ns+=clock_step_ns;return clock_ns;}
int main(void){
 leg_state_t l={0},r={0};leg_compensation_values_t v={0};lqr_runtime_config_t c={0};float lo[2],hi[2],tp[2]={2,-3};
 cfg.lqr.roll_height.leg_force_min=-10;c.trq_max_hip=54;
 cfg.lqr.roll_height.mpc.max_iter=100;cfg.lqr.roll_height.mpc.primal_tolerance=.002f;cfg.lqr.roll_height.mpc.dual_tolerance=.0000002f;
 cfg.lqr.roll_height.mpc.solve_div=2;
 l.output.force_valid=r.output.force_valid=1;
 float cases[4][4]={{.1f,1,-.2f,.5f},{-.1f,-1,.3f,-.5f},{0,1,.2f,.5f},{.00001f,1,-.3f,2}};
 for(int k=0;k<4;k++){
  l.output.force_map[0][0]=r.output.force_map[0][0]=cases[k][0];l.output.force_map[0][1]=r.output.force_map[0][1]=cases[k][1];
  l.output.force_map[1][0]=r.output.force_map[1][0]=cases[k][2];l.output.force_map[1][1]=r.output.force_map[1][1]=cases[k][3];
  assert(Roll_Height_MPC_Bounds(&l,&r,tp,&v,&c,lo,hi)==MPC_OK);
  for(int s=0;s<2;s++)for(int edge=0;edge<2;edge++)for(int j=0;j<2;j++)assert(fabsf(cases[k][j*2]*(edge?hi[s]:lo[s])+cases[k][j*2+1]*tp[s])<=54.001f);
 }
 l.output.force_map[0][0]=0;l.output.force_map[0][1]=100;assert(Roll_Height_MPC_Bounds(&l,&r,tp,&v,&c,lo,hi)!=MPC_OK);
 puts("bounds signs/zero/small/empty: PASS");
 l.output.force_map[0][0]=.1f;l.output.force_map[0][1]=1;
 cfg.lqr.roll_height.support_force[0]=cfg.lqr.roll_height.support_force[1]=25.544f*9.81f*.5f;
 c.leg_swing_enable=1;
#if MACHINE_ROLL_HEIGHT_USE_MPC
 static roll_height_balance_t state; lqr_state_t lqr={0};
 state.target[2]=.25f;state.x[2]=.27f;
 for(int i=0;i<2;i++){
  state.lqr.base_force[i]=cfg.lqr.roll_height.support_force[i]-50;
  state.force[i]=state.lqr.base_force[i]-roll_height_model.K[i][2]*.02f;
 }
 for(int k=0;k<=RH_MPC_N;k++)state.mpc.reference[k*4+2]=.25f;
 Roll_Height_MPC_Update(&state,&lqr,&l,&r,&v,&c);
 assert(state.mpc.backend==1);
 for(int i=0;i<2;i++) assert(fabsf(state.force[i]-(state.lqr.base_force[i]-roll_height_mpc_model_lqr_gain[i*4+2]*.02f))<.02f);
 puts("adapter same-Q/R 2ms LQR equivalence: PASS");
 int solves=0,holds=0;
 for(int k=0;k<20;k++){
  state.x[0]=.01f*sinf((float)k);state.x[2]=.25f+.01f*cosf((float)k);
  for(int i=0;i<2;i++){state.force[i]=state.lqr.base_force[i];for(int j=0;j<4;j++)state.force[i]-=roll_height_model.K[i][j]*(state.x[j]-state.target[j]);}
  uint32_t before=state.mpc.solve_count;
  Roll_Height_MPC_Update(&state,&lqr,&l,&r,&v,&c);
  solves+=(int)(state.mpc.solve_count-before);holds+=(state.mpc.solve_count==before && state.mpc.backend==1);
  assert(state.mpc.backend==1 || state.mpc.backend==2);
  for(int i=0;i<2;i++)assert(state.force[i]>=state.mpc.lower[i]-.001f && state.force[i]<=state.mpc.upper[i]+.001f);
 }
 assert(solves==10 && holds==10);
 puts("500Hz solve with 1kHz guarded hold: PASS");
 state.mpc.solve_countdown=1;state.mpc.hold_valid=1;state.mpc.hold_force[0]=1000000;
 uint32_t before_reject=state.mpc.solve_count;
 Roll_Height_MPC_Update(&state,&lqr,&l,&r,&v,&c);
 assert(state.mpc.backend==2 && !state.mpc.hold_valid && state.mpc.solve_count==before_reject);
 puts("changed bounds reject held action: PASS");
#endif
 static machine_spring_cfg_t spring;static machine_gravity_cfg_t gravity;
 cfg.compensation.spring=&spring;cfg.compensation.gravity=&gravity;
 assert(Roll_Height_MPC_Bounds(&l,&r,tp,&v,&c,lo,hi)!=MPC_OK);
 v.spring.valid=v.gravity.valid=1;
 for(int j=0;j<4;j++){v.spring.delta_dm[j]=.5f*(j+1);v.gravity.delta_dm[j]=-.25f*(j+1);}
 assert(Roll_Height_MPC_Bounds(&l,&r,tp,&v,&c,lo,hi)==MPC_OK);
 for(int side=0;side<2;side++)for(int edge=0;edge<2;edge++)for(int j=0;j<2;j++){
  const leg_state_t *leg=side?&r:&l;int index=side*2+j;
  float tau=leg->output.force_map[j][0]*(edge?hi[side]:lo[side])+leg->output.force_map[j][1]*tp[side];
  if(GAS_SPRING_COMP_ENABLE)tau+=v.spring.delta_dm[index];tau+=v.gravity.delta_dm[index];assert(fabsf(tau)<=54.001f);
 }
 puts("compensation validity and torque interval: PASS");
 static float mem[RH_MPC_WORK_SIZE],qs[(RH_MPC_N+1)*4],rs[RH_MPC_N*2],af[RH_MPC_N*4],low[RH_MPC_N*2],high[RH_MPC_N*2];
 float x0[4]={.02f,0,0,0};for(int i=0;i<RH_MPC_N*2;i++){low[i]=-1;high[i]=1;}
 mpc_workspace_t work={mem,RH_MPC_WORK_SIZE};mpc_result_t result;
 mpc_problem_t problem={&roll_height_mpc_model,x0,qs,rs,af,low,high,1,0,0,1};
 assert(MPC_Solve(&problem,&work,&result)==MPC_ITERATION_LIMIT);
 low[0]=2;assert(MPC_Solve(&problem,&work,&result)==MPC_INFEASIBLE);low[0]=-1;
 x0[0]=NAN;assert(MPC_Solve(&problem,&work,&result)==MPC_INVALID);x0[0]=0;
 work.capacity=1;assert(MPC_Solve(&problem,&work,&result)==MPC_INVALID);
 puts("iteration/empty/NaN/workspace failure: PASS");
static leg_balance_t lb; lqr_state_t target={0};
 cfg.rc.len_rate=.5f;cfg.lqr.roll_height.length_ref_accel_max=1;
 for(int i=0;i<2;i++){lb.length_ramp[i].out=.25f;lb.length_ref_rate[i]=.1f;target.leg_length_target[i]=.29f;}
 ramp_t saved[2]={lb.length_ramp[0],lb.length_ramp[1]};
 Leg_Balance_Reference_Preview(&lb,&target);
 for(int i=0;i<2;i++){assert(lb.length_ramp[i].out==saved[i].out && lb.length_ref_rate[i]==.1f);}
 assert(fabsf(lb.roll_height.mpc.reference[2]-.25f)<1e-6f);
 assert(fabsf(lb.roll_height.mpc.reference[3]-.1f)<1e-6f);
 assert(lb.roll_height.mpc.reference[RH_MPC_N*4+2]>.25f);
 puts("actual reference preview and unchanged history: PASS");
 static leg_balance_t active; lqr_state_t live={0};torque_output_t torque={0};
 cfg.lqr.turn_comp.track=1;cfg.lqr.filter.lpf_alpha[2]=cfg.lqr.filter.lpf_alpha[3]=1;
 l.output.virtual_leg_length=r.output.virtual_leg_length=.25f;l.output.valid=r.output.valid=1;
 live.leg_length_target[0]=live.leg_length_target[1]=.25f;
 Leg_Balance_Init(&active);Roll_Height_Begin_Tick(&active.roll_height,1,0,0);
 assert(Leg_Balance_Compute(&active,&live,&l,&r,&v,&c,1,.001f,&torque));
 assert(fabsf(active.F[0]-active.roll_height.lqr.base_force[0])<.02f);
 assert(fabsf(active.F[1]-active.roll_height.lqr.base_force[1])<.02f);
 #if MACHINE_ROLL_HEIGHT_USE_MPC
 assert(active.roll_height.mpc.backend==1);
 #else
 assert(active.roll_height.mpc.backend==0);
 #endif
 puts("real NORMAL leg/height/roll/output path: PASS");
#if MACHINE_ROLL_HEIGHT_USE_MPC
 float ref0[4]={0,0,.25f,.1f},ref1[4]={0,0,.250202f,.102f},input_reference[2];
 assert(Roll_Height_MPC_Input_Reference(&active.roll_height.mpc,ref0,ref1,input_reference));
 for(unsigned row=1u;row<4u;row+=2u)
 {
  float predicted=active.roll_height.mpc.model_affine[row];
  for(unsigned j=0;j<4u;j++){predicted+=roll_height_mpc_model.A[row*4u+j]*ref0[j];}
  for(unsigned j=0;j<2u;j++){predicted+=roll_height_mpc_model.B[row*2u+j]*(cfg.lqr.roll_height.support_force[j]+input_reference[j]);}
  assert(fabsf(predicted-ref1[row])<.000001f);
 }
 ref1[3]=NAN;assert(!Roll_Height_MPC_Input_Reference(&active.roll_height.mpc,ref0,ref1,input_reference));
 puts("planned force matches discrete velocity equations; invalid reference rejected: PASS");
#endif

 #if MACHINE_ROLL_HEIGHT_USE_MPC
 active.roll_height.mpc.initialized=0;active.roll_height.mpc.solve_countdown=0;clock_step_ns=1000000;
 Roll_Height_MPC_Update(&active.roll_height,&live,&l,&r,&v,&c);
 assert(active.roll_height.mpc.backend==1 && active.roll_height.mpc.result.status==MPC_OK);
 assert(active.roll_height.mpc.elapsed_us>=1000);
 puts("single-solve time cutoff removed: PASS");
 #endif
#if MACHINE_ROLL_HEIGHT_USE_MPC
 active.roll_height.mpc.hold_valid=1;active.roll_height.mpc.initialized=1;
 Roll_Height_Begin_Tick(&active.roll_height,1,0,1);
 assert(active.roll_height.mpc.hold_valid && active.roll_height.mpc.initialized);
 Roll_Height_Begin_Tick(&active.roll_height,1,1,0);
 assert(!active.roll_height.mpc.initialized);
 puts("observer and applied-output permits separated: PASS");
 active.roll_height.mpc.solve_countdown=0;int attempts=0;
 for(int i=0;i<5;i++){
  Roll_Height_Begin_Tick(&active.roll_height,1,0,0);
  uint32_t before=active.roll_height.mpc.solve_count;
  Roll_Height_MPC_Update(&active.roll_height,&live,&l,&r,&v,&c);
  attempts+=(int)(active.roll_height.mpc.solve_count-before);
 }
 assert(attempts==3);
 puts("failed output does not defeat solve decimation: PASS");
 #endif
 return 0;
}
