#include "lqr_balance.h"
#ifdef NDEBUG
#error "Host regression requires assertions"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
static machine_cfg_t cfg;
const machine_cfg_t *const machine=&cfg;
static lqr_gain_desc_t gain;
motion_guard_t motion_guard;
void Motion_Guard_Reset(motion_guard_t *s){memset(s,0,sizeof(*s));s->reference_scale=1;}
uint8_t Motion_Guard_Update(motion_guard_t *s,const motion_guard_param_t *p,const motion_guard_input_t *i){(void)p;(void)i;s->reference_scale=1;return 1;}
const lqr_gain_desc_t *LQR_Gain_Info(void){return &gain;}
uint8_t LQR_Gain_Eval(float l,float r,float K[40]){for(unsigned j=0;j<40;j++)K[j]=(float)((int)(j%9)-4)*.31f+l-r;return 1;}
extern lqr_debug_t lqr_debug_Baseline;
void LQR_Control_Update_Baseline(lqr_state_t*,uint8_t);
uint8_t LQR_Target_Update_Baseline(lqr_state_t*,const rc_command_t*);
static unsigned seed=12345;
static float random_f(void){seed=1664525u*seed+1013904223u;return (float)(seed>>8)/16777216.0f*4-2;}
int main(void)
{
 cfg.leg.leg_len_min=.14f;cfg.leg.leg_len_max=.33f;cfg.lqr.position_hold.pos_target=.7f;
 cfg.lqr.reference.leg_trim[0]=.015f;cfg.lqr.reference.leg_trim[1]=-.02f;cfg.lqr.reference.pitch_trim=.025f;
 gain.x_eq[4]=.004f;gain.x_eq[6]=-.003f;gain.x_eq[8]=.002f;
 gain.len_min=.15f;gain.len_max=.31f;gain.u_eq[0]=.13f;gain.u_eq[1]=-.07f;
 for(unsigned k=0;k<5000;k++)
 {
  lqr_state_t a,b;LQR_Init(&a);a.valid=1;a.air_control=(k%7==0);a.air_hip_max=2;
  a.len[0]=.23f;a.len[1]=.25f;
  for(unsigned j=0;j<10;j++){a.x[j]=random_f();a.target[j]=random_f();}
  lqr_debug.trq_max_wheel=2;lqr_debug.trq_max_hip=5;lqr_debug.pos_hold=k&1;lqr_debug.yaw_hold=(k>>1)&1;lqr_debug.yaw_rate_hold=(k>>2)&1;
  b=a;lqr_debug_Baseline=lqr_debug;
  rc_command_t cmd={0};cmd.online=1;cmd.velocity_target=(k%3)?random_f():0;cmd.yaw_rate_target=(k%4)?random_f():0;cmd.leg_length_target=.2f;
  a.pos_armed=b.pos_armed=k&1;
  assert(LQR_Target_Update(&a,&cmd)==LQR_Target_Update_Baseline(&b,&cmd));
  assert(memcmp(a.target,b.target,sizeof(a.target))==0);
  assert(memcmp(a.leg_length_target,b.leg_length_target,sizeof(a.leg_length_target))==0);
  assert(a.operating_point[0]==gain.x_eq[0]);
  if(cmd.velocity_target==0 && a.pos_armed && !a.air_control){assert(a.target[0]==.7f);}
  assert(fabsf(a.target[4]+gain.x_eq[4]-a.operating_point[4]-cfg.lqr.reference.leg_trim[0])<.000001f);
  assert(fabsf(a.target[8]+gain.x_eq[8]-a.operating_point[8]-cfg.lqr.reference.pitch_trim)<.000001f);
  LQR_Control_Update(&a,0);LQR_Control_Update_Baseline(&b,0);
  assert(a.gain_valid==b.gain_valid && memcmp(a.u,b.u,sizeof(a.u))==0);

 }
 puts("5000 LQR cases: target/control outputs bit-identical to pre-working-point baseline; 0.7 position and pose trims preserved: PASS");return 0;
}
