/* 十态物理适配，供后台任务单写者调用；模型输入序与原LQR一致，Tp=-u_hip。 */
#ifndef BALANCE_MPC_H
#define BALANCE_MPC_H
#include "balance_mpc_data.h"
#include "mpc_factor.h"
#include "lqr_balance.h"
#include "leg_compensation.h"
#define BALANCE_MPC_MODEL_DELTA 0.005f /* 调度滞回/结果模型偏差门槛m，不改实测腿长。 */
#define BALANCE_MPC_SOLVE_DIV 2u /* 250Hz推理；模型节点仍为BALANCE_MPC_TS=2ms。 */
#define BALANCE_MPC_WARM_SHIFT 2u /* 上次计划相对新快照前移的节点数，仅热启动。 */
typedef struct {
    float state[10],reference[10],slope[2],length[2],lower[4],upper[4],u_eq[4],applied[4];
    float x_eq[10]; /* 与state/reference同坐标的静态工作点，静态速度分量应为0。 */
    uint64_t time_us;
    uint32_t epoch,sample;
} balance_mpc_input_t;
typedef struct {
    float input[BALANCE_MPC_N*4u],length[2];
    uint64_t time_us;
    uint32_t epoch,sample,solve_us,model_us,model_version;
    mpc_status_t status;
    uint16_t iterations;
    float primal,dual;
    uint8_t valid,constraint_active; /* 只诊断：成功序列任一物理输入距界<=0.001Nm。 */
} balance_mpc_solution_t;
typedef struct {
    mpc_model_t model;
    float A[100],B[40],terminal[100],Ct[BALANCE_MPC_N*100u];
    float P[(BALANCE_MPC_N+1u)*100u],K[BALANCE_MPC_N*40u],Hinv[BALANCE_MPC_N*16u];
    float scratch[220],storage[BALANCE_MPC_WORK_SIZE];
    float q[(BALANCE_MPC_N+1u)*10u],r[BALANCE_MPC_N*4u],affine[BALANCE_MPC_N*10u];
    float lower[BALANCE_MPC_N*4u],upper[BALANCE_MPC_N*4u];
    float length[2];
    uint32_t epoch,model_version;
    uint8_t model_valid;
} balance_mpc_context_t;
/* current length必须在原模型域，禁止外推；worker独占context。 */
mpc_status_t Balance_MPC_Model(balance_mpc_context_t *context,const float length[2]);
mpc_status_t Balance_MPC_Bounds(const leg_state_t *left,const leg_state_t *right,
    const float force[2],const leg_compensation_values_t *values,
    const lqr_runtime_config_t *config,float lower[4],float upper[4]);
void Balance_MPC_Solve(balance_mpc_context_t *context,const balance_mpc_input_t *input,
    balance_mpc_solution_t *solution);
#endif
