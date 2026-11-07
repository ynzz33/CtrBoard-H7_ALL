/* 通用有限时域输入盒约束MPC；矩阵行优先，序列节点优先，无硬件依赖。 */
#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H
#include <stdint.h>
#include <stddef.h>
typedef struct {
    uint16_t nx, nu, horizon;
    const float *A, *B, *Q;
    const float *P; /* (N+1)*nx*nx，终端为原物理DARE代价。 */
    const float *K; /* N*nu*nx */
    const float *Hinv; /* N*nu*nu，含输入分裂rho的有限因子。 */
    float rho;
    const float *Ct; /* N*nx*nx，逐阶段(A-BK)^T；NULL时求解入口构建。 */
    float relaxation; /* ADMM过松弛，0按1处理；合法范围(0,2)。 */
} mpc_model_t;
typedef struct {
    const mpc_model_t *model;
    const float *state; /* nx；坐标变换由适配层完成。 */
    const float *linear_state; /* (N+1)*nx，代价q[k]'x[k]，终端亦包含。 */
    const float *linear_input; /* N*nu，代价r[k]'u[k]。二次项统一使用1/2系数。 */
    const float *affine; /* N*nx，x[k+1]=A*x[k]+B*u[k]+affine[k]。 */
    const float *input_min, *input_max; /* N*nu，求解变量坐标。 */
    uint16_t max_iter;
    float primal_tolerance, dual_tolerance;
    uint8_t constraints_enabled;
    uint8_t (*should_stop)(void *context); /* 可为空；在迭代边界检查，不读取硬件时钟。 */
    void *stop_context;
} mpc_problem_t;
typedef enum { MPC_OK, MPC_ITERATION_LIMIT, MPC_INVALID, MPC_INFEASIBLE, MPC_TIME_LIMIT } mpc_status_t;
typedef struct {
    float *storage;
    size_t capacity; /* float个数；每个控制实例独占，不跨任务共享可写历史。 */
} mpc_workspace_t;
typedef struct {
    mpc_status_t status;
    uint16_t iterations;
    float primal, dual;
    const float *input; /* N*nu，投影序列；只在MPC_OK时可用于控制。 */
} mpc_result_t;
#define MPC_WORKSPACE_FLOATS(n,a,N) (2u*((N)+1u)*(n)+5u*(N)*(a)+2u*(n)+2u*(a)+(N)*(n)+(N)*(a)+(N)*(n)*(n))
void MPC_Dynamics_Cache(const mpc_model_t *model,float *Ct);
size_t MPC_Workspace_Size(const mpc_model_t *model); /* float个数，不是字节。 */
void MPC_Reset(mpc_workspace_t *workspace);
/* 把上次解按shift个节点前移，保留对偶；只作热启动初值，不改变收敛验收。 */
mpc_status_t MPC_Shift_Warm_Start(const mpc_model_t *model, mpc_workspace_t *workspace, uint16_t shift);
mpc_status_t MPC_Solve(const mpc_problem_t *problem, mpc_workspace_t *workspace, mpc_result_t *result);
#endif
