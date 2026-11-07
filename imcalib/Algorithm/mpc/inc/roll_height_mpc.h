/* 四态物理适配：仅NORMAL且轴向未释放时调用，输入补偿缓存已准备。
 * F为映射前N，Tp为本机映射摆矩Nm；原始状态/目标每拍已更新一次。
 * 通用内核不接触机器人量；本接口负责单位转换、当前区间与LQR回退。 */
#ifndef ROLL_HEIGHT_MPC_H
#define ROLL_HEIGHT_MPC_H
#include "roll_height_mpc_data.h"
#include "leg_compensation.h"
#include "lqr_balance.h"
struct roll_height_balance;
typedef struct {
    float storage[RH_MPC_WORK_SIZE];
    float reference[(RH_MPC_N+1u)*4u];
    float q[(RH_MPC_N+1u)*4u], r[RH_MPC_N*2u], affine[RH_MPC_N*4u];
    float lo[RH_MPC_N*2u], hi[RH_MPC_N*2u];
    float lower[2], upper[2], candidate[2];
    float reference_gain[4]; /* 预览速度方程对输入的2x2逆，仅模型初始化时计算。 */
    float model_affine[4], hold_force[2]; /* 重采样仿射项；本次MPC总F零阶保持值。 */
    mpc_result_t result;
    uint32_t elapsed_us, fallback_count, solve_count;
    uint8_t backend; /* 0原LQR，1MPC，2受限LQR，3空区间LQR，4模型不匹配，5约束输入无效 */
    uint8_t initialized;
    uint8_t hold_valid;
    uint16_t solve_countdown; /* 正常情况下解算一次后跳过solve_div-1个基础拍。 */
} roll_height_mpc_t;
mpc_status_t Roll_Height_MPC_Bounds(const leg_state_t *left, const leg_state_t *right,
    const float Tp[2], const leg_compensation_values_t *values,
    const lqr_runtime_config_t *config, float lower[2], float upper[2]);
uint8_t Roll_Height_MPC_Input_Reference(const roll_height_mpc_t *context,const float current[4],const float next[4],float input[2]);
void Roll_Height_MPC_Update(struct roll_height_balance *state, const lqr_state_t *lqr,
    const leg_state_t *left, const leg_state_t *right,
    const leg_compensation_values_t *values, const lqr_runtime_config_t *config);
#endif
