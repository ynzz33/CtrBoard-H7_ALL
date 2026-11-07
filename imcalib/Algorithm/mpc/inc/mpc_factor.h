/* 从同一Ad/Bd/Q/R/终端P构建有限时域因子；调用方持有可写缓存，单位与求解器相同。 */
#ifndef MPC_FACTOR_H
#define MPC_FACTOR_H
#include "mpc_solver.h"
size_t MPC_Factor_Scratch_Size(const mpc_model_t *model); /* float个数。 */
mpc_status_t MPC_Factor_Build(mpc_model_t *model, const float *R, const float *terminal,
    float *P, float *K, float *Hinv, float *scratch, size_t capacity);
#endif
