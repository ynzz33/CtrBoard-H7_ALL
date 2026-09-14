#ifndef LEG_SOLVER_H
#define LEG_SOLVER_H

#include <stdint.h>

typedef struct {
    float lu;
    float lg;
    float offset_f;
    float offset_b;
    float offset_phi0;
    int8_t mirror;
    uint8_t configured;
} leg_config_t;

typedef struct {
    float hip_f;
    float hip_b;
    float d_hip_f;
    float d_hip_b;
} leg_input_t;

typedef struct {
    float l0;
    float phi0;
    float dl0;
    float dphi0;
    float lower_angle;
    float formula_virtual_shank;
    float virtual_shank;
    float d_virtual_shank;
    float vshank_jac[2];
    float point_jac[2][2];
    float leg_jac[2][2];
    float force_map[2][2];
    float force_det;
    float force_test_error;
    uint8_t force_valid;
    uint8_t valid;
} leg_output_t;

typedef struct {
    leg_config_t config;
    leg_input_t input;
    leg_output_t output;
} leg_state_t;

void Leg_Init(leg_state_t *leg);
uint8_t Leg_Solve(leg_state_t *leg);
uint8_t Leg_Force_Map_Forward(const leg_state_t *leg, float force,
                              float torque, float output[2]);

#endif
