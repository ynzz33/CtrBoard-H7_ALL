#ifndef LEG_COMPENSATION_H
#define LEG_COMPENSATION_H

#include "gravity_comp.h"

typedef struct {
    gravity_comp_result_t gravity;
    float spring_dm[4];
    float raw_dm[4];
    uint8_t saturated;
    uint8_t bench;
} compensation_debug_t;

uint8_t Leg_Compensation_Apply(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float limit,
    float output_dm[4], compensation_debug_t *debug);

#endif
