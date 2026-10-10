#include "leg_compensation.h"
#include "gas_spring.h"

#include <string.h>

/* 补偿后限幅 */
uint8_t Leg_Compensation_Apply(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float limit,
    float output_dm[4], compensation_debug_t *debug)
{
    float raw[4];
    uint8_t i;

    memset(debug, 0, sizeof(*debug));
    if (!Gas_Spring_Apply(left, right, base_dm, raw))
    {
        return 0u;
    }
    for (i = 0u; i < 4u; i++)
    {
        debug->spring_dm[i] = raw[i] - base_dm[i];
    }
    if (!Gravity_Comp_Apply(left, right, world_angle, raw, raw, &debug->gravity))
    {
        memset(debug, 0, sizeof(*debug));
        return 0u;
    }
    memcpy(debug->raw_dm, raw, sizeof(raw));
    for (i = 0u; i < 4u; i++)
    {
        output_dm[i] = clampf(raw[i], -limit, limit);
        debug->saturated |= (uint8_t)(output_dm[i] != raw[i]);
    }
    return 1u;
}
