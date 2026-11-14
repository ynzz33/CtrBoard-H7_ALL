#ifndef JETSON_SHADOW_PROTOCOL_H
#define JETSON_SHADOW_PROTOCOL_H

#include "jetson_protocol.h"

/* Shadow frames use the JRL2 header and CRC. Both input halves carry the
 * same session, reset generation, sequence and H7 input-ready timestamp.
 * The reply echoes all four fields. None of these frames grants control. */
enum {
    JRL_SHADOW_INPUT_FIRST = 6,
    JRL_SHADOW_INPUT_SECOND = 7,
    JRL_SHADOW_REPLY = 8,
    JRL_SHADOW_RESULT = 9,
    JRL_SHADOW_RESULT_BYTES = 76,
    JRL_SHADOW_INPUT_FLOATS = 150,
    JRL_SHADOW_HALF_FLOATS = 75,
    JRL_SHADOW_ACTION_FLOATS = 6,
    JRL_SHADOW_INPUT_BYTES = 4 + JRL_SHADOW_HALF_FLOATS * 4,
    JRL_SHADOW_REPLY_BYTES = 4 + JRL_SHADOW_ACTION_FLOATS * 4 + 4
};

/* Payload starts with the little-endian contract. Input floats are the
 * concatenated obs[25], history[125]. Reply floats are raw action[6],
 * followed by Jetson inference duration in microseconds (uint32_t). */
#define JRL_SHADOW_CONTRACT 3500301u

#endif
