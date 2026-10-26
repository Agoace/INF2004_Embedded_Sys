#ifndef BUDDY1_MOTION_H
#define BUDDY1_MOTION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Component codec for Buddy 2's existing W1/W2 layout. Not a final MQTT packet.
 * Requires the real robot_shared.h from the agreed motion source tree.
 * The caller obtains a coherent private copy under the team's actual mutex.
 * No locks, RTOS objects, sensor access or controller actions occur here.
 * Input/output buffers must not overlap; decode outputs must be distinct. */
#define B1_MOTION_COMPONENT_BYTES 8u
bool b1_motion_encode(uint32_t w1, uint32_t w2, uint8_t *out, size_t length);
bool b1_motion_decode(const uint8_t *bytes, size_t length, uint32_t *w1, uint32_t *w2);
#endif
