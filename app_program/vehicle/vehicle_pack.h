/* Vehicle observation storage only. Public decoded API is unchanged. */
#ifndef VEHICLE_PACK_H
#define VEHICLE_PACK_H
#include "vehicle_board.h"

/* Words 0..6: timestamp, sequence, wake_lost, overruns, response_max_us,
 * handoffs, command_id (full u32). Word 7: signed speeds, L low/R high.
 * Word 8: servo+36 [0:6], state [7:8], module [9:11], phase [12:13],
 * opcode [14:16], status [17:19], active [20], calibrated [21].
 * Bits 22..31 are zero. Vehicle is the sole writer. Publish through the
 * existing aligned atomic-word board; decode only a stable local copy.
 * No shared read-modify-write, C bit-fields or unaligned packed accesses.
 */
typedef struct { uint32_t words[9]; } vehicle_packed_t;

/* Invalid ranges/codes, reserved bits or NULL leave destination unchanged. */
bool vehicle_pack(vehicle_packed_t *out, const vehicle_snapshot_t *value);
bool vehicle_unpack(vehicle_snapshot_t *out, const vehicle_packed_t *packed);
#endif
