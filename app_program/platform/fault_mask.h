/*
 * Diagnostic fault mirrors only: never an actuator input or a new latch.
 * One writer per owner board. Health alone updates the combined snapshot.
 */
#ifndef APP_FAULT_MASK_H
#define APP_FAULT_MASK_H

#include <stdint.h>
#include "app_board.h"

#define FAULT_MOTION_STALL_LEFT  (UINT32_C(1) << 0)
#define FAULT_MOTION_STALL_RIGHT (UINT32_C(1) << 1)
#define FAULT_MOTION_ESTOP       (UINT32_C(1) << 2)
#define FAULT_MOTION_STALE       (UINT32_C(1) << 3) /* Not latched. */
#define FAULT_IMU_SAFETY         (UINT32_C(1) << 4)
#define FAULT_SONAR_SENSOR       (UINT32_C(1) << 5)
#define FAULT_SHELL_OTHER        (UINT32_C(1) << 6)
#define FAULT_LOG_FULL           (UINT32_C(1) << 7)
#define FAULT_WIFI_LINK_DOWN     (UINT32_C(1) << 8)
#define FAULT_MQTT_DISCONNECTED  (UINT32_C(1) << 9)
#define FAULT_MASK_ALL          UINT32_C(0x3ff)

#define FAULT_OWNER_MOTION      UINT32_C(0x00f)
#define FAULT_OWNER_VEHICLE     (FAULT_IMU_SAFETY | FAULT_SHELL_OTHER)
#define FAULT_OWNER_SONAR       FAULT_SONAR_SENSOR
#define FAULT_OWNER_TELEMETRY   FAULT_LOG_FULL
#define FAULT_OWNER_NETWORK    (FAULT_WIFI_LINK_DOWN | FAULT_MQTT_DISCONNECTED)

/* Zero initialization means unpublished, not a successful all-clear value.
 * This type contains no global storage. Each module allocates its own board.
 * Do not give multiple writers different bits of the same board.
 */
typedef struct {
    uint32_t sequence;
    uint32_t word;
} fault_mask_board_t;
_Static_assert(sizeof(fault_mask_board_t) == 8u, "Fault owner board size");
_Static_assert(_Alignof(fault_mask_board_t) >= 4u, "Aligned fault atomics");
_Static_assert(__builtin_offsetof(fault_mask_board_t, word) == 4u, "Aligned fault word");

/* Health-owned state. valid marks the fault bits whose source was copied
 * successfully in this collection, including successfully observed zeros.
 * On a failed read, now keeps the last copied value with valid bits clear.
 * Thus unknown sources are never an implicit current all-clear.
 * run is the OR of successful copies in this epoch, not a count and not
 * proof that all transient faults were sampled. epoch uses rt_epoch().
 */
typedef struct {
    uint32_t now;
    uint32_t run;
    uint32_t valid;
    uint32_t epoch;
} fault_mask_snapshot_t;

static inline bool fault_mask_owner_valid(uint32_t owner_mask)
{
    return owner_mask != 0u && (owner_mask & ~FAULT_MASK_ALL) == 0u;
}

/* Writer only. Reject wrong-owner/reserved bits without changing the board. */
static inline bool fault_mask_publish(fault_mask_board_t *board,
                                      uint32_t owner_mask, uint32_t value)
{
    if (board == 0 || !fault_mask_owner_valid(owner_mask) ||
        (value & ~owner_mask) != 0u) {
        return false;
    }
    app_board_publish(&board->sequence, &board->word, &value, sizeof(value));
    return true;
}

/* Bounded, cross-core read. Any failure leaves out unchanged. */
static inline bool fault_mask_try_read(const fault_mask_board_t *board,
                                       uint32_t owner_mask, uint32_t *out)
{
    uint32_t value;
    if (board == 0 || out == 0 || !fault_mask_owner_valid(owner_mask) ||
        !app_board_try_read(&board->sequence, &board->word,
                            &value, sizeof(value)) ||
        (value & ~owner_mask) != 0u) {
        return false;
    }
    *out = value;
    return true;
}

/* Health only, once before collecting owner values. Run history resets on
 * an epoch change, including wrap. No owner fault is cleared by this call.
 */
static inline void fault_mask_collect_begin(fault_mask_snapshot_t *state,
                                            uint32_t epoch)
{
    if (state->epoch != epoch) {
        state->run = 0u;
        state->epoch = epoch;
    }
    state->valid = 0u;
}

/* Health only. NULL or invalid data means unavailable, not zero/cleared. */
static inline bool fault_mask_combine(fault_mask_snapshot_t *state,
                                      uint32_t owner_mask,
                                      const uint32_t *value)
{
    if (!fault_mask_owner_valid(owner_mask)) {
        return false;
    }
    state->valid &= ~owner_mask;
    if (value == 0 || (*value & ~owner_mask) != 0u) {
        return false;
    }
    state->now = (state->now & ~owner_mask) | *value;
    state->run |= *value;
    state->valid |= owner_mask;
    return true;
}

/* Owner modules provide these bounded getters as their F changes land.
 * Each returns true for a coherent copy (zero included), false with out
 * unchanged otherwise. Motion uses its existing observation board instead.
 * No new provider is linked or called merely by including this header.
 */
bool sonar_fault_mask_try_read(uint32_t *out);
bool vehicle_fault_mask_try_read(uint32_t *out);
bool telemetry_fault_mask_try_read(uint32_t *out);
bool robot_network_fault_mask_try_read(uint32_t *out);

#endif
