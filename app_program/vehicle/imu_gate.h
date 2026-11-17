/* Pure Vehicle safety gate. Port of smojjofy 48b5f9b..038ff23.
 * No kernel, GPIO, bus, console or actuator access. Vehicle is sole owner.
 * A request is not an acknowledgement; only the exact writer receipt clears.
 */
#ifndef VEHICLE_IMU_GATE_H
#define VEHICLE_IMU_GATE_H
#include <stdbool.h>
#include <stdint.h>
#include "../subsystem4/imu_safety.h"

#define IG_RETRY_MS 100u
#define IG_INTENT_MS 1000u

typedef enum {
    IG_READY = 0, IG_NO_BOARD = 1, IG_STALE = 2, IG_INVALID = 3,
    IG_POSE = 4, IG_ONE_G = 5, IG_WHEELS = 6, IG_WINDOW = 7,
    IG_PENDING = 8, IG_CANCELLED = 9
} ig_reason_t;

typedef struct {
    imu_safety_report_t report;
    uint32_t token, deadline, retry_at;
    uint8_t latched, cause;
    bool have, pending;
} ig_t;

typedef struct {
    bool stop_edge;       /* Request emergency stop and persistent FAULT once. */
    bool cleared;         /* Exact successful writer receipt, safe fresh pose. */
    bool request_clear;   /* Queue at most one writer request, never wait. */
    uint8_t recovery;     /* One transition observation, zero if none. */
} ig_action_t;

void ig_init(ig_t *g);
/* NULL keeps the previous coherent copy; age continues to advance. */
ig_action_t ig_step(ig_t *g, const imu_safety_report_t *report,
                     uint32_t now_ms, bool active);
bool ig_start_allowed(const ig_t *g, uint32_t now_ms);
bool ig_stop_required(const ig_t *g);
uint8_t ig_refusal(const ig_t *g, uint32_t now_ms);
/* Only a local operator press may begin this bounded intent. Returns true
 * when the caller must queue a request and return its token via ig_queued. */
bool ig_press(ig_t *g, uint32_t now_ms);
void ig_queued(ig_t *g, uint32_t token, uint32_t now_ms);
#endif
