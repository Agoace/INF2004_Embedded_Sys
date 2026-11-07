/*
 * telemetry_mode.c - MQTT publication mode; see telemetry_mode.h.
 */
#include "telemetry_mode.h"

void pm_init(pm_tracker_t *t)
{
    t->seen_busy = false;
    t->busy = false;
    t->mode = PM_MODE_RUN;
    t->idle_at = 0u;
}

pm_mode_t pm_update(pm_tracker_t *t, pm_board_t board, uint8_t phase,
                    bool active, uint32_t now_ms)
{
    bool busy;

    if (board == PM_BOARD_NONE) {
        t->mode = PM_MODE_RUN;
        return t->mode;
    }
    if (board == PM_BOARD_BUSY) {
        return t->mode;
    }
    busy = (phase != 0u) || active;
    if (busy) {
        t->seen_busy = true;
        t->mode = PM_MODE_RUN;
    } else {
        if (t->busy) {
            t->idle_at = now_ms;
        }
        /* Unsigned subtraction keeps the grace correct across a wrap. */
        if (!t->seen_busy ||
            ((uint32_t)(now_ms - t->idle_at) >= PM_RUN_GRACE_MS)) {
            t->mode = PM_MODE_IDLE;
        }
    }
    t->busy = busy;
    return t->mode;
}
