/* Vehicle writes; LINE reads once. Failed coherent reads fail closed. */
#include "grace.h"
#include "../platform/app_board.h"
typedef struct {
    uint32_t epoch;
    uint32_t revision;
    uint32_t revision_ms;
    uint32_t start;
    uint32_t active;
} blg_snapshot_t;
static APP_BOARD(blg_snapshot_t) board;
void blg_publish(const barcode_live_control_t *c, uint32_t start, bool active)
{
    const blg_snapshot_t s = {.epoch = c->epoch,
        .revision = c->eligibility_revision, .revision_ms = c->revision_ms,
        .start = start, .active = active ? 1u : 0u};
    app_board_publish(&board.sequence, board.words, &s, sizeof(s));
}
bool blv_decode_grace_active(const barcode_live_control_t *c, uint32_t now)
{
    blg_snapshot_t s;
    return c && c->mode == BC_LIVE_DISABLED &&
        app_board_try_read(&board.sequence, board.words, &s, sizeof(s)) &&
        s.active && s.epoch == c->epoch &&
        s.revision == c->eligibility_revision &&
        s.revision_ms == c->revision_ms && now - s.start < BLG_SEARCH_MS;
}
