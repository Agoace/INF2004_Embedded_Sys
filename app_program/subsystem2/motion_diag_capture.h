/* Pure fixed snapshots, not a queue. Included only in DIAG=1 and host tests. */
#ifndef MOTION_DIAG_CAPTURE_H
#define MOTION_DIAG_CAPTURE_H
#include <stdint.h>
#include <stdbool.h>
#define MD_DUMP_FLAG     0x10000u
#define MD_GAP_LIMIT_US  11000u
#define MD_WAIT_LIMIT_US 1000u

typedef struct {
    uint32_t entry;
    uint32_t gap;
    uint32_t release;
    uint32_t release_gap;
    uint32_t acquired;
    uint32_t phase;
    const char *prior_owner;
    uint32_t prior_hold;
    int32_t queued;
} md_point_t;

typedef struct {
    volatile uint32_t release;
    volatile uint32_t release_gap;
    volatile uint32_t phase;
    volatile uint32_t dumping;
    uint32_t last_release;
    uint32_t locked_at;
    const char *owner;
    const char *prior_owner;
    uint32_t prior_hold;
    uint32_t observations;
    md_point_t latest;
    md_point_t worst;
} md_capture_t;

static inline void md_release(md_capture_t *s, uint32_t now)
{
    s->release_gap = now - s->last_release;
    s->last_release = now;
    s->release = now;
}

/* Called with the already-held motion mutex. No new locks or output. */
static inline void md_observe(md_capture_t *s, const md_point_t *point)
{
    if (point->gap > MD_GAP_LIMIT_US ||
        point->acquired - point->entry > MD_WAIT_LIMIT_US) {
        s->latest = *point;
        ++s->observations;
        if (s->observations == 1u || point->gap > s->worst.gap) {
            s->worst = *point;
        }
    }
}
#endif
