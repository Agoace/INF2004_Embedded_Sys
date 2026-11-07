#include <assert.h>
#include <string.h>
#include "../app_program/subsystem2/motion_diag.h"
#include "../app_program/subsystem2/motion_diag_capture.h"

int main(void)
{
    md_capture_t capture = {0};
    int side_effect = 0;
    motion_diag_phase(++side_effect);
    motion_diag_dump(++side_effect);
    motion_diag_print();
    assert(side_effect ==
           0); /* default diagnostic calls disappear completely */
    md_release(&capture, 10000u);
    md_release(&capture, 20000u);
    assert(capture.release == 20000u && capture.release_gap == 10000u);
    md_point_t point = {.entry = 20001u,
                        .gap = 10000u,
                        .release = capture.release,
                        .release_gap = capture.release_gap,
                        .acquired = 20003u,
                        .phase = MOTION_DIAG_RUN | (3u << 8u),
                        .prior_owner = "motion_speed",
                        .prior_hold = 3u};
    md_observe(&capture, &point);
    assert(capture.observations == 0u);
    /* Scheduler/preemption delay, not mutex wait or late cyclic dispatch. */
    point.entry = 21038u;
    point.gap = 11038u;
    point.acquired = 21040u;
    md_observe(&capture, &point);
    assert(capture.observations == 1u && capture.worst.gap == 11038u);
    assert(capture.latest.entry - capture.latest.release == 1038u);
    assert(capture.latest.acquired - capture.latest.entry == 2u);
    /* A mutex delay alone does not lengthen the current entry-to-entry gap. */
    point.entry = 30000u;
    point.gap = 10000u;
    point.release = 30000u;
    point.acquired = 31086u;
    point.phase = MOTION_DIAG_STOP | MD_DUMP_FLAG;
    point.prior_owner = "motion_estop";
    point.prior_hold = 1086u;
    point.queued = 1;
    md_observe(&capture, &point);
    assert(capture.observations == 2u && capture.worst.gap == 11038u);
    assert(capture.latest.gap == 10000u && capture.latest.queued == 1);
    assert(capture.latest.phase == (MOTION_DIAG_STOP | MD_DUMP_FLAG));
    assert(!strcmp(capture.latest.prior_owner, "motion_estop"));
    assert(capture.latest.prior_hold == 1086u);
    assert(capture.latest.acquired - capture.latest.entry == 1086u);
    /* Release lateness is distinct from post-release dispatch lateness. */
    point.release_gap = 11050u;
    point.release = UINT32_MAX - 9u;
    point.entry = 0u;
    point.acquired = 2u;
    point.gap = 11050u;
    md_observe(&capture, &point);
    assert(capture.observations == 3u && capture.worst.gap == 11050u);
    assert(capture.worst.entry - capture.worst.release == 10u);
    assert(capture.worst.release_gap == 11050u);
    assert(capture.worst.acquired - capture.worst.entry == 2u);
    return 0;
}
