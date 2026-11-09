#include "../app_program/subsystem3/barcode_drive.h"
#include "../app_program/subsystem3/barcode_settings.h"
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

int main(void)
{
    assert(BARCODE_BOUNDED_DISTANCE_MM <= 620u);
    assert(strcmp(barcode_drive_reason_name(BC_DRIVE_OVERRUN), "OVERRUN") == 0);
    assert(strcmp(barcode_drive_reason_name((barcode_drive_reason_t)999),
                  "UNKNOWN") == 0);
#ifdef EXPECT_CLAMP
    assert(BARCODE_BOUNDED_DISTANCE_MM == 620u);
#else
    assert(BARCODE_BOUNDED_DISTANCE_MM == 600u);
#endif
    barcode_drive_config_t config = {600u, 100u, 1000u};
    assert(barcode_drive_config_valid(&config));
    assert(barcode_drive_timeout_ms(&config) == 9000u);
    assert(barcode_drive_adc_ms(&config) == 2u);
    assert(barcode_drive_motor_ms(&config) == 30u);
    barcode_drive_t job = {0};
    assert(barcode_drive_step(&job, &config, 0u, 0, 0, false, false) == 0);
    assert(barcode_drive_step(&job, &config, 10u, 50, 60, true, false) == 100);
    assert(job.active && !job.backward && job.next_backward);
    assert(barcode_drive_step(&job, &config, 6000u, 649, 659, false, false) ==
           100);
    assert(job.travel_mm == 599);
    assert(barcode_drive_step(&job, &config, 6010u, 650, 660, false, false) ==
           0);
    assert(!job.active && job.reason == BC_DRIVE_DISTANCE);
    assert(barcode_drive_step(&job, &config, 6100u, 650, 660, true, false) ==
           -100);
    assert(job.active && job.backward && !job.next_backward);
    assert(barcode_drive_step(&job, &config, 12100u, 50, 60, false, false) ==
           0);
    assert(job.travel_mm == 600 && job.reason == BC_DRIVE_DISTANCE);
    assert(barcode_drive_step(&job, &config, 13000u, 50, 60, true, false) ==
           100);
    assert(barcode_drive_step(&job, &config, 13100u, 55, 65, true, false) == 0);
    assert(job.reason == BC_DRIVE_BUTTON && !job.inhibited);
    job = (barcode_drive_t){0};
    (void)barcode_drive_step(&job, &config, UINT32_MAX - 100u, INT32_MAX - 100,
                             INT32_MAX - 100, true, false);
    assert(barcode_drive_step(&job, &config, 6000u, INT32_MIN + 499,
                              INT32_MIN + 499, false, false) == 0);
    assert(job.reason == BC_DRIVE_DISTANCE && job.travel_mm == 600);
    job = (barcode_drive_t){0};
    (void)barcode_drive_step(&job, &config, 0u, 0, 0, true, false);
    assert(barcode_drive_step(&job, &config, 9000u, 0, 0, false, false) == 0);
    assert(job.reason == BC_DRIVE_TIMEOUT && job.inhibited);
    assert(barcode_drive_step(&job, &config, 10000u, 0, 0, true, false) == 0);
    job = (barcode_drive_t){0};
    (void)barcode_drive_step(&job, &config, 0u, 0, 0, true, false);
    (void)barcode_drive_step(&job, &config, 100u, -21, -21, false, false);
    assert(job.reason == BC_DRIVE_DIRECTION && job.inhibited);
    job = (barcode_drive_t){0};
    (void)barcode_drive_step(&job, &config, 0u, 0, 0, true, false);
    (void)barcode_drive_step(&job, &config, 100u, 150, 1, false, false);
    assert(job.reason == BC_DRIVE_DIVERGENCE && job.inhibited);
    job = (barcode_drive_t){0};
    (void)barcode_drive_step(&job, &config, 0u, 0, 0, true, true);
    assert(job.reason == BC_DRIVE_FAULT && job.inhibited && !job.active);
    job = (barcode_drive_t){.active = true};
    barcode_drive_cancel(&job, BC_DRIVE_COMMAND);
    assert(!job.active && job.inhibited && job.reason == BC_DRIVE_COMMAND);
    config.speed_mm_s = 25u;
    assert(barcode_drive_config_valid(&config));
    assert(barcode_drive_timeout_ms(&config) == 27000u);
    assert(barcode_drive_adc_ms(&config) == 5u);
    assert(barcode_drive_motor_ms(&config) == 60u);
    config.distance_mm = 621u;
    assert(!barcode_drive_config_valid(&config));
    config.distance_mm = 600u;
    config.speed_mm_s = 0u;
    assert(!barcode_drive_config_valid(&config));
    assert(barcode_drive_timeout_ms(&config) == 0u);
    assert(!barcode_drive_config_valid(NULL));
    assert(barcode_drive_step(NULL, NULL, 0u, 0, 0, true, false) == 0);
    barcode_drive_cancel(NULL, BC_DRIVE_FAULT);
    config = (barcode_drive_config_t){600u, 100u, 1000u};
    job = (barcode_drive_t){0};
    (void)barcode_drive_step(&job, &config, 0u, 0, 0, true, false);
    assert(barcode_drive_step(&job, &config, 10u, -1, -1, false, false) == 100);
    barcode_drive_observe(&job, &config, 605, 607);
    assert(job.travel_mm == 606 && job.wheel_mm[0] == 605 && !job.overrun);
    config.counts_per_m = 10000u;
    config.distance_mm = 620u;
    job = (barcode_drive_t){0};
    assert(barcode_drive_step(&job, &config, 0u, 0, 0, true, false) == 100);
    assert(barcode_drive_step(&job, &config, 6199u, 6199, 6199, false, false) ==
           100);
    assert(barcode_drive_step(&job, &config, 6200u, 6200, 6200, false, false) ==
           0);
    assert(job.reason == BC_DRIVE_DISTANCE && !job.overrun);
    barcode_drive_observe(&job, &config, 6201, 6201);
    assert(job.wheel_mm[0] == 620 && job.overrun); /* 620.1 mm, not rounded. */
    barcode_drive_observe(&job, &config, 6199, 6199);
    assert(job.overrun); /* Peak remains latched if wheels roll back. */
    return 0;
}
