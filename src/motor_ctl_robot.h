/*
 * Motor controller settings for this robot (docs/ADR_MOTOR_CONTROLLER.md).
 * The hardware quirks live here and nowhere else.
 *
 * Sources:
 *   - 2535 counts per wheel turn, wheel 65 mm: 12414 counts per metre;
 *     track 111 mm: 24.05 counts per degree (docs/CALIBRATION_RESULTS.md).
 *   - Feed-forward: rough fit to the loaded track logs of 2026-11-03 and
 *     2026-11-04 (duty against counts per 50 ms, divided by 5 for the
 *     10 ms tick). Forward, both wheels: 14 % -> about 6, 20 % -> 15,
 *     32 % -> 25 counts per tick. Reverse: 14 % -> about 5, 20 % -> 10
 *     (left) and 12 (right). MEASURE on the bench: the logs mix straight
 *     drives, turns and curves.
 *   - Start kick 12 %, 10 counts, 250 ms (start policy, docs/CALIBRATION_RESULTS.md;
 *     a loaded wheel does not turn below about 12 %).
 *   - Servo: 1500 us centre, safe range 1100-1900 us, 100/9 us per degree
 *     (src/servo.h, detour_test servo_pulse()); settle estimate 42 ms +
 *     3.6 ms per degree, fitted to the scan settle times (150 ms for 30
 *     degrees, 60 ms for 5 degrees, src/obstacle_scan.h).
 *   - P and I: first values from Motion (Buddy 2) scaled to the 10 ms
 *     tick. TUNE on the bench.
 */
#ifndef MOTOR_CTL_ROBOT_H
#define MOTOR_CTL_ROBOT_H

#include "motor_ctl.h"

#define MC_ROBOT_CONFIG                                                                  \
    {                                                                                    \
        .tick_ms = 10u,                                                                  \
        .counts_per_m = 12414u,                                                          \
        .counts_per_deg_x100 = 2405u,                                                    \
        .ff_offset_permille = {{85u, 85u}, {85u, 90u}},                                  \
        .ff_slope_q8 = {{2304u, 3072u}, {2304u, 2406u}}, /* 9.0, 12.0; 9.0, 9.4 */      \
        .kp_q8 = 1536u,                                  /* 6.0 */                       \
        .ki_q8 = 128u,                                   /* 0.5 per tick */              \
        .integral_max_permille = 400u,                                                   \
        .accel_cpt_q8 = 512u, /* 2 counts/tick per tick: about 1600 mm/s^2 */            \
        .max_permille = 800u,                                                            \
        .kick_permille = 120u,                                                           \
        .kick_counts = 10u,                                                              \
        .kick_timeout_ms = 250u,                                                         \
        .stall_min_permille = 150u,                                                      \
        .stall_timeout_ms = 500u,                                                        \
        .stale_ms = 100u,                                                                \
        .servo_centre_us = 1500u,                                                        \
        .servo_min_us = 1100u,                                                           \
        .servo_max_us = 1900u,                                                           \
        .servo_us_per_deg_x10 = 111u,                                                    \
        .servo_settle_base_ms = 42u,                                                     \
        .servo_settle_per_deg_x10_ms = 36u,                                              \
    }

#endif
