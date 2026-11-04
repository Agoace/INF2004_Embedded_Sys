/*
 * Motor controller settings for this robot (docs/ADR_MOTOR_CONTROLLER.md).
 * The hardware quirks live here and nowhere else.
 *
 * Sources:
 *   - 2535 counts per wheel turn, wheel 65 mm: 12414 counts per metre;
 *     track 111 mm: 24.05 counts per degree (docs/CALIBRATION_RESULTS.md).
 *   - Feed-forward: straight-line fit to the steady points of floor bench
 *     run B1 (2026-11-04, docs/MOTOR_CONTROLLER.md): 20, 40, 150 and
 *     300 mm/s, each wheel and direction. Below 5 counts per tick the
 *     real duty is flat at about 80-97 permille (dead zone); the fit is
 *     about 10 permille low there and the I term adds the rest.
 *     (Step 1 used a rough fit to the track logs; its reverse values
 *     were too high.)
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
        .ff_offset_permille = {{48u, 59u}, {56u, 62u}},                                  \
        .ff_slope_q8 = {{2472u, 2275u}, {2404u, 2219u}}, /* 9.66, 8.89; 9.39, 8.67 */   \
        .kp_q8 = 1536u,                                  /* 6.0 */                       \
        .ki_q8 = 128u,                                   /* 0.5 per tick */              \
        .integral_max_permille = 400u,                                                   \
        .accel_cpt_q8 = 2048u, /* 8 counts/tick per tick: about 6400 mm/s^2 (P5) */      \
        .max_permille = 800u,                                                            \
        .max_speed_mm_s = 460u, /* 60 % open loop (duty map DM1); wheelies at 80 % */   \
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
