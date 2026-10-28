/*
 * Two-wheel drive: motor PWM and wheel encoders on the Robo Pico.
 *
 *   Left wheel  = motor 1: PWM GP8 (forward) / GP9 (reverse), encoder GP16/GP17
 *   Right wheel = motor 2: PWM GP10 (forward) / GP11 (reverse), encoder GP4/GP5
 *
 * Sides and encoder signs were measured on 2026-10-27 (CALIBRATION_RESULTS.md).
 * drive_counts() gives both wheels with forward = positive.
 * 2535 encoder counts = one wheel turn.
 *
 * The encoder interrupt uses the shared GPIO callback, so do not combine
 * this with another gpio_set_irq_enabled_with_callback() user.
 */
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

#define DRIVE_COUNTS_PER_WHEEL_TURN 2535

typedef enum {
    DRIVE_LEFT = 0,
    DRIVE_RIGHT = 1,
} drive_wheel_t;

void drive_init(void);

/* duty_pct: -100..+100, positive = forward. Values outside are limited. */
void drive_set_wheel(drive_wheel_t wheel, int duty_pct);
void drive_set(int left_pct, int right_pct);
void drive_stop(void);

/* Encoder counts since drive_init(), forward = positive. */
void drive_counts(int32_t *left, int32_t *right);

#endif
