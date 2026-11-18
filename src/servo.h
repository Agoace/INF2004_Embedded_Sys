#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/*
 * Positional servo on one PWM pin, 50 Hz.
 *
 * Mounted limits (2026-10-27): the sensor mount touches the robot at about
 * 1040 us (41 degrees right) and 1960 us (41 degrees left). Every pulse is
 * clamped to SERVO_SAFE_MIN_US..SERVO_SAFE_MAX_US, about 36 degrees each
 * side, so a wrong value cannot drive the mount into the robot.
 *
 * Direction: a pulse below 1500 us turns the sensor right; above 1500 us
 * turns it left (seen from behind the robot).
 */
#define SERVO_CENTRE_US 1500u
#define SERVO_SAFE_MIN_US 1100u
#define SERVO_SAFE_MAX_US 1900u

/* Configure the pin for PWM and move to SERVO_CENTRE_US. */
void servo_init(unsigned int pin);

/* Set the pulse width. Values outside the safe range are clamped.
 * Returns the pulse that was set. */
uint16_t servo_set_pulse_us(uint16_t pulse_us);

#endif
