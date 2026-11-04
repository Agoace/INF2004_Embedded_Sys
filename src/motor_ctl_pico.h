/*
 * Motor controller, Pico layer (docs/MOTOR_CONTROLLER.md, step 2).
 *
 * The only code that writes the motor PWM and the servo PWM and reads the
 * wheel encoders (docs/ADR_MOTOR_CONTROLLER.md). Do not link it with
 * drive.c or servo.c: they drive the same pins.
 *
 *   Left wheel  : PWM GP8 (forward) / GP9 (reverse), encoder GP16/GP17
 *   Right wheel : PWM GP10 (forward) / GP11 (reverse), encoder GP4/GP5
 *   Servo       : GP15, 50 Hz
 *   Motor PWM   : 20 kHz (wrap 6249); 0 on both pins = coast
 *
 * Timing (docs/ADR_HEARTBEAT.md): mcp_init() adds a heartbeat hook every
 * cfg.tick_ms. The hook runs in the heartbeat interrupt: it reads the
 * encoders, runs mc_tick(), and writes the motor and servo PWM. The
 * main-loop calls below only store values; the next hook acts on them.
 *
 * Sharing: every call takes a critical section (interrupts off on this
 * core and a hardware spin lock), so it is safe from core 0 and core 1.
 * Each section is a few microseconds.
 *
 * Encoders: own GPIO callback (gpio_set_irq_enabled_with_callback), as in
 * drive.c. Other GPIO interrupt users must use a raw handler
 * (ultrasonic.c does).
 */
#ifndef MOTOR_CTL_PICO_H
#define MOTOR_CTL_PICO_H

#include <stdbool.h>
#include <stdint.h>

#include "heartbeat.h"
#include "motor_ctl.h"

typedef struct {
    int32_t counts[MC_WHEELS];         /* forward positive, since mcp_init() */
    int32_t travel_mm[MC_WHEELS];      /* since mcp_init() */
    int16_t speed_mm_s[MC_WHEELS];     /* last tick */
    int16_t duty_permille[MC_WHEELS];  /* last tick, signed */
    int32_t heading_cdeg;              /* since mcp_init(), left positive */
    uint16_t servo_us;
    int8_t servo_deg;
    bool servo_settled;
    uint8_t faults;                    /* MC_FAULT_* */
    uint32_t ticks;                    /* mc_tick() calls */
    uint32_t tick_max_us;              /* longest hook time */
} mcp_status_t;

/* Set up the pins, the PWM (all 0, servo centre), the encoders and the
 * core, and add the hook to hb. Call before heartbeat_start().
 * False: cfg.tick_ms is not a whole number of heartbeat ticks, or the
 * heartbeat hook table is full. */
bool mcp_init(hb_t *hb, const mc_config_t *cfg);

/* Main loop: as mc_set_speed(), mc_stop(), mc_estop(), mc_clear_faults(),
 * mc_set_servo(). */
void mcp_set_speed(int16_t left_mm_s, int16_t right_mm_s);
void mcp_stop(void);
void mcp_estop(void);
void mcp_clear_faults(void);
int mcp_set_servo(int deg);

/* Main loop: one consistent copy of all measurements. */
void mcp_status(mcp_status_t *s);

/* Milliseconds since boot: the clock of every mcp_* call. */
uint32_t mcp_now_ms(void);

#endif
