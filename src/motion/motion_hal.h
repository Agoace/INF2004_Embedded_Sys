/**
 * @file motion_hal.h
 * @brief Hardware boundary for the motion core. The Pico build implements it
 *        in motion_hal_pico.c and encoder.c; the host unit tests implement it
 *        with a simulated robot (tests/motion/test_motion.c).
 */
#ifndef MOTION_HAL_H
#define MOTION_HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_shared.h"

/** Wrapping 16-bit count, forward positive (written only by the ISR). */
uint16_t motion_hal_encoder_count(motor_id_t motor);

/** Raw phase A and B edge totals since boot (self-test only). */
void     motion_hal_encoder_edges(motor_id_t motor,
                                  uint32_t * p_a_edges, uint32_t * p_b_edges);

/** Signed duty in permille (-1000..1000), forward positive. Called by the
 *  motion core only, always inside motion_hal_irq_save(). */
void     motion_hal_pwm_write(motor_id_t motor, int16_t permille);

/** Both motors to 0 duty. Safe from any context, including an ISR. */
void     motion_hal_pwm_zero_all(void);

uint32_t motion_hal_irq_save(void);
void     motion_hal_irq_restore(uint32_t state);
uint32_t motion_hal_time_us(void);

/** Logic-analyser marker (no-op if MOTION_DEBUG_GPIO < 0). */
void     motion_hal_debug_pin(bool level);

/** Pico build only: PWM to 0, encoder ISR, debug pin. Call before
 *  motion_init(). */
void     motion_hal_pico_init(void);

#endif /* MOTION_HAL_H */
