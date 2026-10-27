/**
 * @file encoder.h
 * @brief A/B quadrature encoder ISR for both wheels (FR6, NFR5).
 *
 * Read-only API for every task: Line & Barcode (search limits), IMU (hump
 * distance, turn cross-check), Vehicle Controller and Telemetry. The ISR is
 * the only writer; counters are never reset. Use enc_delta16() or an
 * odometer_t from robot_shared.h to get travel.
 *
 * The ISR is installed with gpio_add_raw_irq_handler_masked(), NOT
 * gpio_set_irq_enabled_with_callback(). The SDK has only one shared GPIO
 * callback per core, so the raw handler leaves that callback free for the
 * ESTOP (GP20), Echo (GP2) and DRDY (GP6) ISRs of the other buddies.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "motion_units.h"
#include "robot_shared.h"

typedef struct
{
    uint32_t a_edges;       /* Total phase A edges since boot.           */
    uint32_t b_edges;       /* Total phase B edges since boot.           */
    uint32_t invalid;       /* Both phases changed at once (missed edge). */
} encoder_diag_t;

/** Configure pins and install the ISR. Call once, before motion_init(). */
void     encoder_init(void);

/** Wrapping 16-bit count, forward positive. Lock-free, any context. */
uint16_t encoder_count(motor_id_t motor);

/** Both counts read inside one critical section (same instant). */
void     encoder_count_pair(uint16_t * p_left, uint16_t * p_right);

void     encoder_get_diag(motor_id_t motor, encoder_diag_t * p_diag);

/* Unit conversions (counts <-> mm) are in motion_units.h. */

#endif /* ENCODER_H */
