/* Pure operator-only straight-pass bounds. No motion/kernel dependencies. */
#ifndef BARCODE_DRIVE_H
#define BARCODE_DRIVE_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BC_DRIVE_IDLE,
    BC_DRIVE_DISTANCE,
    BC_DRIVE_BUTTON,
    BC_DRIVE_TIMEOUT,
    BC_DRIVE_FAULT,
    BC_DRIVE_COMMAND,
    BC_DRIVE_CAPACITY,
    BC_DRIVE_DIRECTION,
    BC_DRIVE_DIVERGENCE,
    BC_DRIVE_OVERRUN
} barcode_drive_reason_t;

typedef struct {
    uint32_t distance_mm;
    uint32_t speed_mm_s;
    uint32_t counts_per_m;
} barcode_drive_config_t;

typedef struct {
    uint32_t started_ms;
    int32_t origin[2];
    int32_t travel_mm;
    int32_t wheel_mm[2];
    barcode_drive_reason_t reason;
    bool active;
    bool backward;
    bool next_backward;
    bool inhibited;
    bool overrun; /* Latched per pass, compares counts before mm rounding. */
} barcode_drive_t;

const char *barcode_drive_reason_name(barcode_drive_reason_t reason);
bool barcode_drive_config_valid(const barcode_drive_config_t *config);
uint32_t barcode_drive_timeout_ms(const barcode_drive_config_t *config);
/* Includes 2 s storage margin after the motion timeout. */
uint32_t barcode_drive_adc_ms(const barcode_drive_config_t *config);
uint32_t barcode_drive_motor_ms(const barcode_drive_config_t *config);
void barcode_drive_cancel(barcode_drive_t *job, barcode_drive_reason_t reason);
void barcode_drive_observe(barcode_drive_t *job,
                           const barcode_drive_config_t *config, int32_t left,
                           int32_t right);
/* Press starts only if idle/healthy, or cancels if already active. A fault
 * inhibits future starts until reboot; this function never confirms a stop.
 * Caller must confirm stop before decode, dump or accepting another start. */
int16_t barcode_drive_step(barcode_drive_t *job,
                           const barcode_drive_config_t *config,
                           uint32_t now_ms, int32_t left, int32_t right,
                           bool press, bool fault);
#endif
