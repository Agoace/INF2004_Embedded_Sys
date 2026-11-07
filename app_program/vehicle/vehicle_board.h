#ifndef VEHICLE_BOARD_H
#define VEHICLE_BOARD_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t timestamp_us;
    uint32_t sequence;
    uint32_t wake_lost;
    uint32_t overruns;
    uint32_t response_max_us;
    uint32_t handoffs;
    uint32_t command_id;
    int16_t left_mm_s;
    int16_t right_mm_s;
    int8_t servo_deg;
    uint8_t state;
    uint8_t module;
    uint8_t phase;
    uint8_t command_opcode;
    uint8_t command_status;
    bool active;
    bool calibrated;
} vehicle_snapshot_t;

/* Bounded read-only last vehicle step; E_TMOUT busy, E_OBJ no vehicle role. */
int vehicle_snapshot_try_read(vehicle_snapshot_t *out);
/* Pure policy result: 0 accepted,1not_idle,2uncalibrated,3busy_log,5badop. */
uint8_t vehicle_remote_policy(uint8_t opcode, bool active, bool pending,
                              bool log_busy, bool calibrated);
#endif
