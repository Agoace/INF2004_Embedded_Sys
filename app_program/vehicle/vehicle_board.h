#ifndef VEHICLE_BOARD_H
#define VEHICLE_BOARD_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t timestamp_us, sequence, wake_lost, overruns, response_max_us,
        handoffs, command_id;
    int16_t left_mm_s, right_mm_s;
    int8_t servo_deg;
    uint8_t state, module, phase, command_opcode, command_status;
    bool active, calibrated;
} vehicle_snapshot_t;

/* Bounded read-only last vehicle step; E_TMOUT busy, E_OBJ no vehicle role. */
int vehicle_snapshot_try_read(vehicle_snapshot_t *out);
/* Pure policy result: 0 accepted,1not_idle,2uncalibrated,3busy_log,5badop. */
uint8_t vehicle_remote_policy(uint8_t opcode, bool active, bool pending,
                              bool log_busy, bool calibrated);
#endif
