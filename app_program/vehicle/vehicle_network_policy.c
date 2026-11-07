/*
 * vehicle_network_policy.c - application telemetry and service interface.
 */
#include "vehicle_board.h"

/*
 * Check idle, calibration and log policy without executing a command.
 */
uint8_t vehicle_remote_policy(uint8_t opcode, bool active, bool pending,
                              bool log_busy, bool calibrated)
{
    if (opcode == 1u || opcode == 2u) {
        return 0u;
    }
    if (opcode != 3u && opcode != 4u) {
        return 5u;
    }
    if (active || pending) {
        return 1u;
    }
    if (log_busy) {
        return 3u;
    }
    if (opcode == 4u && !calibrated) {
        return 2u;
    }
    return 0u;
}
