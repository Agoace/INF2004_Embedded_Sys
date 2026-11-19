/* Pure mirror of existing Vehicle-owned state; not an actuator input. */
#ifndef VEHICLE_FAULT_H
#define VEHICLE_FAULT_H
#include "../platform/fault_mask.h"
#include "vehicle_sm.h"
#include "../subsystem4/imu_safety.h"

static inline uint32_t vehicle_fault_value(uint8_t imu_latched,
                                           uint8_t state, uint8_t cause)
{
    uint32_t value = imu_latched != IMU_SAFETY_NONE ? FAULT_IMU_SAFETY : 0u;
    if (state == VSM_FAULT && cause != VSM_FAULT_SAFETY &&
        cause != VSM_FAULT_IMU) {
        value |= FAULT_SHELL_OTHER;
    }
    return value;
}
#endif
