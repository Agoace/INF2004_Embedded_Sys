/**
 * @file imu_core.h
 * @brief The IMU producer contract used by the future RoboCar Manager.
 *
 * Every entry point is non-blocking with respect to the 13 ms cadence: a step
 * performs at most one bus transaction pair. The core owns no GPIO, no encoder
 * interrupt, and no Wi-Fi.
 */
#ifndef IMU_CORE_H
#define IMU_CORE_H

#include "imu_types.h"

/**
 * @brief Initialise state, counters, and begin the self-test.
 *
 * The platform must have called imu_hal_bus_init() first. The first
 * imu_step() performs the bus configuration and readback check.
 */
void imu_init(uint32_t now_ms);

/** One 13 ms IMU cycle. Call at IMU_CFG_PERIOD_MS. */
void imu_step(uint32_t now_ms);

/**
 * @brief Supply the most recent encoder-derived information.
 *
 * The value is consumed once, by the next imu_step(). A missing call simply
 * integrates zero distance for that cycle and increments a diagnostic.
 */
void imu_publish_encoder_input(const imu_encoder_input_t * input);

/** Clear the retained hump peak and start a new run odometer. */
void imu_begin_run(void);

/** Copy the current two snapshot words. The IMU task is the only writer. */
bool imu_snapshot_read(imu_snapshot_t * out);

/** Pop one queued event. Returns false when the queue is empty. */
bool imu_event_pop(imu_event_t * out);

/** Copy the internal diagnostic structure. */
void imu_diag_read(imu_diag_t * out);

/** Stable names for logging. */
const char * imu_terrain_name(imu_terrain_t terrain);
const char * imu_event_name(uint8_t id);

#endif /* IMU_CORE_H */
