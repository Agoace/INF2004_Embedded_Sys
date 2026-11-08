/* Observer-pulled IMU board. No driving, MQTT or KR1 interface. */
#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "imu_core.h"
#include "imu_probe.h"

typedef struct {
    imu_core_output_t value;
    imu_units_t raw;
    imu_probe_t probe;
    uint32_t timestamp_us;
    uint32_t timestamp_ms;
    uint32_t marks;
    uint32_t gap_max_us;
    uint32_t missed_wakes;
    uint32_t wake_lost;
    uint32_t work_last_us;
    uint32_t work_max_us;
    uint32_t work_total_us;
    uint32_t compute_last_us;
    uint32_t compute_max_us;
    uint32_t compute_total_us;
} imu_snapshot_t;

int imu_service_init(void);
int imu_service_start(void);
/* One bounded board copy. E_PAR null, E_OBJ inactive, E_TMOUT busy/empty.
 * Failed reads leave the caller's output unchanged. */
int imu_snapshot_try_read(imu_snapshot_t *out);
/* Single operator/TEST requester. Applied by the writer on its next tick;
 * the returned sequence is acknowledged in snapshot.marks. */
uint32_t imu_request_mark(void);

#endif
