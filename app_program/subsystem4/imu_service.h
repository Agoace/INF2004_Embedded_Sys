/* Observer-pulled IMU boards. No driving, MQTT or KR1 interface; the
 * vehicle reads only the small hump board, never waiting. */
#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "imu_core.h"
#include "imu_probe.h"
#include "imu_capture.h"

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
/* Bounded copy of the newest hump board; same returns as the snapshot.
 * timestamp_ms is the last real IMU observation; check age before use. */
int imu_hump_try_read(imu_hump_report_t *out);
/* TEST GP21 drive boundary (single TEST writer): active=true makes the
 * run epoch odd, false makes it even. Returns the new epoch; other images
 * return 0 and use vehicle_run_epoch(). */
uint32_t imu_test_run(bool active);
/* Single operator/TEST requester. Applied by the writer on its next tick;
 * the returned sequence is acknowledged in snapshot.marks. */
uint32_t imu_request_mark(void);
/* TEST STREAM sole request/reader; disabled roles return false/no-op.
 * Never wait for capture. Frozen storage stays immutable until release;
 * starts while busy fail diagnostically, NEVER affect driving permission. */
bool imu_capture_request(void);
void imu_capture_stop_request(void);
bool imu_capture_frozen(uint32_t *samples, uint32_t *batches,
                         imu_capture_reason_t *reason);
bool imu_capture_batch_read(uint32_t index, imu_capture_batch_t *batch,
                             int16_t raw[IMU_FIFO_MAX_BATCH][3]);
bool imu_capture_release(void);

#endif
