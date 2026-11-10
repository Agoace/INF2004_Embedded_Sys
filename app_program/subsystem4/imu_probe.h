/* LSM303DLHC bounded probe, from Jaison's b2a6bb6 register setup. */
#ifndef IMU_PROBE_H
#define IMU_PROBE_H
#include "imu_decode.h"
#include "imu_fifo.h"

typedef struct {
    uint32_t reads;
    uint32_t faults;
    uint32_t nacks;
    uint32_t timeouts;
    uint32_t identity_errors;
    uint32_t config_errors;
    uint32_t last_us;
    uint32_t max_us;
    uint32_t total_us;
    uint32_t retry_at;
    bool ready;
    /* Last successful setup readbacks, not live register reads. */
    uint8_t ctrl1;
    uint8_t ctrl3;
    uint8_t ctrl4;
    uint8_t ctrl5;
    uint8_t fifo_ctrl;
    uint8_t fifo_status;      /* Last FIFO_SRC_REG_A value. */
    /* Ready read cycles; accepted batches/samples; min/max per accepted
     * batch; EMPTY cycles; OVRN, count>8 and contradictory status cases;
     * successful bypass->stream resets. */
    uint32_t fifo_cycles;
    uint32_t fifo_batches;
    uint32_t fifo_samples;
    uint32_t fifo_min;
    uint32_t fifo_max;
    uint32_t fifo_empty;
    uint32_t fifo_overruns;
    uint32_t fifo_backlogs;
    uint32_t fifo_bad_status;
    uint32_t fifo_recoveries;
    unsigned int fifo_last_samples;
    bool no_sample;          /* EMPTY, not an invalid/duplicate observation. */
    float mean_accel_mg[3];  /* Mapped, fractional batch mean. */
    int16_t fifo_raw[IMU_FIFO_MAX_BATCH][3]; /* Bus words, sensor axes. */
} imu_probe_t;

void imu_probe_init(imu_probe_t *probe);
/* One cycle. On error out is unchanged. No delay/retry loop; configuration
 * retries at most once a second. last_us includes configuration when due. */
bool imu_probe_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *out);
#endif
