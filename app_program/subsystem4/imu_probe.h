/* LSM303DLHC TEST probe, adapted from Jaison's b2a6bb6 register setup. */
#ifndef IMU_PROBE_H
#define IMU_PROBE_H
#include "imu_decode.h"

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
    uint8_t ctrl4;
} imu_probe_t;

void imu_probe_init(imu_probe_t *probe);
/* One cycle. On error out is unchanged. No delay/retry loop; configuration
 * retries at most once a second. last_us includes configuration when due. */
bool imu_probe_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *out);
#endif
