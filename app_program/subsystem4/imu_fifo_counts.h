/* IMU accel FIFO counters for the ROBOT GP20 dump. Small on purpose: the
 * reader copies 6 words, not the whole IMU snapshot. Each word is written
 * only by the IMU task and read with an atomic word load, so each field is
 * exact but the set is not one coherent copy (diagnostic only). */
#ifndef IMU_FIFO_COUNTS_H
#define IMU_FIFO_COUNTS_H
#include <stdint.h>

typedef struct {
    uint32_t partial;    /* Option D: 9..31 queued, oldest 8 read. */
    uint32_t depth_max;  /* Largest FIFO count seen (32 = OVRN). */
    uint32_t overruns;   /* OVRN: FIFO reset, READ_FAILED. */
    uint32_t bad_status; /* Contradictory status: FIFO reset. */
    uint32_t recoveries; /* Successful FIFO resets. */
    uint32_t batches;    /* Valid observations. */
} imu_fifo_counts_t;

/* E_PAR for NULL; E_OBJ before the IMU service runs. */
int imu_fifo_counts_read(imu_fifo_counts_t *out);
#endif
