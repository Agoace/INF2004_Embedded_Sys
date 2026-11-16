/*
 * cpu_load.h - pure CPU load arithmetic for the idle-loop meter.
 *
 * The meter counts how often a fixed loop runs. The count rate is a peak
 * rate when the CPU does nothing else and lower when other work runs.
 *
 *     load = 1000 - 1000 * rate / peak_rate      (permille)
 *
 * Work in interrupts and in every task is counted as busy, because it takes
 * time away from the loop. No hardware access. Host-tested by
 * tests/test_cpu_load.c.
 */
#ifndef CPU_LOAD_H
#define CPU_LOAD_H

#include <stdbool.h>
#include <stdint.h>

/* Value returned when no load can be given. */
#define CPU_LOAD_UNKNOWN 65535u
/* The load scale: 1000 permille is a CPU that is always busy. */
#define CPU_LOAD_FULL 1000u

typedef struct {
    uint32_t peak_rate; /* loop iterations per second with no other work */
    bool have_last;
    uint32_t last_count;
    uint32_t last_us;
} cpu_load_t;

/*
 * Set the peak rate from a calibration: the loop ran `iterations` times in
 * `elapsed_us`. A zero input gives a meter that always returns
 * CPU_LOAD_UNKNOWN.
 */
void cpu_load_init(cpu_load_t *meter, uint32_t iterations, uint32_t elapsed_us);

/*
 * Load in permille since the previous call. The first call, a zero time step
 * or a missing calibration return CPU_LOAD_UNKNOWN. A rate above the peak
 * (measurement noise) returns 0.
 */
uint16_t cpu_load_sample(cpu_load_t *meter, uint32_t count, uint32_t now_us);

/*
 * P1 run-peak latch, owned by one writer (Health, 1 Hz). An odd run epoch
 * is a run: the first sample of a new odd epoch clears the peaks, and later
 * samples keep the maximum. An even epoch freezes the values, so the load of
 * a later dump does not count. CPU_LOAD_UNKNOWN or a value above
 * CPU_LOAD_FULL is not counted. peak[i] stays CPU_LOAD_UNKNOWN until a
 * known value is seen.
 */
typedef struct {
    uint32_t epoch;   /* odd run epoch of the values, 0 before any run */
    uint32_t samples; /* samples counted in that run */
    uint16_t peak[2]; /* core 0, core 1 */
} cpu_peak_t;

void cpu_peak_init(cpu_peak_t *p);
void cpu_peak_update(cpu_peak_t *p, uint32_t run_epoch, uint16_t load0,
                     uint16_t load1);

#endif
