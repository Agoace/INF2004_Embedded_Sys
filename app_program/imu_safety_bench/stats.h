/* Bench-only observation. All times are wall time, not exclusive CPU. */
#ifndef IMU_SAFETY_BENCH_STATS_H
#define IMU_SAFETY_BENCH_STATS_H
#include "replay.h"
#include "../subsystem4/imu_service.h"

/* Timed sections inside the service compute window. REST is compute
 * minus the three timed sections: capture, publication, clear handling
 * and timer reads. Preemption lands in whichever section was running. */
typedef enum {
    IMU_BENCH_CORE = 0,   /* imu_core_step, including a replay. */
    IMU_BENCH_DIAG = 1,   /* imu_hump_diag_step. */
    IMU_BENCH_SAFETY = 2, /* imu_safety_step (the detector). */
    IMU_BENCH_REST = 3,
    IMU_BENCH_SECTIONS = 4
} imu_bench_section_t;

typedef struct {
    uint64_t total_us;
    uint32_t max_us;
    uint32_t min_us;
} imu_bench_split_t;

typedef struct {
    uint64_t compute_total_us;
    uint64_t work_total_us;
    uint32_t samples;
    uint32_t work_samples;
    uint32_t compute_max_us;
    uint32_t compute_min_us;
    uint32_t work_max_us;
    uint32_t overruns;
    uint32_t triggers; /* bit (1 << imu_safety_trigger_t) observed */
    imu_bench_split_t split[IMU_BENCH_SECTIONS];
} imu_bench_phase_t;
typedef struct {
    imu_bench_phase_t phase[IMU_REPLAY_PHASES];
    uint32_t episodes; /* ROBOT stream episodes scheduled. */
    bool robot;
} imu_bench_stats_t;

/* Operator hint for the ROBOT run. Pure; the reporter prints it. */
#define IMU_BENCH_IDLE_MS 70000u
typedef enum {
    IMU_BENCH_HINT_IDLE_WINDOW = 0,   /* Do not press; idle load window. */
    IMU_BENCH_HINT_PRESS_START = 1,   /* Press GP21 once to start. */
    IMU_BENCH_HINT_WAIT = 2,          /* Vehicle is busy. */
    IMU_BENCH_HINT_WAIT_CLEAR = 3,    /* Latched; pose clear window runs. */
    IMU_BENCH_HINT_PRESS_RECOVER = 4  /* Press GP21 once to clear. */
} imu_bench_hint_t;
imu_bench_hint_t imu_bench_hint(uint8_t latched, uint32_t clear_ms,
                                bool vehicle_active, uint32_t uptime_ms);
const char *imu_bench_hint_text(imu_bench_hint_t hint);

void imu_bench_probe_init(imu_probe_t *probe);
bool imu_bench_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *raw);
void imu_bench_record(const imu_snapshot_t *snapshot);
bool imu_bench_stats_try_read(imu_bench_stats_t *out);
void imu_bench_core_step(imu_core_t *core, const imu_core_input_t *input);
void imu_bench_hump_diag_step(imu_hump_diag_t *s,
                              const imu_hump_diag_input_t *in,
                              imu_hump_diag_report_t *out);
void imu_bench_safety_step(imu_safety_t *s, const imu_safety_input_t *in);
void imu_safety_bench_run(void);
#endif
