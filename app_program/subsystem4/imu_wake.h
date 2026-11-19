/* IMU INT1 watermark wake policy. Pure: no SDK, no kernel calls.
 *
 * Wake path (IMU_INT1_WAKE=1, UP images only):
 *   accel FIFO count reaches the watermark -> INT1 (GP7) goes HIGH ->
 *   core-0 GPIO dispatcher client -> imu_wake_edge() -> tk_wup_tsk(IMU).
 *   The task reads the FIFO exactly as before; the burst drains it and
 *   INT1 goes LOW. One RISING edge per batch. No I2C in interrupt context.
 * Fallback: the existing 10 ms cyclic stays. imu_wake_tick() wakes the
 *   task only if no rising edge came for more than IMU_WAKE_FALLBACK_US
 *   AND the last FIFO read is also older than that. (A read drains the
 *   FIFO, so the next edge is due <= 4 samples, ~10.2 ms, after it; a
 *   late task thus never causes a false fallback.) The first fallback
 *   enters POLL mode: every tick wakes, exactly today's 10 ms polling,
 *   until a rising edge proves INT1 alive again. POLL is also the start
 *   state. A dead INT1 thus gives 10 ms polling after <= 22 ms + latency.
 * Missed edge: after a sample, a HIGH INT1 level gives ONE extra pass.
 *
 * Writers: rises/falls/last_edge_us/isr_lost = GPIO ISR only; ticks/fallbacks =
 * cyclic handler only; last_read_us/extra/batches = IMU task only.
 * poll: the ISR stores 0, the cyclic stores 1 (a race costs one poll).
 * Each field is one aligned 32-bit word with atomic access. A reader gets
 * per-field values, not one coherent copy (diagnostic counters only). */
#ifndef IMU_WAKE_H
#define IMU_WAKE_H
#include <stdbool.h>
#include <stdint.h>

#ifndef IMU_INT1_WAKE
#define IMU_INT1_WAKE 0
#endif
#define IMU_INT1_PIN 7u
#define IMU_WAKE_EDGE_FALL 4u /* Same bits as DRV_GPIO_EDGE_FALL/RISE. */
#define IMU_WAKE_EDGE_RISE 8u
/* Above the ~10.2 ms watermark period after a read (measured ODR ~391 Hz, 4 samples);
 * below the 30 ms safety clear-gap and the 50 ms stale-data rules. */
#define IMU_WAKE_FALLBACK_US 12000u
/* FIFO_CTRL_REG_A FTH. Datasheet: WTM is set when the FIFO content
 * EXCEEDS the watermark. Not hardware-verified: with FTH=3, ">" gives
 * n=4 (~10.2 ms) and ">=" gives n=3 (~7.7 ms). Both stay below the 12 ms
 * fallback. FTH=4 with ">" would give n=5 (~12.8 ms) and false
 * fallbacks, so 3 is the safe default. The R1 n histogram decides. */
#ifndef IMU_WAKE_FTH
#define IMU_WAKE_FTH 3u
#endif
#define IMU_WAKE_CTRL3 0x04u /* CTRL_REG3_A I1_WTM only. */
#define IMU_WAKE_FIFO_CTRL (0x80u | IMU_WAKE_FTH) /* Stream mode + FTH. */
#define IMU_WAKE_HISTOGRAM 9u /* Batch n = 0..8 (n>8 is a backlog reset). */

typedef struct {
    uint32_t rises;
    uint32_t falls;
    uint32_t last_edge_us;
    uint32_t isr_lost;
    uint32_t poll;
    uint32_t last_read_us;
    uint32_t ticks;
    uint32_t fallbacks;
    uint32_t extra;
    uint32_t batches[IMU_WAKE_HISTOGRAM];
} imu_wake_t;

static inline uint32_t imu_wake_load(const uint32_t *word)
{
    return __atomic_load_n(word, __ATOMIC_RELAXED);
}

static inline void imu_wake_add(uint32_t *word)
{
    __atomic_store_n(word, imu_wake_load(word) + 1u, __ATOMIC_RELAXED);
}

/* GPIO ISR. Returns true when the IMU task must be woken. */
static inline bool imu_wake_edge(imu_wake_t *w, uint32_t events,
                                 uint32_t now_us)
{
    if ((events & IMU_WAKE_EDGE_FALL) != 0u) {
        imu_wake_add(&w->falls);
    }
    if ((events & IMU_WAKE_EDGE_RISE) == 0u) {
        return false;
    }
    __atomic_store_n(&w->last_edge_us, now_us, __ATOMIC_RELAXED);
    __atomic_store_n(&w->poll, 0u, __ATOMIC_RELAXED);
    imu_wake_add(&w->rises);
    return true;
}

/* Start state: POLL (today's cadence) until the first rising edge. */
static inline void imu_wake_init(imu_wake_t *w)
{
    *w = (imu_wake_t){0};
    w->poll = 1u;
}

/* IMU task, immediately before each FIFO read. */
static inline void imu_wake_read(imu_wake_t *w, uint32_t now_us)
{
    __atomic_store_n(&w->last_read_us, now_us, __ATOMIC_RELAXED);
}

/* Cyclic handler. Returns true for a fallback wake. An edge that
 * interrupts this check can only cause one harmless extra poll. */
static inline bool imu_wake_tick(imu_wake_t *w, uint32_t now_us)
{
    imu_wake_add(&w->ticks);
    if (imu_wake_load(&w->poll) == 0u &&
        (now_us - imu_wake_load(&w->last_edge_us) <= IMU_WAKE_FALLBACK_US ||
         now_us - imu_wake_load(&w->last_read_us) <= IMU_WAKE_FALLBACK_US)) {
        return false;
    }
    __atomic_store_n(&w->poll, 1u, __ATOMIC_RELAXED);
    imu_wake_add(&w->fallbacks);
    return true;
}

/* IMU task after a sample: record n (0 = empty; pass IMU_WAKE_HISTOGRAM
 * for an invalid read); return true for ONE extra pass. */
static inline bool imu_wake_after(imu_wake_t *w, unsigned int samples,
                                  bool ready, bool int1_high,
                                  bool already_extra)
{
    if (samples < IMU_WAKE_HISTOGRAM) {
        imu_wake_add(&w->batches[samples]);
    }
    if (!ready || !int1_high || already_extra) {
        return false;
    }
    imu_wake_add(&w->extra);
    return true;
}

static inline void imu_wake_copy(imu_wake_t *out, const imu_wake_t *w)
{
    out->rises = imu_wake_load(&w->rises);
    out->falls = imu_wake_load(&w->falls);
    out->last_edge_us = imu_wake_load(&w->last_edge_us);
    out->isr_lost = imu_wake_load(&w->isr_lost);
    out->poll = imu_wake_load(&w->poll);
    out->last_read_us = imu_wake_load(&w->last_read_us);
    out->ticks = imu_wake_load(&w->ticks);
    out->fallbacks = imu_wake_load(&w->fallbacks);
    out->extra = imu_wake_load(&w->extra);
    for (unsigned int i = 0u; i < IMU_WAKE_HISTOGRAM; ++i) {
        out->batches[i] = imu_wake_load(&w->batches[i]);
    }
}
#endif
