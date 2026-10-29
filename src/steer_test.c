/*
 * Bench test: follow the line with proportional steering (src/line_steer.c).
 *
 *   GP20  spin calibration (src/line_spin.c)
 *   GP21  start line following (after 1 s) / stop
 *
 * While following, the wheel encoders give the line position module a side
 * hint: if the left wheel has turned more than the right since the line
 * was last centred, the robot turns right and the line moves to the left.
 *
 * On LINE_LOST the robot searches on the spot: it turns toward the side
 * where the line was last seen (left if unknown) by SEARCH_COUNTS, then to
 * the other side. LINE_FOUND resumes line following.
 *
 * A run stops on: GP21, a failed search, or a stalled wheel. There is no
 * time limit. The log keeps the last LOG_KEEP_MS of the run.
 *
 * Power: a driving test. Use 5 V VIN with USB disconnected
 * (USB_POWER_SAFETY.md). The run is logged every 50 ms and saved to the
 * last two flash sectors when the robot stops, so it survives a restart.
 * Connect USB at any time after the run and open the serial port: the last
 * saved run prints once, then the summary every 5 s.
 *
 * Wi-Fi log (src/wifi_log.c, docs/STEER_WIFI_LOG.md): with
 * src/wifi_config.h, the robot joins the network in the background (LED on
 * when joined) and broadcasts the printf output and the run log live, 5
 * entries every 250 ms. When a run stops, and when Wi-Fi joins with a
 * saved run, the whole kept log is sent again (replay), so the receiver
 * can fill in lost packets.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "drive.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "line_position.h"
#include "line_sensors.h"
#include "line_spin.h"
#include "line_steer.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "steer_log.h"
#include "wifi_log.h"

#define SPIN_BUTTON_PIN 20
#define RUN_BUTTON_PIN 21

/* Steering settings for this test. */
static const steer_config_t STEER = {
    .base_pct = 20u,  /* both wheels when centred (operator, 2026-10-27) */
    .gain_pct = 200u, /* inner wheel stops at position +/-50 (run 6) */
    .kd_pct = 0u,     /* D off; add it if the robot zig-zags */
    .min_pct = 12u,   /* a loaded wheel starts to turn here: speed zero */
    .max_pct = 50u,   /* operator limit, 2026-10-29 (was 30) */
    .right_trim_pct = 1, /* team drive-straight trim: right motor +1 % */
};

#define SAMPLE_PERIOD_US 1000u
#define STEER_PERIOD_MS 10u
#define LOG_PERIOD_MS 50u
#define LOG_KEEP_MS 20000u /* the log keeps the last 20 s */
#define START_DELAY_MS 1000u
#define STALL_GRACE_MS 1000u
#define STALL_TIMEOUT_MS 500u
#define BUTTON_POLL_MS 20u
#define REPORT_REPEAT_MS 5000u
#define IDLE_PRINT_MS 300u
#define LOG_ENTRIES (LOG_KEEP_MS / LOG_PERIOD_MS)

/* Wheel hint: left minus right counts since the line was last centred. */
#define HINT_COUNTS 20
/* Search: on-the-spot turn, an eighth of a wheel turn to each side. */
#define SEARCH_DUTY_PCT 20
#define SEARCH_COUNTS (DRIVE_COUNTS_PER_WHEEL_TURN / 8)

typedef enum {
    STOP_BUTTON,
    STOP_SEARCH_FAILED,
    STOP_STALL_LEFT,
    STOP_STALL_RIGHT,
} stop_reason_t;

static const char *const STOP_NAMES[] = {
    "GP21 pressed", "search failed", "left wheel stalled", "right wheel stalled",
};

static const char *side_name(unsigned int side) {
    return side == VC_LINE_SIDE_LEFT ? "L" : side == VC_LINE_SIDE_RIGHT ? "R" : "-";
}

/* Log entry info byte: bits 0-1 mode, 2-3 fade side, 4-5 side source,
 * 6-7 wheel hint. */
#define MODE_FOLLOW 0u
#define MODE_SEARCH 1u

typedef sl_entry_t log_entry_t; /* counts are per 50 ms */

#define LIVE_BATCH 5u /* entries per live packet: one packet every 250 ms */

/* |position| buckets for the summary. */
#define BUCKETS 5u
static const char *const BUCKET_NAMES[BUCKETS] = {"0", "1-20", "21-50", "51-99", "100"};

static unsigned int bucket_of(int position) {
    const int a = abs(position);
    return a == 0 ? 0u : a <= 20 ? 1u : a <= 50 ? 2u : a < 100 ? 3u : 4u;
}

#define RUN_MAGIC 0x53545234u /* "STR4": a saved run is in flash */

typedef struct {
    uint32_t magic;
    bool valid;
    stop_reason_t reason;
    uint32_t duration_ms;
    int32_t left_counts;
    int32_t right_counts;
    uint32_t steer_updates;
    uint32_t sum_abs_position;
    uint32_t bucket[BUCKETS];
    uint32_t source_updates[4]; /* steer updates per lp_side_source_t */
    uint16_t searches;
    uint16_t recoveries;
    uint16_t floor[LP_SENSORS];
    uint16_t line[LP_SENSORS];
    steer_config_t steer; /* settings used in this run */
    uint32_t log_total; /* entries written; the log is a ring of the last LOG_ENTRIES */
    log_entry_t log[LOG_ENTRIES];
} run_t;

static run_t run;
static lp_t lp;
static uint8_t run_number; /* 0: a saved run from before the restart */

/* Wi-Fi: send log entries [first, first + count) of the current run. */
static void send_entries(uint32_t first, uint32_t count, uint8_t flags) {
    uint8_t packet[SL_LOG_HEADER_BYTES + SL_MAX_ENTRIES * SL_ENTRY_BYTES];
    if (count > SL_MAX_ENTRIES) count = SL_MAX_ENTRIES;
    size_t n = sl_write_log_header(packet, run_number, flags, (uint16_t)first, (uint8_t)count);
    for (uint32_t i = 0u; i < count; ++i) {
        sl_pack_entry(&run.log[(first + i) % LOG_ENTRIES], &packet[n]);
        n += SL_ENTRY_BYTES;
    }
    (void)wifi_log_send(SL_TYPE_LOG, packet, n);
}

/* Replay of the kept log, one packet per call from the idle loop. */
static struct {
    bool active;
    uint32_t next;
} replay;

static void replay_start(void) {
    const uint32_t kept = run.log_total < LOG_ENTRIES ? run.log_total : LOG_ENTRIES;
    replay.next = run.log_total - kept;
    replay.active = kept > 0u;
}

static void replay_step(void) {
    if (!replay.active || !wifi_log_receiver_known()) {
        return;
    }
    const uint32_t left = run.log_total - replay.next;
    const uint32_t count = left < SL_MAX_ENTRIES ? left : SL_MAX_ENTRIES;
    const bool last = count == left;
    send_entries(replay.next, count,
                 (uint8_t)(SL_FLAG_REPLAY | (last ? SL_FLAG_REPLAY_END : 0u)));
    replay.next += count;
    replay.active = !last;
}

/* The last two flash sectors hold the last run. The program is far smaller. */
#define RUN_FLASH_SECTORS 2u
#define RUN_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - RUN_FLASH_SECTORS * FLASH_SECTOR_SIZE)
#define RUN_FLASH_BYTES \
    (((sizeof(run_t) + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)
_Static_assert(RUN_FLASH_BYTES <= RUN_FLASH_SECTORS * FLASH_SECTOR_SIZE,
               "run log must fit the reserved flash sectors");

static void save_run(void) {
    static uint8_t buffer[RUN_FLASH_BYTES];
    for (unsigned int i = 0u; i < RUN_FLASH_BYTES; ++i) buffer[i] = 0xFFu;
    run.magic = RUN_MAGIC;
    const uint8_t *src = (const uint8_t *)&run;
    for (unsigned int i = 0u; i < sizeof(run_t); ++i) buffer[i] = src[i];
    const uint32_t saved = save_and_disable_interrupts();
    flash_range_erase(RUN_FLASH_OFFSET, RUN_FLASH_SECTORS * FLASH_SECTOR_SIZE);
    flash_range_program(RUN_FLASH_OFFSET, buffer, RUN_FLASH_BYTES);
    restore_interrupts(saved);
}

static void load_run(void) {
    const run_t *stored = (const run_t *)(XIP_BASE + RUN_FLASH_OFFSET);
    if (stored->magic == RUN_MAGIC && stored->valid) {
        run = *stored;
    }
}

static bool button_pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

static void print_summary(void) {
    printf("[run] %lu ms, stopped by %s\n", (unsigned long)run.duration_ms,
           STOP_NAMES[run.reason]);
    printf("[run] wheels L %ld  R %ld counts (%.2f / %.2f wheel turns)\n",
           (long)run.left_counts, (long)run.right_counts,
           (double)run.left_counts / DRIVE_COUNTS_PER_WHEEL_TURN,
           (double)run.right_counts / DRIVE_COUNTS_PER_WHEEL_TURN);
    printf("[run] LINE_LOST searches %u, line found again %u\n", run.searches,
           run.recoveries);
    printf("[run] calibration (floor/line): L %u/%u  C %u/%u  R %u/%u\n", run.floor[0],
           run.line[0], run.floor[1], run.line[1], run.floor[2], run.line[2]);
    printf("[run] steering: base %u%% gain %u%% kd %u%% min %u%% (speed zero) max %u%% "
           "right trim %+d%%\n",
           run.steer.base_pct, run.steer.gain_pct, run.steer.kd_pct, run.steer.min_pct,
           run.steer.max_pct, run.steer.right_trim_pct);
    if (run.steer_updates > 0u) {
        printf("[run] mean |pos| %lu;  |pos| share:",
               (unsigned long)(run.sum_abs_position / run.steer_updates));
        for (unsigned int b = 0u; b < BUCKETS; ++b) {
            printf("  %s: %lu%%", BUCKET_NAMES[b],
                   (unsigned long)(run.bucket[b] * 100u / run.steer_updates));
        }
        printf("\n[run] side known from: sensor %lu%%  gap rise %lu%%  wheel hint %lu%%\n",
               (unsigned long)(run.source_updates[LP_SIDE_SOURCE_SENSOR] * 100u /
                               run.steer_updates),
               (unsigned long)(run.source_updates[LP_SIDE_SOURCE_GAP] * 100u /
                               run.steer_updates),
               (unsigned long)(run.source_updates[LP_SIDE_SOURCE_HINT] * 100u /
                               run.steer_updates));
    }
}

static void print_log(void) {
    printf("\n[log] t_ms mode  pos  dutyL dutyR  rawL rawC rawR  side src hint  cntL cntR\n");
    static const char *const SOURCE[4] = {"-", "sensor", "gap", "hint"};
    const uint32_t kept = run.log_total < LOG_ENTRIES ? run.log_total : LOG_ENTRIES;
    if (kept < run.log_total) {
        printf("[log] (only the last %u s are kept)\n", LOG_KEEP_MS / 1000u);
    }
    for (uint32_t i = run.log_total - kept; i < run.log_total; ++i) {
        const log_entry_t *e = &run.log[i % LOG_ENTRIES];
        printf("[log] %6lu %s %+4d  %3d %3d  %4u %4u %4u  %s %-6s %s  %4d %4d\n",
               (unsigned long)(i * LOG_PERIOD_MS), (e->info & 3u) == MODE_SEARCH ? "SRCH" : "FOLW",
               e->position, e->left_pct, e->right_pct, e->raw[0], e->raw[1], e->raw[2],
               side_name((e->info >> 2) & 3u), SOURCE[(e->info >> 4) & 3u],
               side_name((e->info >> 6) & 3u), e->left_counts, e->right_counts);
    }
}

typedef struct {
    bool active;
    int direction;     /* +1 turn left first, -1 turn right first */
    unsigned int step; /* 0: first side, 1: other side */
    int32_t start_l;
    int32_t start_r;
} search_t;

/* Robot turn since the search start, in wheel counts: positive = left. */
static int32_t search_angle(const search_t *s, const int32_t c[2]) {
    return ((c[1] - s->start_r) - (c[0] - s->start_l)) / 2;
}

static void follow_line(steer_t *steer, bool *previous_run) {
    run = (run_t){0};
    run.steer = STEER;
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        run.floor[s] = lp_floor(&lp, s);
        run.line[s] = lp_line(&lp, s);
    }
    steer_reset(steer);
    replay.active = false;
    ++run_number;
    if (run_number == 0u) run_number = 1u;
    printf("[run] start, run %u\n", run_number);

    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    int32_t start[2];
    drive_counts(&start[0], &start[1]);
    int32_t last_log[2] = {start[0], start[1]};
    int32_t last_steer[2] = {start[0], start[1]};
    int32_t last_count[2] = {start[0], start[1]};
    uint32_t moved_ms[2] = {t0, t0};
    int duty[2] = {0, 0};
    int32_t heading = 0; /* left minus right counts since the line was centred */
    uint16_t raw[LP_SENSORS] = {0u, 0u, 0u};
    search_t search = {0};
    absolute_time_t next_sample = get_absolute_time();
    uint32_t next_steer_ms = t0;
    uint32_t next_log_ms = t0 + LOG_PERIOD_MS;
    uint32_t next_button_ms = t0;
    stop_reason_t reason = STOP_BUTTON;

    while (true) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        int32_t c[2];
        drive_counts(&c[0], &c[1]);
        wifi_log_poll(now);

        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            line_sensors_read(raw);
            const vc_event_t event = lp_update(&lp, raw, now);
            if (vc_event_type(event) == VC_EVENT_LINE_LOST && !search.active) {
                const vc_line_side_t side = (vc_line_side_t)vc_event_payload(event);
                search = (search_t){
                    .active = true,
                    .direction = side == VC_LINE_SIDE_RIGHT ? -1 : +1,
                    .step = 0u,
                    .start_l = c[0],
                    .start_r = c[1],
                };
                ++run.searches;
                duty[0] = -search.direction * SEARCH_DUTY_PCT;
                duty[1] = search.direction * SEARCH_DUTY_PCT;
                drive_set(duty[0], duty[1]);
            } else if (vc_event_type(event) == VC_EVENT_LINE_FOUND && search.active) {
                search.active = false;
                ++run.recoveries;
                steer_reset(steer);
                heading = 0;
                next_steer_ms = now; /* steer at once */
            }
        }

        if (search.active) {
            const int32_t angle = search_angle(&search, c) * search.direction;
            if (search.step == 0u && angle >= SEARCH_COUNTS) {
                search.step = 1u; /* now the other side */
                duty[0] = search.direction * SEARCH_DUTY_PCT;
                duty[1] = -search.direction * SEARCH_DUTY_PCT;
                drive_set(duty[0], duty[1]);
            } else if (search.step == 1u && angle <= -SEARCH_COUNTS) {
                reason = STOP_SEARCH_FAILED;
                break;
            }
        } else if ((int32_t)(now - next_steer_ms) >= 0) {
            next_steer_ms = now + STEER_PERIOD_MS;
            heading += (c[0] - last_steer[0]) - (c[1] - last_steer[1]);
            last_steer[0] = c[0];
            last_steer[1] = c[1];
            if (lp_centred(&lp)) {
                heading = 0;
            }
            /* Robot turning right (left wheel ahead): line moves to the left. */
            lp_set_hint(&lp, heading > HINT_COUNTS    ? VC_LINE_SIDE_LEFT
                             : heading < -HINT_COUNTS ? VC_LINE_SIDE_RIGHT
                                                      : VC_LINE_SIDE_UNKNOWN);
            const steer_word_t w = steer_update(steer, lp.position);
            duty[0] = (int)steer_left(w);
            duty[1] = (int)steer_right(w);
            drive_set(duty[0], duty[1]);
            ++run.steer_updates;
            run.sum_abs_position += (uint32_t)abs(lp.position);
            ++run.bucket[bucket_of(lp.position)];
            ++run.source_updates[lp.fade_source & 3u];
        }
        if (search.active) {
            /* The search turn is not drift: do not count it in the heading. */
            last_steer[0] = c[0];
            last_steer[1] = c[1];
        }

        bool stalled[2];
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (c[w] != last_count[w] || duty[w] == 0) {
                last_count[w] = c[w];
                moved_ms[w] = now;
            }
            stalled[w] = now - t0 >= STALL_GRACE_MS && now - moved_ms[w] >= STALL_TIMEOUT_MS;
        }
        if (stalled[0] || stalled[1]) {
            reason = stalled[0] ? STOP_STALL_LEFT : STOP_STALL_RIGHT;
            break;
        }

        if ((int32_t)(now - next_log_ms) >= 0) {
            next_log_ms += LOG_PERIOD_MS;
            {
                run.log[run.log_total++ % LOG_ENTRIES] = (log_entry_t){
                    .position = lp.position,
                    .left_pct = (int8_t)duty[0],
                    .right_pct = (int8_t)duty[1],
                    .info = (uint8_t)((search.active ? MODE_SEARCH : MODE_FOLLOW) |
                                      ((lp.fade_side & 3u) << 2) |
                                      ((lp.fade_source & 3u) << 4) |
                                      ((lp.hint & 3u) << 6)),
                    .raw = {raw[0], raw[1], raw[2]},
                    .left_counts = (int16_t)(c[0] - last_log[0]),
                    .right_counts = (int16_t)(c[1] - last_log[1]),
                };
                if (run.log_total % LIVE_BATCH == 0u) {
                    send_entries(run.log_total - LIVE_BATCH, LIVE_BATCH, 0u);
                }
            }
            last_log[0] = c[0];
            last_log[1] = c[1];
        }

        if ((int32_t)(now - next_button_ms) >= 0) {
            next_button_ms = now + BUTTON_POLL_MS;
            if (button_pressed(RUN_BUTTON_PIN, previous_run)) {
                reason = STOP_BUTTON;
                break;
            }
        }
    }

    drive_stop();
    const uint32_t end = to_ms_since_boot(get_absolute_time());
    int32_t c[2];
    drive_counts(&c[0], &c[1]);
    run.reason = reason;
    run.duration_ms = end - t0;
    run.left_counts = c[0] - start[0];
    run.right_counts = c[1] - start[1];
    run.valid = true;
    save_run();
    print_summary();
    replay_start();
}

int main(void) {
    stdio_init_all();
    wifi_log_init();
    line_sensors_init();
    drive_init();
    drive_stop();
    const unsigned int buttons[2] = {SPIN_BUTTON_PIN, RUN_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(buttons[i]);
        gpio_set_dir(buttons[i], GPIO_IN);
        gpio_pull_up(buttons[i]);
    }
    sleep_ms(2000);
    printf("\nLine steering test. GP20: spin calibration. GP21: start/stop.\n");

    load_run();
    if (run.valid) {
        printf("[run] A saved run is in flash; it prints when a terminal connects.\n");
    }
    lp_init(&lp);
    steer_t steer;
    steer_init(&steer, &STEER);

    bool previous_spin = false;
    bool previous_run = false;
    bool was_connected = false;
    bool receiver_was_known = false;
    uint32_t next_report_ms = 0u;
    uint32_t next_idle_print_ms = 0u;

    while (true) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        wifi_log_poll(now);
        replay_step();

        /* Receiver found with a saved run: send its summary and log. */
        const bool wifi_up = wifi_log_up();
        const bool receiver = wifi_log_receiver_known();
        if (receiver && !receiver_was_known && run.valid) {
            print_summary();
            replay_start();
        }
        receiver_was_known = receiver;

        if (button_pressed(SPIN_BUTTON_PIN, &previous_spin)) {
            (void)line_spin_calibrate(&lp);
            printf("\n");
        }

        if (button_pressed(RUN_BUTTON_PIN, &previous_run)) {
            if (!lp_calibrated(&lp)) {
                printf("[run] Not calibrated. Press GP20 first.\n");
            } else {
                sleep_ms(START_DELAY_MS);
                follow_line(&steer, &previous_run);
                next_report_ms = to_ms_since_boot(get_absolute_time()) + REPORT_REPEAT_MS;
            }
        }

        /* Print the log once when a terminal connects after a run. */
        const bool connected = stdio_usb_connected();
        if (connected && !was_connected && run.valid) {
            sleep_ms(500);
            print_summary();
            wifi_log_mirror_text(false); /* the Wi-Fi log has its own packets */
            print_log();
            wifi_log_mirror_text(true);
            next_report_ms = now + REPORT_REPEAT_MS;
        }
        was_connected = connected;
        if (connected && run.valid && (int32_t)(now - next_report_ms) >= 0) {
            next_report_ms = now + REPORT_REPEAT_MS;
            print_summary();
        }

        /* While idle and calibrated, show the line state (no motors). */
        uint16_t raw[LP_SENSORS];
        line_sensors_read(raw);
        (void)lp_update(&lp, raw, now);
        if ((connected || wifi_up) && lp_calibrated(&lp) &&
            (int32_t)(now - next_idle_print_ms) >= 0) {
            next_idle_print_ms = now + IDLE_PRINT_MS;
            printf("[idle] raw %4u %4u %4u  n %4u %4u %4u  pos %+4d  side %s\n", raw[0],
                   raw[1], raw[2], lp.norm[0], lp.norm[1], lp.norm[2], lp.position,
                   side_name(lp.fade_side));
        }

        sleep_ms(BUTTON_POLL_MS);
    }
}
