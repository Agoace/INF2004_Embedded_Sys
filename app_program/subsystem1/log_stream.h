/**
 * @file log_stream.h
 * @brief Live RAM-log stream and DUMP refill for the MQTT link.
 *
 * Two independent streams read the RAM log through the bounded
 * telemetry_log_* readers. Neither one blocks logging, driving or telemetry.
 * They run only on the Wi-Fi task. The module has no lwIP dependency: the
 * caller supplies a publish function.
 *
 * LIVE stream (QoS 0): freshness first. A cursor follows the newest rows and
 * publishes small batches. If the cursor lags, or after a reconnect, it jumps
 * to the newest rows and reports the skipped range.
 *
 * DUMP refill (QoS 1): completeness. A frozen snapshot is sent in batches,
 * with a bounded window of unacknowledged batches. The cursor advances only
 * when the broker acknowledges a batch.
 */
#ifndef LOG_STREAM_H
#define LOG_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Rows in one live batch (10 rows: 32 + 160 = 192 byte payload). */
#define LOG_LIVE_BATCH_ROWS 10u
/** A live batch with fewer rows waits at most this long, in ms. */
#define LOG_LIVE_PERIOD_MS 100u
/** A live cursor that lags more than this many rows jumps (about 1 s). */
#define LOG_LIVE_LAG_MAX_ROWS 100u
/** Rows kept (the newest ones) when the live cursor jumps. */
#define LOG_LIVE_RESUME_ROWS LOG_LIVE_BATCH_ROWS
/** Delay after a failed publish before the same stream tries again, ms. */
#define LOG_RETRY_MS 50u
/** Live batches published per poll. */
#define LOG_LIVE_SENDS_PER_POLL 1u
/** DUMP frames published per poll. */
#define LOG_DUMP_SENDS_PER_POLL 2u
/** Records in one DUMP batch. */
#define LOG_DUMP_BATCH_RECORDS 12u
/** DUMP batches in flight at once (the lwIP request table holds four). */
#define LOG_DUMP_WINDOW 3u
/** BEGIN/ABORT reason: a new run generation began during the dump. */
#define LOG_DUMP_REASON_GENERATION 1u
/** Qualities of service used by the two streams. */
#define LOG_QOS_LIVE 0u
#define LOG_QOS_DUMP 1u

/**
 * Publish one frame on the log topic.
 * @param data   Frame bytes, valid only during the call.
 * @param length Frame length in bytes.
 * @param qos    LOG_QOS_LIVE or LOG_QOS_DUMP.
 * @param token  For QoS 1, pass back to log_stream_complete(); else 0.
 * @return Zero if the frame was accepted, nonzero if it was not.
 */
typedef int (*log_stream_publish_fn)(const uint8_t *data, size_t length,
                                     uint8_t qos, uint32_t token);

/** Counters for tests and diagnostics. All wrap modulo 2^32. */
typedef struct {
    uint32_t live_batches;
    uint32_t live_rows;
    uint32_t live_skipped;
    uint32_t live_jumps;
    uint32_t live_failed;
    uint32_t dump_frames;
    uint32_t dump_rewinds;
    uint32_t dump_failed;
} log_stream_stats_t;

/** Reset both streams to their initial state. */
void log_stream_init(uint32_t boot_id);

/** Tell the streams that the MQTT link went down. */
void log_stream_link_down(void);

/**
 * Ask for one DUMP refill. Safe to call from another core-0 task.
 * @return 0 if queued, or a negative error if a dump is already pending.
 */
int log_stream_dump_request(uint32_t request_id);

/** True while a DUMP is requested or running. */
bool log_stream_dump_active(void);

/** Run both streams once. Call only on the Wi-Fi task, link up. */
void log_stream_service(log_stream_publish_fn publish);

/**
 * Report the outcome of a QoS 1 frame.
 * @param token   Value given to the publish function.
 * @param success True if the broker acknowledged the frame.
 */
void log_stream_complete(uint32_t token, bool success);

/** Copy the counters. */
void log_stream_stats(log_stream_stats_t *out);

#endif /* LOG_STREAM_H */
