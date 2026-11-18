#include <stddef.h>
#include "junction_source.h"
#include "junction_live.h"
#include "../platform/app_board.h"
#include "../platform/drv_time.h"

#if JUNCTION_TEST_CMD != 0
static APP_BOARD(ja_command_t) pending;
static uint32_t source_sequence;
static uint32_t acknowledged;
#else
static jlive_t live;
#endif
static uint32_t consumer_busy;

void junction_source_start(uint8_t command, uint32_t epoch, uint32_t now,
                            const int32_t counts[2])
{
#if JUNCTION_TEST_CMD != 0
    ++source_sequence;
    if (source_sequence == 0u) {
        ++source_sequence;
    }
    const ja_command_t value = {
        .epoch = epoch, .sequence = source_sequence, .read_ms = now,
        .read_counts = {counts[0], counts[1]}, .command = command,
        .source = 1u,
        .valid = (uint8_t)((epoch & 1u) != 0u && command >= JA_LEFT &&
                          command <= JA_UTURN)
    };
    app_board_publish(&pending.sequence, pending.words, &value,
                      sizeof(value));
#else
    (void)command;
    (void)counts;
    jlive_start(&live, epoch, now);
#endif
}

bool junction_source_read(ja_command_t *out)
{
#if JUNCTION_TEST_CMD != 0
    ja_command_t copy;
    if (!out || !app_board_try_read(&pending.sequence, pending.words,
                                     &copy, sizeof(copy))) {
        return false;
    }
    *out = copy;
    return true;
#else
    if (!out) {
        return false;
    }
    barcode_live_record_t copy;
    const int status = barcode_live_source_try_read(&copy);
    /* Receipt age uses the clock AFTER the coherent whole-record copy. */
    return jlive_read(&live, status == 0 ? &copy : NULL, drv_time_ms(), out);
#endif
}

void junction_source_acknowledge(uint32_t sequence)
{
#if JUNCTION_TEST_CMD != 0
    __atomic_store_n(&acknowledged, sequence, __ATOMIC_SEQ_CST);
#else
    /* The live slot was retired at offer/discard, not motor consumption.
     * Repeating a delayed consumer ACK must not undo a newer disposition. */
    (void)sequence;
#endif
}

uint32_t junction_source_acknowledged(void)
{
#if JUNCTION_TEST_CMD != 0
    return __atomic_load_n(&acknowledged, __ATOMIC_SEQ_CST);
#else
    return live.control.ack_sequence;
#endif
}

void junction_source_set_busy(bool busy)
{
    __atomic_store_n(&consumer_busy, busy ? 1u : 0u, __ATOMIC_SEQ_CST);
}

bool junction_source_busy(void)
{
    return __atomic_load_n(&consumer_busy, __ATOMIC_SEQ_CST) != 0u;
}

void junction_source_route(uint8_t module, uint32_t now)
{
#if JUNCTION_TEST_CMD == 0
    jlive_route(&live, module, now);
#else
    (void)module;
    (void)now;
#endif
}

void junction_source_tick(uint8_t module, const lp_t *line,
                            const ja_sample_t *sample,
                            const int32_t counts[2], uint32_t now,
                            ja_t *action)
{
#if JUNCTION_TEST_CMD == 0
    jlive_route(&live, module, now);
    jlive_tick(&live, line, sample, counts, now, action,
                action->phase == JA_PENDING || ja_active(action));
#else
    (void)module;
    (void)line;
    (void)sample;
    (void)counts;
    (void)now;
    (void)action;
#endif
}

void junction_source_offer_result(uint32_t sequence, bool accepted)
{
#if JUNCTION_TEST_CMD == 0
    jlive_disposition(&live, sequence, accepted);
#else
    (void)sequence;
    (void)accepted;
#endif
}

void junction_source_stop(uint32_t now)
{
#if JUNCTION_TEST_CMD == 0
    jlive_stop(&live, now);
#else
    (void)now;
#endif
}

bool junction_source_report(jlive_report_t *out)
{
#if JUNCTION_TEST_CMD == 0
    return jlive_report(&live, out);
#else
    (void)out;
    return false;
#endif
}

bool junction_source_counts(jlive_counts_t *out)
{
#if JUNCTION_TEST_CMD == 0
    return jlive_counts(&live, out);
#else
    (void)out;
    return false;
#endif
}

const char *junction_source_name(uint8_t command)
{
    static const char *const names[] = {"NONE", "A", "B", "C", "D"};
    return command <= JA_UTURN ? names[command] : "INVALID";
}
