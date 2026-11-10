#include "junction_source.h"
#include "../platform/app_board.h"

static APP_BOARD(ja_command_t) pending;
static uint32_t source_sequence;
static uint32_t acknowledged;
static uint32_t consumer_busy;

void junction_source_start(uint8_t command, uint32_t epoch, uint32_t now,
                            const int32_t counts[2])
{
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
}

bool junction_source_read(ja_command_t *out)
{
    ja_command_t copy;
    if (!out || !app_board_try_read(&pending.sequence, pending.words,
                                     &copy, sizeof(copy))) {
        return false;
    }
    *out = copy;
    return true;
}

void junction_source_acknowledge(uint32_t sequence)
{
    __atomic_store_n(&acknowledged, sequence, __ATOMIC_SEQ_CST);
}

uint32_t junction_source_acknowledged(void)
{
    return __atomic_load_n(&acknowledged, __ATOMIC_SEQ_CST);
}

void junction_source_set_busy(bool busy)
{
    __atomic_store_n(&consumer_busy, busy ? 1u : 0u, __ATOMIC_SEQ_CST);
}

bool junction_source_busy(void)
{
    return __atomic_load_n(&consumer_busy, __ATOMIC_SEQ_CST) != 0u;
}

const char *junction_source_name(uint8_t command)
{
    static const char *const names[] = {"NONE", "A", "B", "C", "D"};
    return command <= JA_UTURN ? names[command] : "INVALID";
}
