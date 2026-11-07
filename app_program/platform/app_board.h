/*
 * app_board.h - bounded core-0 publication of a value (a "board").
 *
 * A reader never waits for a preempted writer. There is one writer, or
 * writers serialised by an existing owner mutex. Aligned 32-bit atomic words
 * avoid C data races; the loads and stores need no libatomic.
 *
 * The writer makes the sequence odd, writes the words, then makes it even.
 * A reader accepts a copy only if the sequence was even and unchanged.
 */
#ifndef APP_BOARD_H
#define APP_BOARD_H
#include <stdbool.h>
#include <stdint.h>

#define APP_BOARD_WORD_BYTES 4u
#define APP_BOARD_BYTE_BITS  8u
#define APP_BOARD_WORDS(type)                                                  \
    ((sizeof(type) + APP_BOARD_WORD_BYTES - 1u) / APP_BOARD_WORD_BYTES)
#define APP_BOARD(type)                                                        \
    struct {                                                                   \
        uint32_t sequence;                                                     \
        uint32_t words[APP_BOARD_WORDS(type)];                                 \
    }

/*
 * Publish size bytes of value into the board (sequence, words).
 * Writer side only. Never waits.
 */
static inline void app_board_publish(uint32_t *sequence, uint32_t *words,
                                     const void *value, unsigned int size)
{
    const unsigned char *bytes = value;
    const uint32_t old = __atomic_load_n(sequence, __ATOMIC_SEQ_CST);
    __atomic_store_n(sequence, old + 1u, __ATOMIC_SEQ_CST);
    for (unsigned int i = 0u;
         i < (size + APP_BOARD_WORD_BYTES - 1u) / APP_BOARD_WORD_BYTES; ++i) {
        uint32_t word = 0u;
        for (unsigned int b = 0u;
             b < APP_BOARD_WORD_BYTES && i * APP_BOARD_WORD_BYTES + b < size;
             ++b) {
            word |= (uint32_t)bytes[i * APP_BOARD_WORD_BYTES + b]
                    << (b * APP_BOARD_BYTE_BITS);
        }
        __atomic_store_n(&words[i], word, __ATOMIC_SEQ_CST);
    }
    __atomic_store_n(sequence, old + 2u, __ATOMIC_SEQ_CST);
}
#ifndef APP_BOARD_READ_HOOK
#define APP_BOARD_READ_HOOK(sequence, index) ((void)0)
#endif
/*
 * Copy the board into value. Returns false if no value was published yet or
 * a write was in progress. Reader side; bounded, never waits.
 */
static inline bool app_board_try_read(const uint32_t *sequence,
                                      const uint32_t *words, void *value,
                                      unsigned int size)
{
    unsigned char *bytes = value;
    const uint32_t before = __atomic_load_n(sequence, __ATOMIC_SEQ_CST);
    if (before == 0u || (before & 1u) != 0u) {
        return false;
    }
    for (unsigned int i = 0u;
         i < (size + APP_BOARD_WORD_BYTES - 1u) / APP_BOARD_WORD_BYTES; ++i) {
        const uint32_t word = __atomic_load_n(&words[i], __ATOMIC_SEQ_CST);
        for (unsigned int b = 0u;
             b < APP_BOARD_WORD_BYTES && i * APP_BOARD_WORD_BYTES + b < size;
             ++b) {
            bytes[i * APP_BOARD_WORD_BYTES + b] =
                (unsigned char)(word >> (b * APP_BOARD_BYTE_BITS));
        }
        APP_BOARD_READ_HOOK(sequence, i);
    }
    return before == __atomic_load_n(sequence, __ATOMIC_SEQ_CST);
}

/* Failed copies are NOT values: callers use a local scratch object and commit
 * it to their cache/output only after success. Sequence wrap is permitted;
 * zero means unavailable for one publication at the 2^31-write boundary. */
#endif
