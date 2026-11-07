/* Bounded core-0 publication. A reader never waits for a preempted writer.
 * Single writer (or writers serialized by an existing owner mutex).
 * Atomic aligned words avoid C data races; loads/stores need no libatomic. */
#ifndef APP_BOARD_H
#define APP_BOARD_H
#include <stdbool.h>
#include <stdint.h>
#define APP_BOARD_WORDS(type) ((sizeof(type) + 3u) / 4u)
#define APP_BOARD(type)                                                        \
    struct {                                                                   \
        uint32_t sequence;                                                     \
        uint32_t words[APP_BOARD_WORDS(type)];                                 \
    }

static inline void app_board_publish(uint32_t *sequence, uint32_t *words,
                                     const void *value, unsigned int size)
{
    const unsigned char *bytes = value;
    const uint32_t old = __atomic_load_n(sequence, __ATOMIC_SEQ_CST);
    __atomic_store_n(sequence, old + 1u, __ATOMIC_SEQ_CST);
    for (unsigned int i = 0u; i < (size + 3u) / 4u; ++i) {
        uint32_t word = 0u;
        for (unsigned int b = 0u; b < 4u && i * 4u + b < size; ++b) {
            word |= (uint32_t)bytes[i * 4u + b] << (b * 8u);
        }
        __atomic_store_n(&words[i], word, __ATOMIC_SEQ_CST);
    }
    __atomic_store_n(sequence, old + 2u, __ATOMIC_SEQ_CST);
}
#ifndef APP_BOARD_READ_HOOK
#define APP_BOARD_READ_HOOK(sequence, index) ((void)0)
#endif
static inline bool app_board_try_read(const uint32_t *sequence,
                                      const uint32_t *words, void *value,
                                      unsigned int size)
{
    unsigned char *bytes = value;
    const uint32_t before = __atomic_load_n(sequence, __ATOMIC_SEQ_CST);
    if (before == 0u || (before & 1u) != 0u) {
        return false;
    }
    for (unsigned int i = 0u; i < (size + 3u) / 4u; ++i) {
        const uint32_t word = __atomic_load_n(&words[i], __ATOMIC_SEQ_CST);
        for (unsigned int b = 0u; b < 4u && i * 4u + b < size; ++b) {
            bytes[i * 4u + b] = (unsigned char)(word >> (b * 8u));
        }
        APP_BOARD_READ_HOOK(sequence, i);
    }
    return before == __atomic_load_n(sequence, __ATOMIC_SEQ_CST);
}

/* Failed copies are NOT values: callers use a local scratch object and commit
 * it to their cache/output only after success. Sequence wrap is permitted;
 * zero means unavailable for one publication at the 2^31-write boundary. */
#endif
