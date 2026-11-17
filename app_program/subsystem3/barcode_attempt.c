#include "barcode_attempt.h"
#if TM_MQTT && !APP_TEST && !APP_BENCH
/* Single producer/consumer; payload ownership transfers with release/acquire
 * indices. A consumer never advances tail before PUBACK. No overwrite races. */
static rm_barcode_t rows[BC_ATTEMPT_CAPACITY];
static uint32_t head, tail, dropped;
_Static_assert(sizeof(rows) + sizeof(head) + sizeof(tail) + sizeof(dropped) <= 240u,
               "Attempt FIFO RAM budget");
bool barcode_attempt_enqueue(const rm_barcode_t *value)
{
    if (!value) return false;
    const uint32_t h = __atomic_load_n(&head, __ATOMIC_RELAXED);
    const uint32_t t = __atomic_load_n(&tail, __ATOMIC_ACQUIRE);
    if (h - t >= BC_ATTEMPT_CAPACITY) {
        __atomic_store_n(&dropped,
            __atomic_load_n(&dropped, __ATOMIC_RELAXED) + 1u, __ATOMIC_RELEASE);
        return false;
    }
    rows[h % BC_ATTEMPT_CAPACITY] = *value;
    __atomic_store_n(&head, h + 1u, __ATOMIC_RELEASE);
    return true;
}
bool barcode_attempt_peek(rm_barcode_t *out)
{
    if (!out) return false;
    const uint32_t t = __atomic_load_n(&tail, __ATOMIC_RELAXED);
    if (t == __atomic_load_n(&head, __ATOMIC_ACQUIRE)) return false;
    *out = rows[t % BC_ATTEMPT_CAPACITY];
    return true;
}
bool barcode_attempt_ack(uint32_t sequence)
{
    const uint32_t t = __atomic_load_n(&tail, __ATOMIC_RELAXED);
    if (t == __atomic_load_n(&head, __ATOMIC_ACQUIRE) ||
        rows[t % BC_ATTEMPT_CAPACITY].sequence != sequence) return false;
    __atomic_store_n(&tail, t + 1u, __ATOMIC_RELEASE);
    return true;
}
uint32_t barcode_attempt_dropped(void)
{
    return __atomic_load_n(&dropped, __ATOMIC_ACQUIRE);
}
#else
bool barcode_attempt_enqueue(const rm_barcode_t *value) { (void)value; return false; }
bool barcode_attempt_peek(rm_barcode_t *out) { (void)out; return false; }
bool barcode_attempt_ack(uint32_t sequence) { (void)sequence; return false; }
uint32_t barcode_attempt_dropped(void) { return 0u; }
#endif
