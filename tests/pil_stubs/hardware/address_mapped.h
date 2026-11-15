#ifndef PIL_FAKE_ADDRESS_H
#define PIL_FAKE_ADDRESS_H
#include <stdint.h>
static inline void hw_set_bits(uint32_t *p, uint32_t v) { *p |= v; }
static inline void hw_clear_bits(uint32_t *p, uint32_t v) { *p &= ~v; }
#endif
