/* Private local byte codecs. Callers prove bounds and hold the payload lock
 * or use local scratch. These helpers are NOT shared-memory atomics. */
#ifndef APP_VALUE_BITS_H
#define APP_VALUE_BITS_H
#include <stdint.h>
static inline void value_bits_put(uint8_t *bytes, unsigned int *offset,
                                  uint32_t value, unsigned int bits)
{
    for (unsigned int i=0u;i<bits;++i) {
        const unsigned int bit=*offset+i;
        const uint8_t mask=(uint8_t)(1u << (bit%8u));
        bytes[bit/8u]=(uint8_t)((bytes[bit/8u] & (uint8_t)~mask) |
            (uint8_t)(((value >> i)&1u) << (bit%8u)));
    }
    *offset+=bits;
}
static inline uint32_t value_bits_get(const uint8_t *bytes,
                                      unsigned int *offset, unsigned int bits)
{
    uint32_t value=0u;
    for (unsigned int i=0u;i<bits;++i) {
        const unsigned int bit=*offset+i;
        value|=(uint32_t)((bytes[bit/8u] >> (bit%8u))&1u) << i;
    }
    *offset+=bits;
    return value;
}
/* Defined signed reconstruction: no implementation-defined out-of-range cast. */
static inline int32_t value_bits_i32(uint32_t value)
{
    return value <= INT32_MAX ? (int32_t)value :
        -1 - (int32_t)(UINT32_MAX-value);
}
#endif
