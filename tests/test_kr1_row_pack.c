/* CR1 KR1 RAM row: 16-byte KR1 row <-> private 15-byte storage.
 * Every value the real producer (vr_pack) can make must round-trip to the
 * same 16 bytes; anything else must be refused, never stored wrong. */
#define _POSIX_C_SOURCE 199309L
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "../app_program/vehicle/vehicle_log.h"
#include "../app_program/subsystem1/kr1_row_pack.h"
#include "../app_program/subsystem1/telemetry.h"

_Static_assert(KR1_ROW_BYTES == VR_BYTES && KR1_ROW_BYTES == TELEMETRY_BYTES,
               "KR1 row size");

static uint32_t lcg = 12345u;
static uint32_t rnd(void) { lcg = lcg * 1664525u + 1013904223u; return lcg; }

static unsigned long checked;

/* Pack, unpack and compare. Returns the pack result. */
static bool round_trip(const uint8_t in[16], uint32_t index)
{
    uint8_t stored[15], out[16];
    memset(stored, 0xA5, sizeof(stored));
    uint8_t untouched[15];
    memcpy(untouched, stored, sizeof(stored));
    if (!kr1_row_pack(stored, in, index)) {
        assert(memcmp(stored, untouched, sizeof(stored)) == 0);
        return false;
    }
    kr1_row_unpack(out, stored, index);
    assert(memcmp(out, in, 16u) == 0);
    ++checked;
    return true;
}

/* vr_pack the values, then require a lossless round trip at this index. */
static void producer(const vr_values_t *v, uint32_t index)
{
    vr_row_t row;
    vr_pack(&row, v);
    assert(round_trip(row.bytes, index));
}

static vr_values_t base_values(uint32_t index)
{
    return (vr_values_t){.elapsed_ms = index * 20u + 7u,
        .raw = {1234u, 3210u}, .echo = 0x2A5Au,
        .asked = {-300, 450}, .measured = {-17, 511},
        .position = -73, .servo = 29, .state = 2u, .module = 5u,
        .events = 0xA5u};
}

static void test_fields(void)
{
    const uint32_t indices[] = {0u, 1u, 2u, 255u, 4095u, 4499u, 4500u};
    for (unsigned int n = 0u; n < sizeof(indices) / sizeof(indices[0]); ++n) {
        const uint32_t i = indices[n];
        vr_values_t v = base_values(i);
        for (uint32_t s = 0u; s < 4u; ++s) { v = base_values(i); v.state = (uint8_t)s; producer(&v, i); }
        for (uint32_t m = 0u; m < 8u; ++m) { v = base_values(i); v.module = (uint8_t)m; producer(&v, i); }
        for (uint32_t r = 0u; r < 4096u; ++r) {
            v = base_values(i); v.raw[0] = (uint16_t)r; producer(&v, i);
            v = base_values(i); v.raw[1] = (uint16_t)r; producer(&v, i);
        }
        for (int p = -128; p <= 127; ++p) { v = base_values(i); v.position = (int8_t)p; producer(&v, i); }
        for (int32_t s = -32768; s <= 32767; s += 1) {
            /* Whole int16 domain: vr_pack clamps to -512..511 and sets flag 0. */
            if (s > -600 && s < 600) {
                for (unsigned int k = 0u; k < 4u; ++k) {
                    v = base_values(i);
                    int16_t *f = k < 2u ? &v.asked[k] : &v.measured[k - 2u];
                    *f = (int16_t)s;
                    producer(&v, i);
                }
            } else if ((s & 255) == 0 || s == -32768 || s == 32767) {
                v = base_values(i); v.asked[0] = (int16_t)s; producer(&v, i);
                v = base_values(i); v.measured[1] = (int16_t)s; producer(&v, i);
            }
        }
        for (uint32_t e = 0u; e <= 0x3FFFu; ++e) { v = base_values(i); v.echo = (uint16_t)e; producer(&v, i); }
        for (int s = -64; s <= 63; ++s) { v = base_values(i); v.servo = (int8_t)s; producer(&v, i); }
        for (uint32_t e = 0u; e < 256u; ++e) { v = base_values(i); v.events = (uint8_t)e; producer(&v, i); }
    }
}

/* Time: every offset 0..8191 from the row index stores; 8192+ is refused. */
static void test_time_window(void)
{
    const uint32_t indices[] = {0u, 1u, 3u, 1000u, 4500u};
    for (unsigned int n = 0u; n < sizeof(indices) / sizeof(indices[0]); ++n) {
        const uint32_t i = indices[n];
        for (uint32_t d = 0u; d < 8192u + 64u; ++d) {
            for (uint32_t ms = 0u; ms < 20u; ms += 19u) {
                vr_values_t v = base_values(i);
                v.elapsed_ms = (i + d) * 20u + ms;
                vr_row_t row;
                vr_pack(&row, &v);
                assert(round_trip(row.bytes, i) == (d < 8192u));
            }
        }
    }
    /* Guard boundary on raw rows: d = 8191 stores, d = 8192 is refused. */
    uint8_t row[16] = {0};
    row[0] = (uint8_t)(4500u + 8191u); row[1] = (uint8_t)((4500u + 8191u) >> 8);
    assert(round_trip(row, 4500u));
    row[0] = (uint8_t)(4500u + 8192u); row[1] = (uint8_t)((4500u + 8192u) >> 8);
    assert(!round_trip(row, 4500u));
    /* Time below the index (impossible for real rows) is refused too. */
    row[0] = 3u; row[1] = 0u;
    assert(!round_trip(row, 4u));
}

/* The 16-bit time field wraps after 65536 ticks (21.8 min). The guard and
 * decode act on the stored 16-bit field, so the output stays byte-identical:
 * a wrapped row is stored only when (time16 - index) mod 65536 < 8192. */
static void test_wrap(void)
{
    vr_values_t v = base_values(3u);
    v.elapsed_ms = (65536u + 5u) * 20u; /* time16 = 5, index 3: offset 2 */
    producer(&v, 3u);
    v.elapsed_ms = (65536u + 1u) * 20u; /* time16 = 1, index 4: refused */
    vr_row_t row;
    vr_pack(&row, &v);
    assert(!round_trip(row.bytes, 4u));
    v.elapsed_ms = 65535u * 20u; /* index 0, offset 65535: refused */
    vr_pack(&row, &v);
    assert(!round_trip(row.bytes, 0u));
    v.elapsed_ms = 65535u * 20u; /* arithmetic only: offset 8191 at top */
    vr_pack(&row, &v);
    assert(round_trip(row.bytes, 65535u - 8191u));
    uint8_t raw[16] = {0x02u, 0x00u}; /* (65534 + 4) mod 65536 = 2: decode wraps */
    assert(round_trip(raw, 65534u));
}

/* Every single input bit: kept bits round-trip, zero-only bits are refused. */
static void test_bits(void)
{
    for (unsigned int bit = 16u; bit < 128u; ++bit) {
        uint8_t row[16] = {0};
        row[bit / 8u] = (uint8_t)(1u << (bit % 8u));
        const bool zero_only = bit == 107u || bit == 108u || bit >= 125u;
        assert(round_trip(row, 0u) == !zero_only);
    }
    uint8_t all[16];
    memset(all, 0xFF, sizeof(all));
    all[0] = 0u; all[1] = 0u;
    all[13] &= (uint8_t)~0x18u;  /* bits 107..108 */
    all[15] &= (uint8_t)~0xE0u;  /* bits 125..127 */
    assert(round_trip(all, 0u));
}

/* No producer value sets a zero-only bit: random values over the whole C
 * domain of vr_values_t, with echo in its 14-bit ultrasonic_result_t range. */
static void test_random_producer(void)
{
    for (unsigned long n = 0u; n < 400000u; ++n) {
        const uint32_t i = rnd() % 4501u;
        const vr_values_t v = {.elapsed_ms = (i + rnd() % 8192u) * 20u + rnd() % 20u,
            .raw = {(uint16_t)rnd(), (uint16_t)rnd()}, .echo = (uint16_t)(rnd() & 0x3FFFu),
            .asked = {(int16_t)rnd(), (int16_t)rnd()},
            .measured = {(int16_t)rnd(), (int16_t)rnd()},
            .position = (int8_t)rnd(), .servo = (int8_t)rnd(),
            .state = (uint8_t)rnd(), .module = (uint8_t)rnd(), .events = (uint8_t)rnd()};
        producer(&v, i);
    }
    /* Contract edge: an echo above 14 bits is refused, never truncated. */
    vr_values_t v = base_values(0u);
    v.echo = 0x4000u;
    vr_row_t row;
    vr_pack(&row, &v);
    assert(!round_trip(row.bytes, 0u));
}

static double ns_between(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) * 1e9 + (double)(b->tv_nsec - a->tv_nsec);
}

/* Host timing of one pack and one unpack (diagnostic, not a gate). */
static void bench(void)
{
    enum { N = 20000000 };
    uint8_t in[16], stored[15], out[16];
    memset(in, 0x5A, sizeof(in));
    in[0] = 0u; in[1] = 0u; in[13] &= (uint8_t)~0x18u; in[15] &= (uint8_t)~0xE0u;
    volatile uint32_t sink = 0u;
    struct timespec a, b;
    clock_gettime(CLOCK_MONOTONIC, &a);
    for (uint32_t n = 0u; n < N; ++n) {
        in[0] = (uint8_t)n;
        sink += kr1_row_pack(stored, in, n & 0xFFu) ? stored[n % 15u] : 0u;
        __asm__ volatile("" ::: "memory");
    }
    clock_gettime(CLOCK_MONOTONIC, &b);
    const double pack_ns = ns_between(&a, &b) / N;
    clock_gettime(CLOCK_MONOTONIC, &a);
    for (uint32_t n = 0u; n < N; ++n) {
        stored[0] = (uint8_t)n;
        kr1_row_unpack(out, stored, n & 0xFFFu);
        sink += out[3];
        __asm__ volatile("" ::: "memory");
    }
    clock_gettime(CLOCK_MONOTONIC, &b);
    const double unpack_ns = ns_between(&a, &b) / N;
    printf("kr1_row host pack %.2f ns unpack %.2f ns (sink %u)\n", pack_ns, unpack_ns,
           (unsigned int)sink);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "bench") == 0) { bench(); return 0; }
    test_fields();
    test_time_window();
    test_wrap();
    test_bits();
    test_random_producer();
    printf("kr1_row round trips checked: %lu\n", checked);
    return 0;
}
