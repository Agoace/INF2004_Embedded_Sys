/* CR1 F pack: snapshot 16 -> 8 B, STOP capture 32 -> 12 B. Every valid
 * value round-trips; every out-of-field bit is refused without a write. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/platform/fault_pack.h"

static uint32_t lcg = 99u;
static uint32_t rnd(void) { lcg = lcg * 1664525u + 1013904223u; return lcg; }

static void snapshot_round_trip(fault_mask_snapshot_t v)
{
    fault_snapshot_packed_t p;
    fault_mask_snapshot_t back;
    assert(fault_snapshot_pack(&p, &v));
    assert((p.masks & 0xC0000000u) == 0u);
    fault_snapshot_unpack(&back, &p);
    assert(memcmp(&back, &v, sizeof(v)) == 0);
}

static void test_snapshot(void)
{
    for (uint32_t m = 0u; m <= FAULT_MASK_ALL; ++m) {
        snapshot_round_trip((fault_mask_snapshot_t){m, rnd() & 0x3ffu, rnd() & 0x3ffu, rnd()});
        snapshot_round_trip((fault_mask_snapshot_t){rnd() & 0x3ffu, m, rnd() & 0x3ffu, rnd()});
        snapshot_round_trip((fault_mask_snapshot_t){rnd() & 0x3ffu, rnd() & 0x3ffu, m, rnd()});
    }
    snapshot_round_trip((fault_mask_snapshot_t){0x3ffu, 0x3ffu, 0x3ffu, UINT32_MAX});
    snapshot_round_trip((fault_mask_snapshot_t){0u, 0u, 0u, 0u});
    for (unsigned int bit = 10u; bit < 32u; ++bit) {
        for (unsigned int f = 0u; f < 3u; ++f) {
            fault_mask_snapshot_t v = {1u, 2u, 3u, 4u};
            uint32_t *w = f == 0u ? &v.now : f == 1u ? &v.run : &v.valid;
            *w |= UINT32_C(1) << bit;
            fault_snapshot_packed_t p = {0xA5A5A5A5u, 0x5A5A5A5Au};
            assert(!fault_snapshot_pack(&p, &v));
            assert(p.masks == 0xA5A5A5A5u && p.epoch == 0x5A5A5A5Au);
        }
    }
    assert(!fault_snapshot_pack(0, &(fault_mask_snapshot_t){0}));
}

static void stop_round_trip(uint32_t faults, uint32_t valid, uint32_t epoch, uint32_t us)
{
    fault_stop_values_t v = {.valid = valid, .epoch = epoch, .us = us};
    for (unsigned int i = 0u; i < FAULT_PACK_OWNERS; ++i) {
        v.owner[i] = faults & fault_pack_owner_masks[i];
    }
    fault_stop_packed_t p;
    fault_stop_values_t back;
    assert(fault_stop_pack(&p, &v));
    assert((p.masks & 0xFFF00000u) == 0u);
    fault_stop_unpack(&back, &p);
    assert(memcmp(&back, &v, sizeof(v)) == 0);
}

static void test_stop(void)
{
    /* All 2^10 owner patterns x all 2^10 valid patterns. */
    for (uint32_t f = 0u; f <= FAULT_MASK_ALL; ++f) {
        for (uint32_t valid = 0u; valid <= FAULT_MASK_ALL; ++valid) {
            stop_round_trip(f, valid, f * 7u + valid, valid * 13u);
        }
    }
    stop_round_trip(FAULT_MASK_ALL, FAULT_MASK_ALL, UINT32_MAX, UINT32_MAX);
    /* Each owner field is contiguous and in its stated slot. */
    const struct { unsigned int owner; uint32_t bit; uint32_t slot; } map[] = {
        {0u, FAULT_MOTION_STALL_LEFT, 1u << 0}, {0u, FAULT_MOTION_STALE, 1u << 3},
        {1u, FAULT_IMU_SAFETY, 1u << 4}, {1u, FAULT_SHELL_OTHER, 1u << 5},
        {2u, FAULT_SONAR_SENSOR, 1u << 6}, {3u, FAULT_LOG_FULL, 1u << 7},
        {4u, FAULT_WIFI_LINK_DOWN, 1u << 8}, {4u, FAULT_MQTT_DISCONNECTED, 1u << 9}};
    for (unsigned int n = 0u; n < sizeof(map) / sizeof(map[0]); ++n) {
        fault_stop_values_t v = {0};
        v.owner[map[n].owner] = map[n].bit;
        fault_stop_packed_t p;
        assert(fault_stop_pack(&p, &v) && p.masks == map[n].slot);
    }
    /* Refusal: a bit outside its owner, or valid above bit 9. No write. */
    for (unsigned int i = 0u; i < FAULT_PACK_OWNERS; ++i) {
        for (unsigned int bit = 0u; bit < 32u; ++bit) {
            const uint32_t b = UINT32_C(1) << bit;
            if ((b & fault_pack_owner_masks[i]) != 0u) continue;
            fault_stop_values_t v = {0};
            v.owner[i] = b;
            fault_stop_packed_t p = {0xA5A5A5A5u, 1u, 2u};
            assert(!fault_stop_pack(&p, &v));
            assert(p.masks == 0xA5A5A5A5u && p.epoch == 1u && p.us == 2u);
        }
    }
    for (unsigned int bit = 10u; bit < 32u; ++bit) {
        fault_stop_values_t v = {.valid = UINT32_C(1) << bit};
        fault_stop_packed_t p = {0xA5A5A5A5u, 1u, 2u};
        assert(!fault_stop_pack(&p, &v) && p.masks == 0xA5A5A5A5u);
    }
    /* Every stored pattern decodes; extra stored bits do not leak out. */
    fault_stop_packed_t junk = {UINT32_MAX, 3u, 4u};
    fault_stop_values_t out;
    fault_stop_unpack(&out, &junk);
    for (unsigned int i = 0u; i < FAULT_PACK_OWNERS; ++i) {
        assert(out.owner[i] == fault_pack_owner_masks[i]);
    }
    assert(out.valid == FAULT_MASK_ALL);
}

int main(void)
{
    test_snapshot();
    test_stop();
    printf("fault pack: snapshot and STOP capture round trips pass\n");
    return 0;
}
