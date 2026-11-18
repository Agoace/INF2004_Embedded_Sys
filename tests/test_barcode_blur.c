/* Hypothetical 141 mm optical widths, NOT a measured small-print trace.
 * All 43 symbols, partial narrow peaks, bounded additive bar/space bias. */
#include "../app_program/subsystem3/barcode.h"
#include <assert.h>
#include <string.h>
static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
static const uint16_t patterns[] = {
    0x034u, 0x121u, 0x061u, 0x160u, 0x031u, 0x130u, 0x070u, 0x025u, 0x124u,
    0x064u, 0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu,
    0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u,
    0x106u, 0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u,
    0x085u, 0x184u, 0x0c4u, 0x0a8u, 0x0a2u, 0x08au, 0x02au};
static barcode_sample_t rows[BC_MAX_SAMPLES];
static barcode_motion_t motors[BC_MAX_MOTION];
static barcode_workspace_t workspace;
static barcode_report_t report;
static size_t count;
static uint32_t elapsed;

static void segment(uint16_t raw, uint32_t duration)
{
    for (uint32_t i = 0u; i < duration; ++i, ++elapsed) {
        if (elapsed % 2u == 0u) {
            assert(count < BC_MAX_SAMPLES);
            rows[count++] = (barcode_sample_t){elapsed * 1000u, raw, 0u};
        }
    }
}

static void symbol(uint16_t pattern, bool reverse, uint16_t peak)
{
    for (unsigned int i = 0u; i < 9u; ++i) {
        const bool wide =
            (pattern & (UINT16_C(0x0100) >> (reverse ? 8u - i : i))) != 0u;
        const bool bar = (i & 1u) == 0u;
        const uint32_t ideal = wide ? 87u : 29u;
        const uint32_t duration = bar ? ideal - 16u : ideal + 16u;
        segment(bar ? (wide ? 1500u : peak) : 150u, duration);
    }
}

static void run(uint16_t pattern, bool reverse, uint16_t peak)
{
    count = 0u;
    elapsed = 0u;
    segment(150u, 400u);
    symbol(UINT16_C(0x094), reverse, peak);
    segment(150u, 45u);
    symbol(pattern, reverse, peak);
    segment(150u, 45u);
    symbol(UINT16_C(0x094), reverse, peak);
    segment(150u, 400u);
    segment(1989u, 200u); /* Unrelated dark map area, as in real capture. */
    size_t m = 0u;
    for (uint32_t ms = 0u; ms <= elapsed + 10u; ms += 10u) {
        assert(m < BC_MAX_MOTION);
        motors[m++] = (barcode_motion_t){ms * 1000u, (int32_t)ms, (int32_t)ms};
    }
    barcode_config_t config = barcode_default_config;
    config.counts_per_m = 10000u;
    barcode_decode(rows, count, motors, m, &config, &workspace, &report);
}

int main(void)
{
    _Static_assert(sizeof(patterns) / sizeof(patterns[0]) ==
                       sizeof(alphabet) - 1u,
                   "Fixture table mismatch");
    for (size_t i = 0u; i < sizeof(alphabet) - 1u; ++i) {
        for (unsigned int reverse = 0u; reverse < 2u; ++reverse) {
            run(patterns[i], reverse != 0u, 350u);
            assert(report.time.decoded && report.distance.decoded);
            assert(report.time.text[0] == alphabet[i]);
            assert(report.distance.text[0] == alphabet[i]);
            assert(report.time.text[1] == '\0');
            if (alphabet[i] != 'A' && alphabet[i] != 'B' &&
                alphabet[i] != 'C' && alphabet[i] != 'D') {
                assert(report.time.command == BC_CMD_NONE);
            }
        }
    }
    run(UINT16_C(0x058), false, 320u); /* Below required rise 333.9. */
    assert(!report.time.decoded && !report.distance.decoded);
    return 0;
}
