#include "pil_gate1_breadcrumb.h"
#include <tk/tkernel.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
static uint32_t clock_us = 100u;
static ID task_id = 10;
static unsigned int feeds;
static int expected_active = 1;
static uint32_t real_time = 100u;
static int interrupt_feed;
static jmp_buf feed_exit;
uint32_t drv_time_us(void) { return clock_us++; }
ID tk_get_tid(void) { return task_id; }
void __real_drv_watchdog_feed(void)
{
    assert((int)(pil_bc_live[4] & 1u) == expected_active);
    assert(clock_us == real_time);
    ++feeds;
    if (interrupt_feed) { longjmp(feed_exit, 1); }
}
static void fault(uint32_t status)
{
    pil_bc_live[16] = status;
    for (unsigned int i = 17u; i <= 25u; ++i) {
        pil_bc_live[i] = 0x12340000u + i;
    }
    if (status == 3u) {
        pil_bc_live[18] = pil_bc_live[19] = pil_bc_live[20] = UINT32_MAX;
    }
    uint32_t check = PIL_BC_SALT ^ status;
    for (unsigned int i = 17u; i <= 25u; ++i) { check ^= pil_bc_live[i]; }
    pil_bc_live[26] = check; pil_bc_live[27] = ~check;
}
int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "valid";
    if (strcmp(mode, "inactive") == 0) {
        expected_active = 0; __wrap_drv_watchdog_feed();
        assert(feeds == 1u && clock_us == 100u && pil_bc_live[4] == 0u);
        return 0;
    }
    if (strcmp(mode, "interrupted") == 0) {
        pil_gate1_bc_begin(0x1234u, 0u);
        __wrap_drv_watchdog_feed(); real_time = clock_us; interrupt_feed = 1;
        if (setjmp(feed_exit) == 0) { __wrap_drv_watchdog_feed(); }
        assert(feeds == 2u && (pil_bc_live[4] & 1u) != 0u);
        uint32_t words[PIL_BC_WORDS];
        for (unsigned int i = 0u; i < PIL_BC_WORDS; ++i) {
            words[i] = pil_bc_live[i];
        }
        assert(pil_gate1_bc_validate(words, 1u) == 1u);
        return 0;
    }
    uint32_t expected = 127u, reason = 1u;
    if (strcmp(mode, "empty") != 0) {
        pil_gate1_bc_begin(0x1234u, 0u);
        __wrap_drv_watchdog_feed(); assert(feeds == 1u);
        task_id = 9; pil_gate1_bc_pil(2u); pil_gate1_bc_tick(1000u);
        assert(pil_bc_live[5] == 100u && pil_bc_live[6] == 10u);
        assert(pil_bc_live[10] == 0x902u && pil_bc_live[14] == 9u);
        fault(2u);
        if (strcmp(mode, "partial") == 0) {
            pil_bc_live[4] |= 1u; pil_bc_live[16] = 1u; expected = 45u;
        } else if (strcmp(mode, "badframe") == 0) {
            fault(3u); expected = 63u;
        } else if (strcmp(mode, "power") == 0) {
            reason = 0u; expected = 0u;
        }
    } else { expected = 0u; }
    uint32_t before[PIL_BC_WORDS];
    for (unsigned int i = 0u; i < PIL_BC_WORDS; ++i) {
        before[i] = pil_bc_live[i];
    }
    assert(pil_gate1_bc_validate(before, reason) == expected);
    if (expected == 127u) {
        for (unsigned int i = 0u; i < PIL_BC_WORDS; ++i) {
            uint32_t damaged[PIL_BC_WORDS];
            memcpy(damaged, before, sizeof(damaged)); damaged[i] ^= 1u;
            const uint32_t mask = pil_gate1_bc_validate(damaged, reason);
            if (i < 4u) { assert(mask == 0u); }
            else if (i < 16u) { assert((mask & (1u << (i/4u))) == 0u); }
            else { assert((mask & (16u | 64u)) == 0u); }
        }
    }
    pil_gate1_bc_begin(0x5678u, reason);
    assert(pil_bc_live[2] == 0x5678u && pil_bc_live[4] == 0u);
    assert(pil_bc_active == 1u);
    const uint8_t *line = pil_gate1_bc_line();
    assert(memcmp(line, "[PILG1BC1 00005678 ", 19u) == 0);
    assert(line[PIL_BC_LINE-2u] == ']' && line[PIL_BC_LINE-1u] == '\n');
    for (unsigned int i = 0u; i < 30u; ++i) {
        unsigned int word;
        assert(sscanf((const char *)line + 10u + i*9u, "%8x", &word) == 1);
        assert(word == (i == 0u ? 0x5678u : i == 1u ? expected : before[i-2u]));
    }
    if (argc > 2) {
        FILE *out = fopen(argv[2], "wb"); assert(out != NULL);
        assert(fwrite(line, 1u, PIL_BC_LINE, out) == PIL_BC_LINE);
        assert(fclose(out) == 0);
    }
    return 0;
}
