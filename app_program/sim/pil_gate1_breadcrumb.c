/* Gate1 only: bounded progress stores; never a watchdog feeder of its own. */
#include "pil_gate1_breadcrumb.h"
#include "../platform/drv_time.h"
#include <tk/tkernel.h>
#if !defined(PIL_GATE1_RUNTIME_TEST)
#include "hardware/regs/addressmap.h"
#include "hardware/regs/timer.h"
#include "hardware/regs/sio.h"
_Static_assert(TIMER_BASE + TIMER_TIMERAWL_OFFSET == 0x40054028u,
               "fault timer address changed");
_Static_assert(SIO_BASE + SIO_CPUID_OFFSET == 0xd0000000u,
               "fault CPU id address changed");
_Static_assert(SRAM_BASE == 0x20000000u && SRAM_END == 0x20042000u,
               "fault RAM bounds changed");
#endif

volatile uint32_t pil_bc_live[PIL_BC_WORDS]
    __attribute__((section(".pil_retained"), used, aligned(4)));
volatile uint32_t pil_bc_active;
static uint32_t previous[PIL_BC_WORDS];
static uint8_t boot_line[PIL_BC_LINE];
_Static_assert(sizeof(pil_bc_live) == 112u, "breadcrumb layout changed");
/* Up to256B fault instructions/literals and7B global alignment padding. */
_Static_assert(sizeof(pil_bc_live) + sizeof(previous) + sizeof(boot_line) +
               sizeof(pil_bc_active) + 256u + 7u <= PIL_BC_RAM_BUDGET,
               "breadcrumb storage exceeds approved800B");

static void barrier(void)
{
#if defined(PIL_GATE1_RUNTIME_TEST)
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#else
    __asm__ volatile ("dmb" ::: "memory");
#endif
}
static void publish(unsigned int base, uint32_t time, uint32_t value)
{
    const uint32_t seq = (pil_bc_live[base] + 2u) & ~1u;
    pil_bc_live[base] = seq | 1u;
    barrier();
    pil_bc_live[base + 1u] = time;
    pil_bc_live[base + 2u] = value;
    pil_bc_live[base + 3u] = PIL_BC_SALT ^ base ^ seq ^ time ^ value;
    barrier();
    pil_bc_live[base] = seq;
}
uint32_t pil_gate1_bc_validate(const uint32_t w[PIL_BC_WORDS], uint32_t reason)
{
    if ((reason & 3u) == 0u || w[0] != PIL_BC_MAGIC ||
        w[1] != PIL_BC_PRODUCER || w[2] == 0u ||
        w[3] != (PIL_BC_SALT ^ w[0] ^ w[1] ^ w[2])) { return 0u; }
    uint32_t valid = 1u;
    for (unsigned int base = 4u; base <= 12u; base += 4u) {
        if (w[base] != 0u && (w[base] & 1u) == 0u &&
            w[base + 3u] == (PIL_BC_SALT ^ base ^ w[base] ^
                             w[base + 1u] ^ w[base + 2u])) {
            valid |= 1u << (base / 4u);
        }
    }
    if (w[16] >= 1u && w[16] <= 3u) { valid |= 32u; }
    uint32_t check = PIL_BC_SALT ^ w[16];
    for (unsigned int i = 17u; i <= 25u; ++i) { check ^= w[i]; }
    if ((w[16] == 2u || w[16] == 3u) && w[26] == check &&
        w[27] == ~check) {
        valid |= 16u;
        if (w[16] == 2u) { valid |= 64u; }
    }
    return valid;
}
void pil_gate1_bc_begin(uint32_t nonce, uint32_t reason)
{
    pil_bc_active = 0u;
    for (unsigned int i = 0u; i < PIL_BC_WORDS; ++i) {
        previous[i] = pil_bc_live[i];
    }
    const uint32_t mask = pil_gate1_bc_validate(previous, reason);
    static const char prefix[] = "[PILG1BC1 ";
    static const char hex[] = "0123456789abcdef";
    unsigned int at = sizeof(prefix) - 1u;
    for (unsigned int i = 0u; i < at; ++i) {
        boot_line[i] = (uint8_t)prefix[i];
    }
    for (unsigned int i = 0u; i < 30u; ++i) {
        const uint32_t word = i == 0u ? nonce : i == 1u ? mask : previous[i-2u];
        if (i != 0u) { boot_line[at++] = ' '; }
        for (unsigned int digit = 0u; digit < 8u; ++digit) {
            boot_line[at++] = (uint8_t)hex[(word >> (28u - digit*4u)) & 15u];
        }
    }
    boot_line[at++] = ']'; boot_line[at] = '\n';
    for (unsigned int i = 0u; i < PIL_BC_WORDS; ++i) { pil_bc_live[i] = 0u; }
    pil_bc_live[0] = PIL_BC_MAGIC; pil_bc_live[1] = PIL_BC_PRODUCER;
    pil_bc_live[2] = nonce;
    pil_bc_live[3] = PIL_BC_SALT ^ PIL_BC_MAGIC ^ PIL_BC_PRODUCER ^ nonce;
    barrier(); pil_bc_active = 1u;
}
const uint8_t *pil_gate1_bc_line(void) { return boot_line; }
void pil_gate1_bc_pil(uint32_t state)
{
    if (pil_bc_active != 0u) {
        const uint32_t id = (uint32_t)tk_get_tid();
        const uint32_t value = id <= 0xffffffu ?
            (id << 8u) | state : UINT32_MAX;
        publish(8u, drv_time_us(), value);
    }
}
void pil_gate1_bc_tick(uint32_t entry)
{
    if (pil_bc_active != 0u) { publish(12u, entry, (uint32_t)tk_get_tid()); }
}
extern void __real_drv_watchdog_feed(void);
void __wrap_drv_watchdog_feed(void)
{
    const uint32_t active = pil_bc_active;
    if (active != 0u) {
        /* An interrupted feed must not leave a plausible older publication. */
        pil_bc_live[4] |= 1u;
        barrier();
    }
    __real_drv_watchdog_feed();
    if (active != 0u) {
        publish(4u, drv_time_us(), (uint32_t)tk_get_tid());
    }
}

#if !defined(PIL_GATE1_RUNTIME_TEST)
/* All instructions/literals precede delegation in RAM; no stack or C call. */
__asm__(
".syntax unified\n.thumb\n"
".section .data.ramfunc.pil_fault,\"aw\",%progbits\n.align 2\n"
".global __wrap_HardFault_Handler\n"
".type __wrap_HardFault_Handler,%function\n.thumb_func\n"
"__wrap_HardFault_Handler:\n"
"ldr r2, =pil_bc_active\nldr r2, [r2]\ncmp r2, #0\nbeq 9f\n"
"mov r0, lr\nmovs r2, #4\ntst r0, r2\nbeq 1f\n"
"mrs r1, psp\nb 2f\n1: mrs r1, msp\n2:\n"
"ldr r2, =pil_bc_live+64\nmovs r3, #1\nstr r3, [r2]\n"
"str r1, [r2, #20]\nstr r0, [r2, #24]\n"
"ldr r3, =0x40054028\nldr r3, [r3]\nstr r3, [r2, #4]\n"
"mrs r3, primask\nstr r3, [r2, #28]\n"
"mrs r3, ipsr\nstr r3, [r2, #32]\n"
"ldr r3, =0xd0000000\nldr r3, [r3]\nstr r3, [r2, #36]\n"
"movs r3, #3\ntst r1, r3\nbne 3f\n"
"ldr r3, =0x20000000\ncmp r1, r3\nblo 3f\n"
"ldr r3, =0x20041fe0\ncmp r1, r3\nbhi 3f\n"
"ldr r3, [r1, #24]\nstr r3, [r2, #8]\n"
"ldr r3, [r1, #20]\nstr r3, [r2, #12]\n"
"ldr r3, [r1, #28]\nstr r3, [r2, #16]\nmovs r0, #2\nb 4f\n"
"3: movs r3, #0\nmvns r3, r3\nstr r3, [r2, #8]\n"
"str r3, [r2, #12]\nstr r3, [r2, #16]\nmovs r0, #3\n"
"4: mov r12, r0\nldr r3, =0x9e3779b9\neors r3, r0\n"
"movs r1, #4\n5: ldr r0, [r2, r1]\neors r3, r0\n"
"adds r1, #4\ncmp r1, #40\nblo 5b\n"
"str r3, [r2, #40]\nmvns r3, r3\nstr r3, [r2, #44]\ndmb\n"
"mov r0, r12\nstr r0, [r2]\n"
"9: ldr r0, =__real_HardFault_Handler\nbx r0\n.ltorg\n"
".size __wrap_HardFault_Handler, .-__wrap_HardFault_Handler\n.text\n");
#endif
