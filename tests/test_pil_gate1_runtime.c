/* Real runtime/session/stream with fake kernel/MMIO, never serial access. */
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "pil_gate1_runtime.h"
#include "pil_gate1_session.h"
#include "pil_gate1_uart_hw.h"
#include "pil_gate1_reset.h"
#include "pil_gate1_dump.h"
#include "hardware/structs/uart.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Production UP ready queue; only the consumed TCB fields are modelled. */
#define Inline static inline
#define IMPORT extern
#define TRUE 1
#define FALSE 0
#define NUM_TSKPRI 32
#define INT_BITWIDTH 32
typedef int BOOL;
typedef int32_t W;
#include "../include/sys/queue.h"
typedef struct {
    QUEUE tskque;
    INT priority;
    BOOL klocked, klockwait;
} TCB;
#include "../kernel/tkernel/ready_queue.h"
void *knl_memset(void *bytes, int value, SZ length)
{
    assert(length >= 0); return memset(bytes, value, (size_t)length);
}
static void priority_order(INT worker_priority)
{
    RDYQUE ready;
    TCB app[8] = {0}, pil = {0};
    const INT priorities[8] = {10, 11, 12, 13, 14, 21, 30, 1};
    knl_ready_queue_initialize(&ready);
    for (unsigned int i = 0u; i < 7u; ++i) {
        app[i].priority = priorities[i] - 1;
        (void)knl_ready_queue_insert(&ready, &app[i]);
    }
    assert(knl_ready_queue_top(&ready) == &app[0]);
    pil.priority = worker_priority - 1;
    assert(knl_ready_queue_insert(&ready, &pil));
    assert(knl_ready_queue_top(&ready) == &pil);
    /* Priority does not bypass a kernel-lock owner or initial task1. */
    ready.klocktsk = &app[0];
    assert(knl_ready_queue_top(&ready) == &app[0]);
    ready.klocktsk = NULL;
    app[7].priority = 0;
    (void)knl_ready_queue_insert(&ready, &app[7]);
    assert(knl_ready_queue_top(&ready) == &app[7]);
    knl_ready_queue_delete(&ready, &app[7]);
    assert(knl_ready_queue_top(&ready) == &pil);
    knl_ready_queue_delete(&ready, &pil);
    assert(knl_ready_queue_top(&ready) == &app[0]);
}

extern INT __wrap_usermain(void);
extern pil_gate1_session_t *pil_gate1_test_session(void);
extern void pil_gate1_test_service(void);
extern uint32_t pil_gate1_test_rx_depth(void);
static UINT primask, dispatch_disabled;
static bool pendsv, expect_rx_publication;
static unsigned int wakes;
static TCB background, pil_context;
static TCB *current = &background, *scheduled = &background;
UINT pil_test_irq_disable(void)
{
    const UINT saved = primask; primask = 1u; return saved;
}
void pil_test_irq_restore(UINT saved) { assert(saved <= 1u); primask = saved; }
static UINT disint(void) { return pil_test_irq_disable(); }
static void set_primask(UINT saved) { pil_test_irq_restore(saved); }
static BOOL isDI(UINT saved) { return saved != 0u; }
static TCB **knl_ctxtsk_slot(void) { return &current; }
static TCB **knl_schedtsk_slot(void) { return &scheduled; }
static UINT *knl_dispatch_disabled_slot(void) { return &dispatch_disabled; }
static void knl_dispatch(void) { pendsv = true; }
#define KNL_KERNEL_LOCK_ENTER() ((void)0)
#define KNL_KERNEL_LOCK_LEAVE() ((void)0)
#include "pil_gate1_critical.h"
static FP task;
static void (*handlers[32])(UINT);
static jmp_buf sleep_exit;
static uint32_t now = 500000u, deadline;
/* PL011 register double: TXIM alone does not raise TX IRQ on an empty FIFO. */
uart_hw_t fake_uart;
static unsigned int fifo_depth;
static bool tx_pending;
#define tx_enabled ((fake_uart.imsc & 32u) != 0u)
static uint32_t uart_errors;
static uint8_t rx[256], uart[32768], usb[32768];
static size_t rx_at, rx_size, uart_size, usb_size;
static unsigned int reset_reads;
static UW usb_state = 3u, usb_pending;
void pil_gate1_reset_snapshot(pil_gate1_reset_t *out)
{
    ++reset_reads; assert(reset_reads == 1u);
    out->reason = 1u; out->chip_reset = 0x100u;
    for (unsigned int i = 0u; i < 8u; ++i) { out->scratch[i] = 0x20u + i; }
}
ID tk_cre_tsk(const T_CTSK *c)
{
    priority_order(c->itskpri);
    assert(c->itskpri == 9 && c->stksz == 2048);
    assert(c->bufptr != NULL && (c->tskatr & TA_USERBUF) != 0u);
#if TK_SUPPORT_SMP
    assert((c->tskatr & TA_ASSPRC) != 0u && c->assprc == TP_PRC1);
#endif
    task = c->task; return 9;
}
ER tk_sta_tsk(ID id, INT code) { assert(id == 9 && code == 0); return 0; }
ER tk_del_tsk(ID id) { (void)id; return 0; }
ER tk_wup_tsk(ID id)
{
    assert(id == 9);
    if (expect_rx_publication) { assert(pil_gate1_test_rx_depth() >= 1u); }
    BEGIN_CRITICAL_SECTION;
    ++wakes; scheduled = &pil_context;
    END_CRITICAL_SECTION;
    return 0;
}
ER tk_slp_tsk(INT delay) { (void)delay; longjmp(sleep_exit, 1); }
ER tk_def_int(UINT n, const T_DINT *d)
{
    assert(n < 32u); handlers[n] = (void (*)(UINT))d->inthdr; return 0;
}
void DisableInt(UINT n) { assert(n == 1u || n == 20u); }
void EnableInt(UINT n, UINT priority)
{
    assert(n == 1u || n == 20u); assert(priority == 1u);
}
void ClearInt(UINT n) { assert(n == 1u || n == 20u); }
uint32_t drv_time_us(void) { return now++; }
int drv_time_init(void) { return 0; }
bool boot_id_hw_generate(uint32_t *out) { *out = 123u; return true; }
bool app_startup_complete(void) { return true; }
uint32_t pil_gate1_native_entry(void) { return now; }
INT __real_usermain(void)
{
    assert(reset_reads == 1u); /* Before native watchdog/scratch overwrite. */
    pil_gate1_native_grid(now + 1000u);
    pil_gate1_motor_seen(0, 0, 1500u); return 0;
}
UW tm_usb_state(void) { return usb_state; }
UW tm_usb_pending(void) { return usb_pending; }
void tm_printf(UB *fmt, ...)
{
    if (strstr((const char *)fmt, "stopped; reset required") == NULL) {
        return;
    }
    assert(strcmp((const char *)fmt, "[PILG1] stopped; reset required; "
                  "worker stack untouched %lu/2048 bytes\n") == 0);
    char text[256]; va_list args;
    va_start(args, fmt);
    const unsigned long untouched = va_arg(args, unsigned long);
    va_end(args);
    const int n = snprintf(text, sizeof(text),
        "[PILG1] stopped; reset required; "
        "worker stack untouched %lu/2048 bytes\n", untouched);
    assert(n > 0 && (size_t)n < sizeof(text));
    assert(usb_size + (size_t)n < sizeof(usb));
    memcpy(usb + usb_size, text, (size_t)n); usb_size += (size_t)n;
}
void tm_snd_dat(const UB *bytes, INT length)
{
    assert(length > 0 && usb_size + (size_t)length <= sizeof(usb));
    memcpy(usb + usb_size, bytes, (size_t)length); usb_size += (size_t)length;
}
bool pil_gate1_uart_init(uint32_t hz) { assert(hz == 125000000u); return true; }
bool pil_gate1_uart_rx(uint8_t *byte, uint32_t *errors)
{
    if (rx_at == rx_size) { return false; }
    *byte = rx[rx_at++]; *errors = 0u; return true;
}
bool pil_gate1_uart_tx(uint8_t byte)
{
    if (fifo_depth == 32u) { return false; }
    assert(uart_size < sizeof(uart)); uart[uart_size++] = byte;
    ++fifo_depth; fake_uart.fr = 8u | (fifo_depth == 32u ? 32u : 0u);
    return true;
}
void pil_gate1_uart_tx_enable(bool enabled)
{
    if (enabled) { fake_uart.imsc |= 32u; }
    else { fake_uart.imsc &= ~32u; }
}
uint32_t pil_gate1_uart_errors(void)
{
    const uint32_t result = uart_errors; uart_errors = 0u; return result;
}
void pil_gate1_uart_ack(void) { tx_pending = false; }
bool pil_gate1_uart_idle(void) { return fifo_depth == 0u; }
void pil_gate1_uart_disable(void) {}
void pil_gate1_lease_ack(void) {}
void pil_gate1_lease_set(bool enabled, uint32_t value)
{
    deadline = enabled ? value : 0u;
}
static void drain(void)
{
    for (unsigned int i = 0u; i < 100u; ++i) {
        if (fifo_depth > 16u) {
            fifo_depth = 16u; tx_pending = true;
        } else if (fifo_depth != 0u) { fifo_depth = 0u; }
        fake_uart.fr = fifo_depth == 0u ? 128u : 8u;
        if (tx_pending && tx_enabled) { handlers[20](20u); }
        if (fifo_depth == 0u && !tx_pending) { break; }
    }
    assert(!tx_enabled && fifo_depth == 0u);
}
static void send(pil_frame_t *f)
{
    assert(pil_encode(f, rx, sizeof(rx), &rx_size) == PIL_OK);
    rx_at = 0u;
    while (rx_at < rx_size) { handlers[20](20u); }
    pil_gate1_test_service(); drain();
}
static void alarm_at(uint32_t time)
{
    pil_gate1_native_grid(time); now = time + 10u;
    pil_gate1_native_alarm();
    pil_gate1_native_grid(time + 1000u);
    pil_gate1_test_service(); drain();
}
int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "clean";
    if (strcmp(mode, "reset") == 0) { usb_state = 0u; }
    const bool missing = strcmp(mode, "missing") == 0;
    const uint32_t applied = missing ? 0u : 8u;
    const uint32_t staged = missing ? 1u : 11u;
    assert(__wrap_usermain() == 0 && task != NULL);
    if (strcmp(mode, "priority") == 0) { return 0; }
    if (setjmp(sleep_exit) == 0) {
        ((void (*)(INT, void *))task)(0, NULL);
    }
    if (strcmp(mode, "reset") == 0) {
        static const char expected[] = "[PILG1RESET1 0000007b 00000000 "
            "00000001 00000100 00000020 00000021 00000022 00000023 "
            "00000024 00000025 00000026 00000027]\n";
        assert(reset_reads == 1u && usb_size == 0u);
        usb_state = 3u; usb_pending = 513u;
        pil_gate1_test_service(); assert(usb_size == 0u);
        usb_pending = 0u; pil_gate1_test_service();
        assert(usb_size == PIL_GATE1_RESET_LINE);
        assert(sizeof(expected) - 1u == usb_size);
        assert(memcmp(usb, expected, usb_size) == 0);
        for (unsigned int i = 0u; i < 5u; ++i) { pil_gate1_test_service(); }
        assert(reset_reads == 1u && usb_size == PIL_GATE1_RESET_LINE);
        return 0;
    }
    /* No RX traffic or synthetic TX interrupt may bootstrap transmission. */
    assert(uart_size == 32u && fifo_depth == 32u && tx_enabled);
    if (strcmp(mode, "wake") == 0 || strcmp(mode, "wakelease") == 0 ||
        strcmp(mode, "wakenative") == 0) {
        const bool uart_case = strcmp(mode, "wake") == 0;
        /* A saved mask1 suppresses PendSV even after the caller restores0. */
        UINT saved; DI(saved); (void)tk_wup_tsk(9);
        assert(scheduled == &pil_context && !pendsv);
        EI(saved); assert(primask == 0u && !pendsv);
        dispatch_disabled = 1u; (void)tk_wup_tsk(9);
        assert(!pendsv); dispatch_disabled = 0u;
        scheduled = current; wakes = 0u;
        pil_frame_t request = {.type = PIL_HELLO, .payload_length = 24u,
            .session = 456u, .ack = 0u};
        const uint32_t words[6] = {123u,456u,4u,1000u,4000u,5000u};
        for (unsigned int i = 0u; i < 6u; ++i) {
            pil_put_u32(request.payload + i * 4u, words[i]);
        }
        assert(pil_encode(&request, rx, sizeof(rx), &rx_size) == PIL_OK);
        rx_at = 0u; expect_rx_publication = true;
        while (rx_at < rx_size) { handlers[20](20u); }
        assert(primask == 0u && scheduled == &pil_context);
        if (uart_case) { assert(pendsv); }
        assert(wakes == 1u && pil_gate1_test_rx_depth() == 1u);
        expect_rx_publication = false;
        /* Completion/error IRQs also request dispatch, without RX traffic. */
        scheduled = current; pendsv = false; wakes = 0u; drain();
        assert(wakes == 1u && primask == 0u);
        if (uart_case) {
            assert(pendsv);
            pil_gate1_test_service(); /* HELLO_ACK now has a FIFO tail. */
            scheduled = current; pendsv = false; wakes = 0u;
            fifo_depth = 8u; fake_uart.fr = 8u; tx_pending = true;
            uart_errors = 8u; handlers[20](20u);
            /* Error plus descriptor completion must coalesce to one wake. */
            assert(pendsv && wakes == 1u && primask == 0u && !tx_enabled);
        } else {
            pil_gate1_test_service(); drain();
            assert(pil_gate1_test_session()->session == 456u);
            scheduled = current; pendsv = false; wakes = 0u;
            if (strcmp(mode, "wakelease") == 0) {
                handlers[1](1u);
            } else {
                const uint32_t at = pil_gate1_test_session()->epoch - 50000u;
                pil_gate1_native_grid(at); now = at + 10u;
                pil_gate1_native_alarm();
            }
            assert(pendsv && wakes == 1u && primask == 0u);
        }
        return 0;
    }
    /* Finish the descriptor while bytes still shift: TXIM is now off.
     * A new enqueue must refill this nonempty FIFO, not wait for an edge. */
    fifo_depth = 16u; tx_pending = true; handlers[20](20u);
    assert(!tx_enabled && fifo_depth == 26u);
    for (unsigned int i = 0u; i < 20u; ++i) {
        now += 100000u;
        pil_gate1_session_t *state = pil_gate1_test_session();
        const uint32_t next = state->grid_anchor +
            ((now - state->grid_anchor) / 1000u + 1u) * 1000u;
        pil_gate1_native_grid(next);
        pil_gate1_test_service();
        assert(tx_enabled && fifo_depth == 32u && state->reason == 0u);
        drain();
        assert(state->counts[G1_TX_FULL] == 0u);
    }
    pil_frame_t f = {.type = PIL_HELLO, .payload_length = 24u,
        .session = 456u, .ack = 0u};
    const uint32_t hello[6] = {123u,456u,4u,1000u,4000u,5000u};
    for (unsigned int i = 0u; i < 6u; ++i) {
        pil_put_u32(f.payload + i * 4u, hello[i]);
    }
    send(&f);
    pil_gate1_session_t *s = pil_gate1_test_session();
    assert(s->session == 456u && s->hello_ack != PIL_NO_ACK);
    alarm_at(s->epoch - 50000u);
    f = (pil_frame_t){.type = PIL_ARM, .payload_length = 8u,
        .session = 456u, .ack = s->hello_ack};
    pil_put_u32(f.payload, 123u); pil_put_u32(f.payload + 4u, 456u);
    send(&f); assert(s->flags == 20u && deadline != 0u);
    alarm_at(s->epoch - 49000u);
    for (uint32_t n = 0u; n < (missing ? 1u : 3u); ++n) {
        now = s->epoch + n * 1000u - 3000u;
        f = (pil_frame_t){.type = PIL_SENSOR, .payload_length = 40u,
            .session = 456u, .sequence = n, .source_us = n * 1000u,
            .ack = PIL_NO_ACK};
        f.payload[7] = 1u; pil_put_u16(f.payload + 36u, 0xc18u); send(&f);
    }
    for (uint32_t n = 0u; n < applied; ++n) {
        alarm_at(s->epoch + n * 1000u);
        assert(s->flags == 1u && s->reason == 0u);
        now = s->epoch + n * 1000u + 200u;
        f.sequence = n + 3u; f.source_us = (n + 3u) * 1000u; send(&f);
    }
    assert(s->counts[G1_STAGED] == staged);
    assert(s->counts[G1_APPLIED] == applied);
    uint32_t reason = 1u, coverage = 0x43fu;
    if (missing) {
        alarm_at(s->epoch); reason = 0x100u; coverage = 0x49fu;
        assert(s->counts[G1_MISSED] == 1u);
        f.sequence = 1u; f.source_us = 1000u;
        send(&f); /* Keep the rejected post-fault packet in diagnostics. */
        assert(s->counts[G1_DROPPED] == 1u);
    } else if (strcmp(mode, "lease") == 0) {
        now = deadline; handlers[1](1u); reason = 0x104u; coverage = 0x49fu;
        assert(s->counts[G1_LEASE] == 1u);
    } else if (strcmp(mode, "overrun") == 0) {
        uart_errors = 8u; handlers[20](20u); reason = 0x109u; coverage = 0x49fu;
        assert(s->counts[G1_OVERRUN] == 1u);
    } else if (strcmp(mode, "lateprogress") == 0) {
        now = deadline; ++f.sequence; f.source_us += 1000u; send(&f);
        reason = 0x104u; coverage = 0x49fu;
        assert(s->counts[G1_LEASE] == 1u);
    } else if (strcmp(mode, "queuefull") == 0) {
        for (unsigned int n = 0u; n < 9u; ++n) {
            assert(pil_encode(&f, rx, sizeof(rx), &rx_size) == PIL_OK);
            rx_at = 0u;
            while (rx_at < rx_size) { handlers[20](20u); }
        }
        reason = 0x107u; coverage = 0x59fu;
        assert(s->counts[G1_RX_FULL] == 1u);
    } else {
        assert(strcmp(mode, "clean") == 0);
        f = (pil_frame_t){.type = PIL_DISARM, .payload_length = 4u,
            .session = 0u, .ack = PIL_NO_ACK};
        pil_put_u32(f.payload, 1u); send(&f);
    }
    for (unsigned int i = 0u; i < 256u; ++i) {
        pil_gate1_test_service(); drain();
    }
    assert(s->reason == reason && s->frozen);
    assert(s->counts[G1_CANCELLED] == (reason == 1u ? staged - applied : 0u));
    assert(s->coverage == coverage);
    assert(s->counts[G1_OUTPUT] == applied);
    assert(s->counts[G1_ENQUEUED] == applied && s->counts[G1_FIFO] == applied);
    size_t uart_at = 0u;
    uint32_t output_sequence = 0u;
    bool inhibited_seen = false, pending_seen = false;
    while (uart_at < uart_size) {
        const size_t start = uart_at;
        while (uart_at < uart_size && uart[uart_at] != 0u) { ++uart_at; }
        assert(uart_at < uart_size); ++uart_at;
        pil_frame_t decoded;
        assert(pil_decode(uart + start, uart_at - start, &decoded) == PIL_OK);
        if (decoded.type == PIL_OUTPUT) {
            assert(decoded.sequence == output_sequence++);
            const uint8_t flags = decoded.payload[7];
            if (flags == 4u && decoded.ack == PIL_NO_ACK) {
                inhibited_seen = true;
            } else if (flags == 20u) {
                assert(decoded.ack == PIL_NO_ACK); pending_seen = true;
            }
        }
    }
    assert(inhibited_seen && pending_seen && output_sequence >= 3u);
    assert(usb_size >= PIL_GATE1_RESET_LINE);
    size_t pos = PIL_GATE1_RESET_LINE;
    uint32_t chain = 0u, records = 0u;
    bool manifest = false;
    while (pos < usb_size) {
        assert(usb[pos++] == 0u);
        const size_t start = pos - 1u;
        while (pos < usb_size && usb[pos] != 0u) { ++pos; }
        assert(pos < usb_size); ++pos;
        pil_gate1_record_t r;
        assert(pil_gate1_dump_decode(usb + start, pos - start, &r));
        assert(r.sequence == records++ && r.nonce == 123u && r.session == 456u);
        if (r.kind == 5u) {
            assert(pil_get_u32(r.payload + 12u) == chain);
            assert(pil_get_u32(r.payload) == records - 1u);
            manifest = true; break;
        }
        assert(pil_gate1_dump_chain(&chain, &r));
    }
    assert(manifest);
    assert(usb_size < sizeof(usb)); usb[usb_size] = 0u;
    const char *tail = (const char *)(usb + pos);
    const char *begin = strstr(tail, "[PILG1RX1 BEGIN ");
    const char *end = strstr(tail, "[PILG1RX1 END ");
    const char *stack = strstr(tail, "stopped; reset required");
    assert(begin != NULL && end != NULL && stack != NULL);
    assert(begin < end && end < stack);
    unsigned int h[8];
    assert(sscanf(begin + strlen("[PILG1RX1 BEGIN "),
        "%x %x %x %x %x %x %x %x", &h[0], &h[1], &h[2], &h[3],
        &h[4], &h[5], &h[6], &h[7]) == 8);
    assert(h[0] == 123u && h[1] == s->session && h[2] == s->stop_time);
    assert(h[3] == reason && h[4] <= 64u && h[5] <= 16u && h[6] <= 16u);
    bool post_sensor = false;
    const char *row = begin;
    while ((row = strstr(row, "[PILG1RX1 P ")) != NULL) {
        unsigned int p[10];
        row += strlen("[PILG1RX1 P ");
        assert(sscanf(row, "%x %x %x %x %x %x %x %x %x %x",
            &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7],
            &p[8], &p[9]) == 10);
        if (p[2] != UINT32_MAX) {
            assert(p[1] <= p[2] && p[2] <= p[3] && p[3] <= p[4]);
            assert(p[4] <= p[5]);
        }
        if ((p[8] & 255u) == PIL_SENSOR && p[6] == 1u &&
            p[9] >> 24u == 3u && p[8] >> 16u == reason) {
            post_sensor = true;
        }
    }
    if (missing) { assert(h[5] >= 1u && post_sensor); }
    const size_t frozen_usb_size = usb_size;
    for (unsigned int i = 0u; i < 100u; ++i) { pil_gate1_test_service(); }
    assert(usb_size == frozen_usb_size);
    const uint32_t count = s->counts[G1_STAGED];
    pil_gate1_motor_seen(0, 0, 1600u);
    assert(s->counts[G1_STAGED] == count && s->servo == 1500u);
    if (argc > 2) {
        FILE *out = fopen(argv[2], "wb");
        assert(out != NULL);
        assert(fwrite(usb, 1u, usb_size, out) == usb_size);
        assert(fclose(out) == 0);
    }
    return 0;
}
