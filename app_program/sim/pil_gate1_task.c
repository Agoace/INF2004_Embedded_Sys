/* Explicit SIM Gate1 image only. Kernel IRQ/task ownership; no SDK headers. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include <string.h>
#include "pil_gate1_runtime.h"
#include "pil_gate1_session.h"
#include "pil_gate1_stream.h"
#include "pil_gate1_dump.h"
#include "pil_gate1_uart_hw.h"
#include "../platform/drv_time.h"
#include "../platform/boot_id_hw.h"
#include "../platform/app_startup.h"

#if !APP_SIM_GATE1 || !APP_SIM || TM_WIFI_CYW43 || TM_NET_LWIP || TM_MQTT
#error Gate1 runtime requires its isolated radio-free SIM image
#endif

extern UW tm_usb_state(void);
extern UW tm_usb_pending(void);
extern void tm_snd_dat(const UB *buf, INT size);
typedef struct { pil_frame_t frame; bool active; } output_work_t;
static struct {
    pil_gate1_session_t session;
    pil_gate1_rx_t rx;
    pil_gate1_queue_t tx;
    output_work_t output[8];
    pil_frame_t irq_frame;
    volatile uint32_t output_head, output_tail;
    uint32_t tx_offset, demand, last_boot, freeze_time;
    uint32_t dump_index, dump_chain;
    ID worker;
    bool initialized, live, final_sent, dump_complete;
} g;
static uint64_t worker_stack[256];
/* Include the sole extra word in pil_gate1_line_time.c. */
_Static_assert(sizeof(g) + sizeof(worker_stack) + sizeof(uint32_t) <= 16384u,
               "Gate1 total static storage exceeds approved16KiB");

static uint32_t real_clock(void *context)
{
    (void)context; return drv_time_us();
}
static void wake_worker(void)
{
    if (g.worker > 0) { (void)tk_wup_tsk(g.worker); }
}
static void lease_update(void)
{
    pil_gate1_lease_set(g.session.armed_once && g.session.reason == 0u,
                        g.session.lease);
}
static void queue_metric(unsigned int id, uint32_t depth)
{
    pil_gate1_count(&g.session, id == 9u ? G1_RX_MUT : G1_TX_MUT);
    const uint32_t t = drv_time_us();
    pil_gate1_note(&g.session, id, PIL_NO_ACK, t, t, 0u, depth);
}
static void queue_failure(unsigned int counter, uint32_t reason)
{
    if (!g.session.frozen) { g.session.coverage |= 1u << 8u; }
    pil_gate1_count(&g.session, counter);
    pil_gate1_fail(&g.session, reason);
}
void pil_gate1_native_grid(uint32_t at_us)
{
    if (!g.initialized) { return; }
    g.demand = at_us;
    pil_gate1_grid(&g.session, at_us);
}
void pil_gate1_native_alarm(void)
{
    if (!g.live || !g.session.grid_ready) { return; }
    UINT irq; DI(irq);
    if (pil_gate1_alarm(&g.session, g.demand, pil_gate1_native_entry(),
                        &g.irq_frame)) {
        if (g.output_head - g.output_tail == 8u) {
            pil_gate1_fail(&g.session, 0x10fu);
        } else {
            output_work_t *w = &g.output[g.output_head & 7u];
            w->frame = g.irq_frame;
            w->active = g.irq_frame.payload[7] == 1u;
            ++g.output_head;
        }
        wake_worker();
    }
    EI(irq);
}
void pil_gate1_motor_seen(int16_t left, int16_t right, uint16_t servo)
{
    if (!g.initialized) { return; }
    UINT irq; DI(irq);
    pil_gate1_motor(&g.session, left, right, servo, drv_time_us());
    EI(irq);
}
void pil_gate1_motor_stopped(void)
{
    if (!g.initialized) { return; }
    UINT irq; DI(irq);
    if (!g.session.frozen) { g.session.write_time = drv_time_us(); }
    EI(irq);
}
static void uart_irq(UINT intno)
{
    UINT irq; DI(irq);
    const uint32_t start = drv_time_us();
    const uint32_t errors = pil_gate1_uart_errors();
    pil_gate1_uart_ack();
    if ((errors & 8u) != 0u) {
        pil_gate1_count(&g.session, G1_OVERRUN);
        pil_gate1_fail(&g.session, 0x109u); wake_worker();
    }
    if ((errors & 1u) != 0u) { pil_gate1_count(&g.session, G1_FRAMING); }
    if ((errors & 2u) != 0u) { pil_gate1_count(&g.session, G1_PARITY); }
    if ((errors & 4u) != 0u) { pil_gate1_count(&g.session, G1_BREAK); }
    for (unsigned int i = 0u; i < 32u; ++i) {
        uint8_t byte; uint32_t flags;
        if (!pil_gate1_uart_rx(&byte, &flags)) { break; }
        const int result = pil_gate1_rx_byte(&g.rx, byte, drv_time_us(), flags);
        if (result == PIL_G1_RX_PACKET) {
            queue_metric(9u, pil_gate1_queue_depth(&g.rx.queue)); wake_worker();
        } else if (result == PIL_G1_RX_FULL) {
            queue_failure(G1_RX_FULL, 0x107u); wake_worker();
        } else if (result == PIL_G1_RX_OVERSIZE) {
            pil_gate1_count(&g.session, G1_BAD);
        }
    }
    for (unsigned int i = 0u; i < 32u; ++i) {
        const pil_gate1_packet_t *p = pil_gate1_queue_front(&g.tx);
        if (p == NULL) { pil_gate1_uart_tx_enable(false); break; }
        if (!pil_gate1_uart_tx(p->bytes[g.tx_offset])) { break; }
        if (++g.tx_offset == p->length) {
            if (p->tag != PIL_NO_ACK) { pil_gate1_count(&g.session, G1_FIFO); }
            (void)pil_gate1_queue_drop(&g.tx); g.tx_offset = 0u;
            queue_metric(10u, pil_gate1_queue_depth(&g.tx)); wake_worker();
        }
    }
    ClearInt(intno);
    pil_gate1_note(&g.session, 11u, PIL_NO_ACK,
                   start, drv_time_us(), 0u, 0u);
    EI(irq);
}
static void lease_irq(UINT intno)
{
    UINT irq; DI(irq);
    pil_gate1_lease_ack();
    pil_gate1_check_lease(&g.session, drv_time_us());
    lease_update(); ClearInt(intno); wake_worker();
    EI(irq);
}
static bool transmit(const pil_frame_t *f, bool active)
{
    pil_gate1_packet_t packet = {.tag = active ? f->ack : PIL_NO_ACK};
    size_t length;
    if (pil_encode(f, packet.bytes, sizeof(packet.bytes), &length) != PIL_OK) {
        UINT irq; DI(irq); pil_gate1_fail(&g.session, 0x10cu); EI(irq);
        return false;
    }
    packet.length = (uint32_t)length;
    UINT irq; DI(irq);
    const bool ok = pil_gate1_queue_put(&g.tx, &packet);
    if (ok) {
        queue_metric(10u, pil_gate1_queue_depth(&g.tx));
        if (active) { pil_gate1_count(&g.session, G1_ENQUEUED); }
        pil_gate1_uart_tx_enable(true);
    } else { queue_failure(G1_TX_FULL, 0x108u); }
    EI(irq);
    return ok;
}
static void dump_one(void)
{
    if (g.dump_complete || tm_usb_state() != 3u || tm_usb_pending() > 512u) {
        return;
    }
    const uint32_t traces = g.session.trace_total < 64u ?
                            g.session.trace_total : 64u;
    const uint32_t last = 14u + traces;
    if (g.dump_index > last) {
        g.dump_complete = true;
        uint32_t free_bytes = 0u;
        const UB *paint = (const UB *)worker_stack;
        while (free_bytes < sizeof(worker_stack) &&
               paint[free_bytes] == 0xa5u) { ++free_bytes; }
        tm_printf((UB *)"[PILG1] stopped; reset required; "
                        "worker stack untouched %lu/2048 bytes\n",
                  (unsigned long)free_bytes);
        return;
    }
    pil_gate1_record_t record = {.sequence = g.dump_index,
        .nonce = g.session.nonce, .session = g.session.session};
    uint32_t words[26] = {0};
    if (g.dump_index == 0u) {
        record.kind = 1u; record.length = 104u;
        const uint32_t c[26] = {PIL_SCHEMA_ID, 4u, g.session.epoch,
            g.session.grid_anchor, 125000000u, 921600u, 8u, 31u,
            1000u, 4000u, 5000u, 8u,
            g.session.next_stage ? 0u : PIL_NO_ACK,
            g.session.next_stage ? g.session.next_stage - 1u : PIL_NO_ACK,
            g.session.counts[G1_APPLIED] ? 0u : PIL_NO_ACK,
            g.session.counts[G1_APPLIED] ?
                g.session.counts[G1_APPLIED] - 1u : PIL_NO_ACK,
            g.session.counts[G1_DEMANDED] ? 0u : PIL_NO_ACK,
            g.session.last_demand, g.session.run_start, g.freeze_time,
            g.session.reason, g.session.flags, g.session.coverage,
            0x79u | (TK_SUPPORT_SMP ? 2u : 0u) |
                (APP_FORCE_CORE0 ? 4u : 0u), 12u, traces};
        memcpy(words, c, sizeof(c));
    } else if (g.dump_index == 1u) {
        record.kind = 2u; record.length = 104u;
        memcpy(words, g.session.counts, sizeof(words));
    } else if (g.dump_index < 14u) {
        record.kind = 3u; record.length = 48u;
        (void)pil_gate1_metric_payload(&g.session.metrics[g.dump_index - 2u],
                                       record.payload);
    } else if (g.dump_index < last) {
        record.kind = 4u; record.length = 64u;
        pil_gate1_trace_get(&g.session, g.dump_index - 14u, words);
    } else {
        record.kind = 5u; record.length = 16u;
        words[0] = last; words[1] = 12u;
        words[2] = traces; words[3] = g.dump_chain;
    }
    if (record.kind != 3u) {
        for (unsigned int i = 0u; i < record.length / 4u; ++i) {
            pil_put_u32(record.payload + i * 4u, words[i]);
        }
    }
    uint8_t wire[PIL_GATE1_D1_WIRE]; size_t length;
    if (!pil_gate1_dump_encode(&record, wire, sizeof(wire), &length)) {
        return;
    }
    if (record.kind != 5u) {
        (void)pil_gate1_dump_chain(&g.dump_chain, &record);
    }
    tm_snd_dat(wire, (INT)length); ++g.dump_index;
}
static void service(void)
{
    const uint32_t start = drv_time_us();
    for (unsigned int i = 0u; i < 8u; ++i) {
        pil_frame_t incoming, reply;
        const pil_gate1_packet_t *packet = pil_gate1_queue_front(&g.rx.queue);
        if (packet == NULL) { break; }
        const uint32_t stamp = packet->stamp, errors = packet->errors;
        const int decoded = pil_decode(packet->bytes, packet->length,
                                        &incoming);
        UINT irq; DI(irq);
        (void)pil_gate1_queue_drop(&g.rx.queue);
        queue_metric(9u, pil_gate1_queue_depth(&g.rx.queue));
        bool send = false;
        if (decoded != PIL_OK || errors != 0u) {
            pil_gate1_count(&g.session, G1_BAD);
        } else {
            send = pil_gate1_receive(&g.session, &incoming, stamp, &reply);
        }
        lease_update(); EI(irq);
        if (send) { (void)transmit(&reply, false); }
    }
    for (unsigned int i = 0u; i < 8u; ++i) {
        output_work_t w;
        UINT irq; DI(irq);
        const bool available = g.output_tail != g.output_head;
        if (available) {
            w = g.output[g.output_tail & 7u]; ++g.output_tail;
        }
        EI(irq);
        if (!available) { break; }
        (void)transmit(&w.frame, w.active);
    }
    if (!g.session.armed_once && g.session.reason == 0u &&
        drv_time_us() - g.last_boot >= 100000u) {
        pil_frame_t boot;
        UINT irq; DI(irq);
        pil_gate1_boot(&g.session, &boot); g.last_boot = boot.source_us;
        EI(irq); (void)transmit(&boot, false);
    }
    if (g.session.reason != 0u && !g.final_sent) {
        g.final_sent = true;
        if (g.session.session != 0u) {
            pil_frame_t final;
            UINT irq; DI(irq); pil_gate1_output(&g.session, &final); EI(irq);
            (void)transmit(&final, false);
        }
    }
    UINT irq; DI(irq);
    pil_gate1_note(&g.session, 12u, PIL_NO_ACK,
                   start, drv_time_us(), 0u, 0u);
    if (g.session.reason != 0u && !g.session.frozen) {
        const bool drained = g.output_head == g.output_tail &&
            pil_gate1_queue_depth(&g.tx) == 0u && pil_gate1_uart_idle();
        if (drained || drv_time_us() - g.session.stop_time >= 100000u) {
            g.freeze_time = drv_time_us();
            if (drained) { g.session.coverage |= 16u; }
            g.session.coverage |= 1024u;
            g.session.frozen = true;
        }
    }
    EI(irq);
    if (g.session.frozen) { dump_one(); }
}
static void worker(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    while (!app_startup_complete() || !g.session.grid_ready) {
        (void)tk_slp_tsk(1);
    }
    DisableInt(20u); DisableInt(1u);
    const T_DINT uart = {.intatr = TA_HLNG, .inthdr = (FP)uart_irq};
    const T_DINT lease = {.intatr = TA_HLNG, .inthdr = (FP)lease_irq};
    if (tk_def_int(20u, &uart) < E_OK || tk_def_int(1u, &lease) < E_OK ||
        !pil_gate1_uart_init((uint32_t)CLK_PERI_FREQ)) {
        pil_gate1_uart_disable(); pil_gate1_fail(&g.session, 0x10cu);
        tm_printf((UB *)"[PILG1] UART/IRQ init failed; inhibited\n");
    } else {
        ClearInt(20u); ClearInt(1u); g.live = true;
        EnableInt(20u, 1); EnableInt(1u, 1);
        tm_printf((UB *)"[PILG1] transport-only cap4 UART0 921600 8N1; "
                        "no physical actuators/radio\n");
    }
    for (;;) { service(); (void)tk_slp_tsk(1); }
}
extern INT __real_usermain(void);
INT __wrap_usermain(void)
{
    uint32_t nonce = 0u;
    if (drv_time_init() < E_OK || !boot_id_hw_generate(&nonce)) {
        tm_printf((UB *)"[PILG1] no boot identity/time; inhibited\n");
        return __real_usermain();
    }
    pil_gate1_session_init(&g.session, nonce, real_clock, NULL);
    pil_gate1_rx_init(&g.rx); g.initialized = true;
    memset(worker_stack, 0xa5, sizeof(worker_stack));
    T_CTSK c = {.task = (FP)worker, .itskpri = 16,
        .tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF,
        .stksz = (SZ)sizeof(worker_stack), .bufptr = worker_stack};
#if TK_SUPPORT_SMP
    c.tskatr |= TA_ASSPRC; c.assprc = TP_PRC1;
#endif
    g.worker = tk_cre_tsk(&c);
    if (g.worker <= 0 || tk_sta_tsk(g.worker, 0) < E_OK) {
        if (g.worker > 0) { (void)tk_del_tsk(g.worker); }
        g.worker = 0; pil_gate1_fail(&g.session, 0x10cu);
        tm_printf((UB *)"[PILG1] worker creation failed; inhibited\n");
    }
    return __real_usermain();
}
#ifdef PIL_GATE1_RUNTIME_TEST
pil_gate1_session_t *pil_gate1_test_session(void) { return &g.session; }
void pil_gate1_test_service(void) { service(); }
#endif
