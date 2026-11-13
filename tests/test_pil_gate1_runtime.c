/* Real runtime/session/stream with fake kernel/MMIO, never serial access. */
#include <tk/tkernel.h>
#include "pil_gate1_runtime.h"
#include "pil_gate1_session.h"
#include "pil_gate1_uart_hw.h"
#include "pil_gate1_dump.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include <stdio.h>

extern INT __wrap_usermain(void);
extern pil_gate1_session_t *pil_gate1_test_session(void);
extern void pil_gate1_test_service(void);
static FP task;
static void (*handlers[32])(UINT);
static jmp_buf sleep_exit;
static uint32_t now = 500000u, deadline;
static bool tx_enabled;
static uint32_t uart_errors;
static uint8_t rx[256], uart[32768], usb[32768];
static size_t rx_at, rx_size, uart_size, usb_size;
ID tk_cre_tsk(const T_CTSK *c)
{
    assert(c->itskpri == 16 && c->stksz == 2048);
    assert(c->bufptr != NULL && (c->tskatr & TA_USERBUF) != 0u);
#if TK_SUPPORT_SMP
    assert((c->tskatr & TA_ASSPRC) != 0u && c->assprc == TP_PRC1);
#endif
    task = c->task; return 9;
}
ER tk_sta_tsk(ID id, INT code) { assert(id == 9 && code == 0); return 0; }
ER tk_del_tsk(ID id) { (void)id; return 0; }
ER tk_wup_tsk(ID id) { assert(id == 9); return 0; }
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
    pil_gate1_native_grid(now + 1000u);
    pil_gate1_motor_seen(0, 0, 1500u); return 0;
}
UW tm_usb_state(void) { return 3u; }
UW tm_usb_pending(void) { return 0u; }
void tm_printf(UB *fmt, ...) { (void)fmt; }
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
    assert(uart_size < sizeof(uart)); uart[uart_size++] = byte; return true;
}
void pil_gate1_uart_tx_enable(bool enabled) { tx_enabled = enabled; }
uint32_t pil_gate1_uart_errors(void)
{
    const uint32_t result = uart_errors; uart_errors = 0u; return result;
}
void pil_gate1_uart_ack(void) {}
bool pil_gate1_uart_idle(void) { return !tx_enabled; }
void pil_gate1_uart_disable(void) {}
void pil_gate1_lease_ack(void) {}
void pil_gate1_lease_set(bool enabled, uint32_t value)
{
    deadline = enabled ? value : 0u;
}
static void drain(void)
{
    for (unsigned int i = 0u; tx_enabled && i < 100u; ++i) {
        handlers[20](20u);
    }
    assert(!tx_enabled);
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
    const bool missing = strcmp(mode, "missing") == 0;
    const uint32_t applied = missing ? 0u : 8u;
    const uint32_t staged = missing ? 1u : 11u;
    assert(__wrap_usermain() == 0 && task != NULL);
    if (setjmp(sleep_exit) == 0) {
        ((void (*)(INT, void *))task)(0, NULL);
    }
    drain();
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
    for (unsigned int i = 0u; i < 40u; ++i) {
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
    size_t pos = 0u;
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
