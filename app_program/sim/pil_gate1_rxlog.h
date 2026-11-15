/* Stopped-only RX timing diagnostics. Caller excludes core0 IRQ access. */
#ifndef PIL_GATE1_RXLOG_H
#define PIL_GATE1_RXLOG_H
#include "pil_wire.h"
#define PIL_RXLOG_PRE 64u
#define PIL_RXLOG_POST 16u
#define PIL_RXLOG_ALARMS 16u
#define PIL_RXLOG_NONE UINT32_MAX

typedef struct {
    uint32_t packet[PIL_RXLOG_PRE + PIL_RXLOG_POST][10];
    uint32_t alarm[PIL_RXLOG_ALARMS][6];
    uint32_t pre_total, post_count, alarm_total, serial, post_lost;
    uint32_t fault_us, reason, printed, crc;
    bool faulted, sealed;
} pil_gate1_rxlog_t;
_Static_assert(sizeof(pil_gate1_rxlog_t) < 4096u, "RX log RAM budget");

uint32_t pil_gate1_rxlog_tag(const pil_gate1_rxlog_t *d);
void pil_gate1_rxlog_rx(pil_gate1_rxlog_t *d, uint32_t stamp,
                        uint32_t length, uint32_t depth, uint32_t result);
void pil_gate1_rxlog_worker(pil_gate1_rxlog_t *d, uint32_t tag,
                            uint32_t stamp, uint32_t depth);
void pil_gate1_rxlog_decode(pil_gate1_rxlog_t *d, uint32_t tag,
                            uint32_t stamp, int result,
                            const pil_frame_t *frame);
void pil_gate1_rxlog_stage(pil_gate1_rxlog_t *d, uint32_t tag,
                           uint32_t stamp, uint32_t reason, bool done,
                           uint32_t depth);
void pil_gate1_rxlog_fault(void *context, uint32_t stamp, uint32_t reason);
uint32_t pil_gate1_rxlog_alarm(pil_gate1_rxlog_t *d, uint32_t demand,
                               uint32_t entry, uint32_t sequence,
                               uint32_t ready);
void pil_gate1_rxlog_alarm_done(pil_gate1_rxlog_t *d, uint32_t tag,
                                uint32_t reason, uint32_t stamp);
void pil_gate1_rxlog_seal(pil_gate1_rxlog_t *d);
bool pil_gate1_rxlog_print(pil_gate1_rxlog_t *d, uint32_t nonce,
                           uint32_t session);
#endif
