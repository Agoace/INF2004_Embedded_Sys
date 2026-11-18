/* SPSC terminal-result FIFO. LINE produces, Wi-Fi consumes/ACKs. No waits. */
#ifndef BARCODE_ATTEMPT_H
#define BARCODE_ATTEMPT_H
#include "../subsystem1/mqtt_wire.h"
#define BC_ATTEMPT_CAPACITY 8u
/* Drop NEWEST when full; old unacknowledged results stay immutable. */
bool barcode_attempt_enqueue(const rm_barcode_t *value);
/* Non-consuming head copy. Only Wi-Fi may call peek/ack. */
bool barcode_attempt_peek(rm_barcode_t *out);
bool barcode_attempt_ack(uint32_t sequence);
/* Lifetime enqueue losses, modulo 2^32. Never reset by readers or epochs. */
uint32_t barcode_attempt_dropped(void);
#endif
