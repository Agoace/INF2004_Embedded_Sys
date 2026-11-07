/*
 * boot_id.h - one nonzero observation identity for this boot.
 *
 * Initialize before application tasks and the watchdog start. The identity
 * is a random label, not a persistent counter or authentication secret.
 * Entropy failure leaves zero; the network observer must then remain off.
 */
#ifndef BOOT_ID_H
#define BOOT_ID_H

#include <stdint.h>

/* E_OK on success; E_SYS if bounded entropy collection failed. Idempotent. */
int boot_id_init(void);

/* Zero until successful initialization; fixed for the rest of this boot. */
uint32_t boot_id_get(void);

#endif
