/*
 * Wi-Fi log: send packets over UDP, port SL_PORT (4243). The PC needs no
 * fixed IP address: scripts/steer_log_receiver.py sends a HELLO packet
 * every second, and the robot then sends to its address (unicast, which
 * Wi-Fi repeats if lost). Until then the robot sends to the broadcast
 * address. See docs/STEER_WIFI_LOG.md.
 *
 * Wi-Fi never blocks or stops the robot:
 *   - wifi_log_init() starts the connection and returns; wifi_log_poll()
 *     in the main loop finishes it, and joins again if the link is lost;
 *   - a send without a link is dropped (the flash log is the backup);
 *   - printf output is also kept in a 2 KB buffer and sent as TEXT
 *     packets from wifi_log_poll(); text that does not fit is dropped.
 *
 * The Pico W LED is on while the Wi-Fi link is up.
 *
 * Without src/wifi_config.h the module builds without Wi-Fi: every
 * function does nothing and wifi_log_up() is false.
 *
 * Call these functions from the main loop only, never from an interrupt.
 */
#ifndef WIFI_LOG_H
#define WIFI_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void wifi_log_init(void);
void wifi_log_poll(uint32_t now_ms);
bool wifi_log_up(void);

/* Send one packet: the module adds the 5-byte header (steer_log.h). */
bool wifi_log_send(unsigned int type, const uint8_t *payload, size_t length);

/* Copy printf output to Wi-Fi (default on). Turn it off around large
 * USB-only output. */
void wifi_log_mirror_text(bool on);

/* Packets dropped (no link, no memory, or text buffer full). */
uint32_t wifi_log_dropped(void);

#endif
