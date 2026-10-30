/*
 * Steering log packets for Wi-Fi (UDP broadcast). See docs/STEER_WIFI_LOG.md.
 *
 * Every packet starts with a 5-byte header:
 *
 *   byte 0     bits 0-3 type (SL_TYPE_*), bits 4-7 format version
 *   bytes 1-2  boot id, little-endian (random at each start)
 *   bytes 3-4  packet sequence number, little-endian (finds lost packets)
 *
 * TEXT packets: the header, then text (the firmware's printf output).
 *
 * HELLO packets: only the header, sent by the receiver once a second to
 * the broadcast address. The robot then sends to the receiver's address
 * (unicast): Wi-Fi repeats lost unicast frames, but not broadcast frames.
 *
 * LOG packets: the header, then a 5-byte log header, then up to
 * SL_MAX_ENTRIES entries of 11 bytes:
 *
 *   byte 5     run number (counts runs since the start)
 *   byte 6     flags: bit 0 replay, bit 1 last replay packet
 *   bytes 7-8  index of the first entry in the run, little-endian
 *              (entry time = index * 50 ms)
 *   byte 9     number of entries
 *
 * Entry: 88 bits, least significant bit first (Change Requirement 1:
 * each field has only the bits that it needs):
 *
 *   bits  0- 7  position, two's complement, -100..+100
 *   bits  8-15  left duty %, two's complement (negative = reverse)
 *   bits 16-23  right duty %, two's complement
 *   bits 24-31  info: bits 0-1 mode, 2-3 side, 4-5 side source, 6-7 hint
 *   bits 32-43  raw left ADC, 12 bits
 *   bits 44-55  raw centre ADC
 *   bits 56-67  raw right ADC
 *   bits 68-77  left wheel counts in the 50 ms, two's complement, +/-511
 *   bits 78-87  right wheel counts
 *
 * Log header flag bit 2 (SL_FLAG_BYPASS): the run is from bypass_test, so
 * mode 3 is BYPS (obstacle stop, scan or bypass) instead of CALM.
 *
 * BYPASS packets (bypass_test): the header, then a 4-byte bypass header
 * (run number; index of the first record, little-endian; number of
 * records), then up to SL_MAX_BYPASS records of 5 bytes, one per
 * ultrasonic reading of the bypass (src/obstacle_bypass.h):
 *
 *   bits  0-12  time since the run start, in 10 ms (to 81.91 s)
 *   bits 13-18  servo angle of the reading + 30, degrees (0-60)
 *   bits 19-29  reading: ultrasonic result bits 0-10 (distance, status)
 *   bits 30-31  bypass state after the reading (bp_state_t)
 *   bits 32-37  steering angle after the reading + 30, degrees (0-60)
 *   bits 38-39  spare, 0
 *
 * Pure logic, no hardware: host-tested by tests/test_steer_log.c. The
 * host decoder is scripts/steer_log_receiver.py.
 */
#ifndef STEER_LOG_H
#define STEER_LOG_H

#include <stddef.h>
#include <stdint.h>

#define SL_VERSION 1u
#define SL_PORT 4243u

#define SL_TYPE_LOG 1u
#define SL_TYPE_TEXT 2u
#define SL_TYPE_HELLO 3u
#define SL_TYPE_BYPASS 4u

#define SL_HEADER_BYTES 5u
#define SL_LOG_HEADER_BYTES 5u
#define SL_ENTRY_BYTES 11u
#define SL_MAX_ENTRIES 20u
#define SL_MAX_PACKET_BYTES \
    (SL_HEADER_BYTES + SL_LOG_HEADER_BYTES + SL_MAX_ENTRIES * SL_ENTRY_BYTES)

#define SL_FLAG_REPLAY 0x01u
#define SL_FLAG_REPLAY_END 0x02u
#define SL_FLAG_BYPASS 0x04u

#define SL_BYPASS_HEADER_BYTES 4u
#define SL_BYPASS_BYTES 5u
#define SL_MAX_BYPASS 40u
#define SL_BYPASS_TIME_MAX 8191u /* 10 ms units */

#define SL_RAW_MASK 0x0FFFu
#define SL_COUNTS_MAX 511 /* wheel counts are clamped to +/-511 */

typedef struct {
    int8_t position;
    int8_t left_pct;
    int8_t right_pct;
    uint8_t info;
    uint16_t raw[3];
    int16_t left_counts;
    int16_t right_counts;
} sl_entry_t;

typedef struct {
    uint16_t time_10ms;  /* clamped to SL_BYPASS_TIME_MAX */
    int8_t servo_deg;    /* -30..+30 */
    uint16_t reading;    /* ultrasonic_result_t; bits 0-10 are kept */
    uint8_t state;       /* bp_state_t */
    int8_t steer_deg;    /* -30..+30 */
} sl_bypass_t;

void sl_pack_bypass(const sl_bypass_t *b, uint8_t out[SL_BYPASS_BYTES]);
void sl_unpack_bypass(const uint8_t in[SL_BYPASS_BYTES], sl_bypass_t *b);

/* Write the 4-byte bypass header. Returns SL_BYPASS_HEADER_BYTES. */
size_t sl_write_bypass_header(uint8_t *out, uint8_t run, uint16_t first, uint8_t count);

void sl_pack_entry(const sl_entry_t *e, uint8_t out[SL_ENTRY_BYTES]);
void sl_unpack_entry(const uint8_t in[SL_ENTRY_BYTES], sl_entry_t *e);

/* Write the 5-byte packet header. Returns SL_HEADER_BYTES. */
size_t sl_write_header(uint8_t *out, unsigned int type, uint16_t boot_id, uint16_t seq);

/* Write the 5-byte log header. Returns SL_LOG_HEADER_BYTES. */
size_t sl_write_log_header(uint8_t *out, uint8_t run, uint8_t flags, uint16_t first,
                           uint8_t count);

#endif
