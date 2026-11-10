/* Local B2b source ABI. S3 owns record writes; vehicle owns control writes. */
#ifndef BARCODE_LIVE_SOURCE_H
#define BARCODE_LIVE_SOURCE_H
#include <stdint.h>

#define BC_LIVE_VERSION 1u
#define BC_LIVE_SOURCE 2u
#define BC_LIVE_RECORD_MAX_AGE_MS 50u
#define BC_LIVE_CONTROL_MAX_AGE_MS 50u
#define BC_LIVE_DISABLED 0u
#define BC_LIVE_SHADOW_MODE 1u
#define BC_LIVE_PERMITTED 2u
#define BC_LIVE_DISTANCE 0x01u
#define BC_LIVE_WINDOW29 0x02u
#define BC_LIVE_LEADING_WHITE 0x04u
#define BC_LIVE_TAIL_WHITE 0x08u
#define BC_LIVE_LOSS_FREE 0x10u
#define BC_LIVE_MAP_CONFIRMED 0x20u
#define BC_LIVE_OFFSET_VERIFIED 0x40u
#define BC_LIVE_SHADOW 0x80u
#define BC_LIVE_REQUIRED 0x7fu

typedef struct {
    uint32_t epoch;
    uint32_t sequence;
    uint32_t read_ms;
    int32_t read_counts[2];
    uint8_t command; /* 0 NONE, 1 LEFT, 2 RIGHT, 3 STRAIGHT, 4 UTURN. */
    uint8_t valid; /* Zero in shadow/unverified/unmapped observations. */
    uint8_t source; /* 2; fake source 1 remains a separate backend. */
    uint8_t reserved; /* Zero. */
    int32_t frame_end_counts[2];
    uint32_t eligibility_revision; /* Shared revision, at FIRST BAR START. */
    uint8_t letter;
    uint8_t direction; /* 1 print order, 2 reverse print; forward motion. */
    uint8_t flags;
    uint8_t version;
} barcode_live_record_t;

/* FINAL 20B control, vehicle SOLE writer, publish every RUN tick unchanged
 * or not. One revision advances on CONFIRMED junction and route boundaries
 * (including SEARCH entry/return); revision_ms retains wrap-safe maximum of
 * original first-edge/route cut times. Source captures at FIRST BAR START,
 * rejects start<=cut and any changed epoch/revision through publication.
 * ACK is terminal retirement/consumer ownership, NOT completed execution.
 * mode0 (BUSY/disabled) must precede or share the accepting ACK publication.
 * mode2 requires measured/approved offset; shadow/unverified stays valid0.
 * First source read or scheduling pause: require publication PROGRESS before
 * trusting eligibility, then fail closed after50ms without progress. Log max
 * ages in shadow. A coherent tag alone is NOT an absolute wall-clock stamp. */
typedef struct {
    uint32_t epoch;
    uint32_t eligibility_revision;
    uint32_t revision_ms;
    uint32_t ack_sequence;
    uint32_t mode;
} barcode_live_control_t;

_Static_assert(sizeof(barcode_live_record_t) == 40u, "Live record ABI");
_Static_assert(__builtin_offsetof(barcode_live_record_t, read_counts) == 12u,
               "Live coherent counts offset");
_Static_assert(__builtin_offsetof(barcode_live_record_t, command) == 20u,
               "Live legacy prefix offset");
_Static_assert(__builtin_offsetof(barcode_live_record_t, frame_end_counts) ==
               24u, "Live frame-end offset");
_Static_assert(__builtin_offsetof(barcode_live_record_t,
                                 eligibility_revision) == 32u,
               "Live eligibility revision offset");
_Static_assert(__builtin_offsetof(barcode_live_record_t, letter) == 36u,
               "Live metadata offset");
_Static_assert(__builtin_offsetof(barcode_live_record_t, version) == 39u,
               "Live version offset");
_Static_assert(sizeof(barcode_live_control_t) == 20u, "Live control ABI");

/* S3 startup only, before tasks. Idempotent; does not reset live state.
 * Does NOT write the vehicle control board. No reader may reset either board.
 * Record read: E_OBJ before init; E_PAR null; E_TMOUT on interrupted copy;
 * E_OK on success. Failed copies leave output unchanged. No wait/consume.
 * Source remains an INVALID shadow observation until producer integration. */
int barcode_live_source_init(void);
int barcode_live_source_try_read(barcode_live_record_t *out);
/* Vehicle is the sole caller/writer. S3 never calls this to reset control. */
int barcode_live_control_publish(const barcode_live_control_t *control);
/* S3 read; optional publication tag is unchanged on failure. */
int barcode_live_control_try_read(barcode_live_control_t *out,
                                  uint32_t *publication_tag);
#endif
