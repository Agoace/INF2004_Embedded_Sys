/*
 * IMU safety latch capture (opt-in IMU_SAFETY_CAPTURE=1 ROBOT profile).
 *
 * A RAM ring of detector observations. While ARMED it keeps the newest
 * ISC_SAMPLES. The first latch edge (latched trigger NONE -> any) starts
 * COLLECTING: up to ISC_PRE samples before it, the edge sample and
 * ISC_POST - 1 more. A Vehicle ESTOP request (isc_request) without a
 * latch does the same with ISC_MANUAL_PRE / ISC_MANUAL_POST, so a gentle
 * lift that does not latch can be captured by a GP21 stop. Then the ring
 * is FROZEN until a reader prints it and calls isc_rearm(). The writer
 * then resets on its next sample. One writer (IMU task), one requester
 * (Vehicle task), one reader (telemetry dump). The reader reads samples
 * only while FROZEN, when the writer never writes them. Never blocks,
 * allocates or prints.
 */
#ifndef IMU_SAFETY_CAPTURE_H
#define IMU_SAFETY_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include "../subsystem4/imu_safety.h"

#define ISC_PRE     64u
#define ISC_POST    192u
#define ISC_SAMPLES (ISC_PRE + ISC_POST)
#define ISC_MANUAL_POST 64u
#define ISC_MANUAL_PRE  (ISC_SAMPLES - ISC_MANUAL_POST)

/* What started the capture. */
#define ISC_SOURCE_LATCH 1u
#define ISC_SOURCE_ESTOP 2u

enum {
    ISC_ARMED = 0,
    ISC_COLLECTING = 1,
    ISC_FROZEN = 2,
    ISC_REARM = 3
};

/* Input bits of one observation. */
#define ISC_IN_VALID        0x01u
#define ISC_IN_CALIBRATED   0x02u
#define ISC_IN_WHEELS_STILL 0x04u
#define ISC_IN_READ_FAILED  0x08u

/* 20 bytes. Values clamp to their type; non-finite gives 0. */
typedef struct {
    uint16_t ms16;  /* now_ms, low 16 bits */
    int16_t a[3];   /* input accel mg, X right, Y forward, Z up */
    uint16_t g;     /* output |a| mg */
    int16_t tilt;   /* output tilt ddeg (0 inside the upright bounds) */
    int16_t vert;   /* output vertical change mg */
    uint8_t flags;  /* IMU_SAFETY_FLAG_* */
    uint8_t ff;     /* freefall run, saturating */
    uint8_t lift;   /* vertical-change run, saturating */
    uint8_t ot;     /* overturn run, saturating */
    uint8_t trig;   /* current << 4 | latched */
    uint8_t in;     /* ISC_IN_* */
} isc_sample_t;

typedef struct {
    isc_sample_t ring[ISC_SAMPLES];
    uint32_t latch_ms;  /* now_ms of the edge sample */
    uint32_t latches;   /* latch edges seen, saturating */
    uint32_t missed;    /* edges while COLLECTING or FROZEN, saturating */
    uint32_t captures;  /* completed captures, saturating */
    uint16_t head;      /* next write index */
    uint16_t count;     /* retained samples */
    uint16_t post_left; /* samples still to collect */
    uint16_t pre;       /* samples before the edge sample */
    uint32_t requests;  /* ESTOP requests, written by the requester only */
    uint32_t requests_seen; /* writer copy of requests */
    uint8_t source;     /* ISC_SOURCE_* of the capture */
    uint8_t trigger;    /* latched trigger at the edge (0 for ESTOP) */
    uint8_t state;      /* ISC_*, atomic */
} isc_t;

void isc_init(isc_t *c);

/* Writer: after imu_safety_step(); latched_before is the latched trigger
 * read before that step. */
void isc_observe(isc_t *c, const imu_safety_input_t *in,
                 const imu_safety_t *after, uint8_t latched_before);

/* Requester (Vehicle task, one task only): capture around this moment if
 * ARMED. A latch already collecting or frozen wins; the request is then
 * dropped. */
void isc_request(isc_t *c);

uint8_t isc_state(const isc_t *c);
bool isc_frozen(const isc_t *c);
unsigned int isc_count(const isc_t *c);
/* Oldest first; NULL past the end. */
const isc_sample_t *isc_at(const isc_t *c, unsigned int index);
/* Sample time relative to the edge, ms (16-bit wrap safe). */
int32_t isc_rel_ms(const isc_t *c, const isc_sample_t *s);
/* Reader: FROZEN -> REARM. Ignored in any other state. */
void isc_rearm(isc_t *c);

#endif /* IMU_SAFETY_CAPTURE_H */
