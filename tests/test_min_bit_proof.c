/*
 * CR1 minimum-bit proof. One table per stored record. For every field the
 * test MEASURES, from the real pack code, which stored bits change when only
 * that field changes (its footprint), and proves:
 *   - the footprint lies inside the declared slot [offset, offset + width);
 *   - the footprint has exactly ceil(log2 n) bits, n = number of values;
 *   - width == ceil(log2 n), or the row names a reason from a closed list;
 *   - every probe value round-trips through the unpack code;
 *   - listed out-of-range values are rejected with the output unchanged;
 *   - slots do not overlap, and all bits outside the slots stay zero;
 *   - sum(width) + spare == element bits, and the native/packed sizes.
 * Per record it computes the storage totals (current, streamed at current
 * widths, streamed at minimum widths) and compares them with the table, so
 * a layout change cannot pass without a table (and document) change.
 * "--markdown" prints the tables that docs/architecture/CR1_MIN_BIT_PROOF.md
 * must contain; test_min_bit_doc compares the two.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TM_MQTT 1
#include "../app_program/subsystem2/motion_pack.h"
#include "../app_program/vehicle/vehicle_pack.h"
#include "../app_program/vehicle/vehicle_log.h"
#include "../app_program/vehicle/obstacle_pack.h"
#include "../app_program/subsystem5/sonar_pack.h"
#include "../app_program/subsystem3/barcode_evidence_pack.h"
#include "../app_program/subsystem1/telemetry_pack.h"
#include "../app_program/subsystem1/network_pack.h"
#include "../app_program/platform/fault_mask.h"
#include "../app_program/subsystem4/imu_hump.h"
#include "../app_program/subsystem4/imu_hump_diag.h"
#include "../app_program/subsystem4/imu_safety_board.h"
#include "../app_program/platform/fault_pack.h"

#define MAX_ELEM_BYTES 128u
#define MAX_FIELDS 48u

/* Closed reason list for width > minimum. */
/* Element layout in RAM. A bit stream over elements is possible only for
 * one plain array; separate allocations (boards, struct members) are not. */
#define L_SEPARATE 0 /* separate allocations (boards, copies) */
#define L_ARRAY 1    /* one plain array of elements */
#define L_STREAMED 2 /* elements already packed back to back */

#define R_NONE 0
#define R_WORD_SLACK 1 /* narrowing saves 0 B in the record total */
#define R_GAP 2        /* report only: real saving, listed in the gap table */
#define R_SMALL_GAIN 3 /* accepted (operator): saving = .accepted bytes */
static const char *const reason_name[] = {"", "WORD_SLACK", "GAP",
                                          "SMALL_GAIN"};
/* Record-level reason: ELEMENT_ALIGNED. An array keeps each element in
 * whole bytes or words for O(1) indexed access; record_t.aligned_bytes
 * states the bytes that one bit stream over the array would save. */

typedef struct {
    const char *name;
    unsigned int off;    /* slot start bit in the element */
    unsigned int width;  /* slot width in bits */
    unsigned int repeat; /* array fields: element count (stride = width) */
    int64_t lo, hi, step;  /* range values lo..hi (step), when list is 0 */
    const int64_t *list;   /* explicit valid values */
    unsigned int list_n;
    uint64_t flags;        /* nonzero: independent bits in this mask */
    const int64_t *bad;    /* values that the pack code must reject */
    unsigned int bad_n;
    int reason;
    unsigned int accepted; /* R_SMALL_GAIN: stated saving in bytes */
    const char *range;     /* text for the "Valid range" column */
    void (*set)(void *native, unsigned int i, int64_t v);
    int64_t (*get)(const void *native, unsigned int i);
} field_t;

typedef struct {
    const char *name;
    const char *source;
    size_t native_size;
    unsigned int elem_bytes; /* stored bytes of one element */
    unsigned int elem_bits;  /* element bits; < elem_bytes*8 if streamed */
    unsigned int count;      /* elements in RAM */
    unsigned int unit;       /* storage unit bits: 8 or 32 */
    int layout;              /* L_SEPARATE, L_ARRAY or L_STREAMED */
    bool (*pack)(uint8_t *out, const void *native);
    bool (*unpack)(void *native, const uint8_t *in);
    const field_t *fields;
    unsigned int nfields;
    unsigned int spare;      /* expected zero bits in one element */
    unsigned int total, streamed, minimum; /* expected bytes (C, Ss, Sm) */
    const char *note;
    unsigned int aligned_bytes; /* ELEMENT_ALIGNED: stated bytes, else 0 */
} record_t;

/* ---- helpers ------------------------------------------------------------ */

static unsigned int ceil_log2(uint64_t n)
{
    unsigned int b = 0u;
    while (b < 64u && (UINT64_C(1) << b) < n) {
        ++b;
    }
    return b;
}

static uint64_t field_n(const field_t *f)
{
    if (f->flags != 0u) {
        return UINT64_C(1) << __builtin_popcountll(f->flags);
    }
    if (f->list != 0) {
        return f->list_n;
    }
    return (uint64_t)((f->hi - f->lo) / f->step) + 1u;
}

static int64_t field_value(const field_t *f, uint64_t k)
{
    if (f->flags != 0u) {
        uint64_t v = 0u;
        uint64_t m = f->flags;
        for (unsigned int b = 0u; m != 0u; ++b, m &= m - 1u) {
            if ((k >> b) & 1u) {
                v |= m & (~m + 1u);
            }
        }
        return (int64_t)v;
    }
    if (f->list != 0) {
        return f->list[k];
    }
    return f->lo + (int64_t)k * f->step;
}

static unsigned int probes(const field_t *f, uint64_t *k)
{
    const uint64_t n = field_n(f);
    unsigned int c = 0u;
    if (n <= 4096u) {
        for (uint64_t i = 0u; i < n; ++i) {
            k[c++] = i;
        }
        return c;
    }
    k[c++] = 0u;
    k[c++] = n - 1u;
    for (unsigned int b = 0u; (UINT64_C(1) << b) < n; ++b) {
        k[c++] = UINT64_C(1) << b;
        k[c++] = (UINT64_C(1) << b) - 1u;
    }
    return c;
}

static bool bit_at(const uint8_t *b, unsigned int i)
{
    return ((b[i / 8u] >> (i % 8u)) & 1u) != 0u;
}

static unsigned int min_bits(const field_t *f)
{
    return ceil_log2(field_n(f));
}

static void base_value(const record_t *r, uint8_t *native)
{
    memset(native, 0, r->native_size);
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        const field_t *f = &r->fields[i];
        for (unsigned int j = 0u; j < (f->repeat ? f->repeat : 1u); ++j) {
            f->set(native, j, field_value(f, 0u));
        }
    }
}

static unsigned int round_up(unsigned int bits, unsigned int unit)
{
    return (bits + unit - 1u) / unit * unit;
}

typedef struct {
    unsigned int used, minimum, total, streamed, smin, aligned;
} totals_t;

static totals_t record_totals(const record_t *r)
{
    totals_t t = {0};
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        const field_t *f = &r->fields[i];
        const unsigned int rep = f->repeat ? f->repeat : 1u;
        t.used += f->width * rep;
        t.minimum += min_bits(f) * rep;
    }
    /* stored: as the code stores it now. aligned: minimum widths, each
     * element alone. streamed/smin: the least bytes the layout allows at
     * the used/minimum widths (one bit stream only for a plain array). */
    t.total = r->layout == L_STREAMED
                  ? round_up(r->count * r->elem_bits, r->unit) / 8u
                  : r->count * r->elem_bytes;
    t.aligned = r->count * (round_up(t.minimum, r->unit) / 8u);
    if (r->layout == L_SEPARATE) {
        t.streamed = r->count * (round_up(t.used, r->unit) / 8u);
        t.smin = t.aligned;
    } else {
        t.streamed = round_up(r->count * t.used, r->unit) / 8u;
        t.smin = round_up(r->count * t.minimum, r->unit) / 8u;
        if (r->layout == L_STREAMED) {
            t.aligned = t.smin;
        }
    }
    return t;
}

/* ---- the proof for one record ------------------------------------------ */

static void prove(const record_t *r)
{
    static uint8_t base[512], native[512], back[512];
    uint8_t p0[MAX_ELEM_BYTES], pk[MAX_ELEM_BYTES], slots[MAX_ELEM_BYTES];
    static uint64_t k[4200];

    assert(r->native_size <= sizeof(base));
    assert(r->elem_bytes <= MAX_ELEM_BYTES);
    assert(r->elem_bits <= r->elem_bytes * 8u);
    memset(slots, 0, sizeof(slots));
    unsigned int used = 0u;
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        const field_t *f = &r->fields[i];
        const unsigned int rep = f->repeat ? f->repeat : 1u;
        for (unsigned int b = 0u; b < f->width * rep; ++b) {
            const unsigned int at = f->off + b;
            assert(at < r->elem_bits);
            if (bit_at(slots, at)) {
                fprintf(stderr, "%s.%s overlaps\n", r->name, f->name);
                assert(false);
            }
            slots[at / 8u] |= (uint8_t)(1u << (at % 8u));
        }
        used += f->width * rep;
        const unsigned int m = min_bits(f);
        if ((f->width == m) != (f->reason == R_NONE) || f->width < m) {
            fprintf(stderr, "%s.%s width %u min %u reason %d\n", r->name,
                    f->name, f->width, m, f->reason);
            assert(false);
        }
    }
    if (used + r->spare != r->elem_bits) {
        fprintf(stderr, "%s: used %u spare %u bits %u\n", r->name, used,
                r->spare, r->elem_bits);
        assert(false);
    }

    base_value(r, base);
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        const field_t *f = &r->fields[i];
        const unsigned int rep = f->repeat ? f->repeat : 1u;
        for (unsigned int j = 0u; j < rep; ++j) {
            uint8_t foot[MAX_ELEM_BYTES] = {0};
            memcpy(native, base, r->native_size);
            f->set(native, j, field_value(f, 0u));
            memset(p0, 0, sizeof(p0));
            assert(r->pack(p0, native));
            const unsigned int nk = probes(f, k);
            for (unsigned int q = 0u; q < nk; ++q) {
                const int64_t v = field_value(f, k[q]);
                memcpy(native, base, r->native_size);
                f->set(native, j, v);
                memset(pk, 0, sizeof(pk));
                if (!r->pack(pk, native)) {
                    fprintf(stderr, "%s.%s[%u] = %lld rejected\n", r->name,
                            f->name, j, (long long)v);
                    assert(false);
                }
                for (unsigned int b = 0u; b < r->elem_bytes; ++b) {
                    foot[b] |= (uint8_t)(p0[b] ^ pk[b]);
                    /* Bits outside every slot are zero. */
                    if ((pk[b] & (uint8_t)~slots[b]) != 0u) {
                        fprintf(stderr, "%s.%s: spare bit set\n", r->name,
                                f->name);
                        assert(false);
                    }
                }
                memset(back, 0, r->native_size);
                assert(r->unpack(back, pk));
                if (f->get(back, j) != v) {
                    fprintf(stderr, "%s.%s[%u]: %lld -> %lld\n", r->name,
                            f->name, j, (long long)v,
                            (long long)f->get(back, j));
                    assert(false);
                }
            }
            unsigned int count = 0u;
            const unsigned int lo = f->off + j * f->width;
            for (unsigned int b = 0u; b < r->elem_bits; ++b) {
                if (bit_at(foot, b)) {
                    ++count;
                    if (b < lo || b >= lo + f->width) {
                        fprintf(stderr, "%s.%s: bit %u outside slot\n",
                                r->name, f->name, b);
                        assert(false);
                    }
                }
            }
            if (count != min_bits(f)) {
                fprintf(stderr, "%s.%s: footprint %u bits, minimum %u\n",
                        r->name, f->name, count, min_bits(f));
                assert(false);
            }
            for (unsigned int q = 0u; q < f->bad_n; ++q) {
                memcpy(native, base, r->native_size);
                f->set(native, j, f->bad[q]);
                memset(pk, 0xA5, sizeof(pk));
                assert(!r->pack(pk, native));
                for (unsigned int b = 0u; b < r->elem_bytes; ++b) {
                    assert(pk[b] == 0xA5u);
                }
            }
        }
    }

    const totals_t t = record_totals(r);
    if (t.total != r->total || t.streamed != r->streamed ||
        t.smin != r->minimum) {
        fprintf(stderr, "%s: totals %u/%u/%u expected %u/%u/%u\n", r->name,
                t.total, t.streamed, t.smin, r->total, r->streamed,
                r->minimum);
        assert(false);
    }
    /* Element alignment: stored bytes above the least bytes at the used
     * widths. Only an array can state it (ELEMENT_ALIGNED), with the
     * exact figure. */
    if (t.total - t.streamed != r->aligned_bytes ||
        (r->aligned_bytes != 0u && r->layout != L_ARRAY)) {
        fprintf(stderr, "%s: element alignment %u B, stated %u B\n",
                r->name, t.total - t.streamed, r->aligned_bytes);
        assert(false);
    }
    /* SMALL_GAIN: the stated bytes equal the record saving; not mixed
     * with GAP, so each figure belongs to one decision. */
    unsigned int small = 0u;
    bool gap = false;
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        small += r->fields[i].reason == R_SMALL_GAIN
                     ? r->fields[i].accepted : 0u;
        gap = gap || r->fields[i].reason == R_GAP;
        if ((r->fields[i].reason == R_SMALL_GAIN) !=
            (r->fields[i].accepted != 0u)) {
            fprintf(stderr, "%s.%s: SMALL_GAIN needs a byte figure\n",
                    r->name, r->fields[i].name);
            assert(false);
        }
    }
    if (small != 0u && (gap || small != t.streamed - t.smin)) {
        fprintf(stderr, "%s: SMALL_GAIN %u B stated, %u B saved\n",
                r->name, small, t.streamed - t.smin);
        assert(false);
    }
    /* WORD_SLACK holds only when the minimum widths save 0 B over the
     * whole record (all elements as one bit stream). GAP and SMALL_GAIN
     * need a saving. */
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        if (r->fields[i].reason == R_WORD_SLACK && t.streamed != t.smin) {
            fprintf(stderr, "%s.%s: WORD_SLACK but %u B can be saved\n",
                    r->name, r->fields[i].name, t.streamed - t.smin);
            assert(false);
        }
        if (r->fields[i].reason == R_GAP && t.streamed == t.smin) {
            fprintf(stderr, "%s.%s: GAP but 0 B can be saved\n", r->name,
                    r->fields[i].name);
            assert(false);
        }
    }
}

/* ---- setters ------------------------------------------------------------ */

#define FIELD_IO(type, id, lval)                                            \
    static void set_##id(void *p, unsigned int i, int64_t x)              \
    {                                                                       \
        type *v = (type *)p;                                                \
        (void)i;                                                            \
        lval = (__typeof__(lval))x;                                         \
    }                                                                       \
    static int64_t get_##id(const void *p, unsigned int i)                 \
    {                                                                       \
        const type *v = (const type *)p;                                    \
        (void)i;                                                            \
        return (int64_t)(lval);                                             \
    }
#define IO(id) set_##id, get_##id

#define RANGE(a, b) .lo = (a), .hi = (b), .step = 1
#define LIST(arr) .list = (arr), .list_n = sizeof(arr) / sizeof((arr)[0])
#define BAD(arr) .bad = (arr), .bad_n = sizeof(arr) / sizeof((arr)[0])
#define U32 RANGE(0, UINT32_MAX)
#define I32 RANGE(INT32_MIN, INT32_MAX)
#define U16 RANGE(0, UINT16_MAX)
#define I16 RANGE(INT16_MIN, INT16_MAX)

#include "min_bit_records.inc"

/* ---- markdown ----------------------------------------------------------- */

static void print_range(const field_t *f)
{
    printf("%s", f->range);
}

static void markdown(const record_t *r)
{
    const totals_t t = record_totals(r);
    printf("### %s\n\n", r->name);
    printf("Source: `%s`. %u element(s) of %u B (%u bits, %u spare).\n\n",
           r->source, r->count, r->elem_bytes, r->elem_bits, r->spare);
    printf("| Field | Valid range | n | Min bits | Bits used | Reason |\n");
    printf("| --- | --- | ---: | ---: | ---: | --- |\n");
    for (unsigned int i = 0u; i < r->nfields; ++i) {
        const field_t *f = &r->fields[i];
        printf("| %s", f->name);
        if (f->repeat > 1u) {
            printf("[%u]", f->repeat);
        }
        printf(" | ");
        print_range(f);
        printf(" | %llu | %u | %u | %s", (unsigned long long)field_n(f),
               min_bits(f), f->width, reason_name[f->reason]);
        if (f->reason == R_SMALL_GAIN) {
            printf(" (%u B)", f->accepted);
        }
        printf(" |\n");
    }
    static const char *const layout_name[] = {
        "separate allocations", "one array", "one bit stream"};
    printf("\nBits per element: used %u, minimum %u. Layout: %s. Bytes: "
           "stored %u; least at used widths %u; least at minimum widths %u "
           "(each element alone in whole %ss: %u).",
           t.used, t.minimum, layout_name[r->layout], t.total, t.streamed,
           t.smin, r->unit == 8u ? "byte" : "word", t.aligned);
    if (r->aligned_bytes != 0u) {
        printf(" ELEMENT_ALIGNED: %u B.", r->aligned_bytes);
    }
    if (r->note) {
        printf(" %s", r->note);
    }
    printf("\n\n");
}

static void field_list(const record_t *r, int reason)
{
    bool any = false;
    for (unsigned int j = 0u; j < r->nfields; ++j) {
        if (r->fields[j].reason == reason) {
            printf("%s%s", any ? ", " : "", r->fields[j].name);
            any = true;
        }
    }
}

static void gap_summary(void)
{
    const unsigned int n = sizeof(records) / sizeof(records[0]);
    printf("### Report-only gaps\n\n");
    printf("| Record | Fields (GAP) | Stored B | Least B | Saving B |\n");
    printf("| --- | --- | ---: | ---: | ---: |\n");
    for (unsigned int i = 0u; i < n; ++i) {
        const record_t *r = records[i];
        const totals_t t = record_totals(r);
        bool gap = false;
        for (unsigned int j = 0u; j < r->nfields; ++j) {
            gap = gap || r->fields[j].reason == R_GAP;
        }
        if (!gap) {
            continue;
        }
        printf("| %s | ", r->name);
        field_list(r, R_GAP);
        printf(" | %u | %u | %u |\n", t.streamed, t.smin,
               t.streamed - t.smin);
    }
    printf("\n### Accepted with a reason\n\n");
    printf("| Record | Reason | Fields | Bytes not saved |\n");
    printf("| --- | --- | --- | ---: |\n");
    for (unsigned int i = 0u; i < n; ++i) {
        const record_t *r = records[i];
        for (unsigned int j = 0u; j < r->nfields; ++j) {
            if (r->fields[j].reason == R_SMALL_GAIN) {
                printf("| %s | SMALL_GAIN | %s | %u |\n", r->name,
                       r->fields[j].name, r->fields[j].accepted);
            }
        }
        if (r->aligned_bytes != 0u) {
            printf("| %s | ELEMENT_ALIGNED | all (%u zero bits per "
                   "element) | %u |\n", r->name, r->spare,
                   r->aligned_bytes);
        }
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    const bool md = argc > 1 && strcmp(argv[1], "--markdown") == 0;
    build_lists();
    build_imu_lists();
    for (unsigned int i = 0u; i < sizeof(records) / sizeof(records[0]); ++i) {
        if (md) {
            markdown(records[i]);
        } else {
            prove(records[i]);
        }
    }
    if (md) {
        gap_summary();
    } else {
        printf("min-bit proof: %zu records\n",
               sizeof(records) / sizeof(records[0]));
    }
    return 0;
}
