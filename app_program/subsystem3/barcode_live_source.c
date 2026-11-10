/* Canonical boards and single LINE-task producer. SHADOW only; no actions. */
#include "subsystem3.h"
#include "barcode_live_source.h"
#if BC_LIVE_SOURCE_HOST_TEST
extern void barcode_live_test_read_hook(const uint32_t *sequence,
                                        unsigned int index);
#define APP_BOARD_READ_HOOK(s, i) barcode_live_test_read_hook((s), (i))
#endif
#include "../platform/app_board.h"

static bool ready;
static APP_BOARD(barcode_live_record_t) record_board;
static APP_BOARD(barcode_live_control_t) control_board;
_Static_assert(sizeof(record_board) == 44u, "Live record board budget");
_Static_assert(sizeof(control_board) == 24u, "Live control board budget");

int barcode_live_source_init(void)
{
    if (!ready) {
        const barcode_live_record_t empty = {
            .source = BC_LIVE_SOURCE, .version = BC_LIVE_VERSION,
            .flags = BC_LIVE_SHADOW};
        app_board_publish(&record_board.sequence, record_board.words,
                          &empty, sizeof(empty));
        /* Control remains unpublished until its SOLE vehicle writer runs. */
        ready = true;
    }
    return E_OK;
}

int barcode_live_source_try_read(barcode_live_record_t *out)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    barcode_live_record_t value;
    if (!app_board_try_read(&record_board.sequence, record_board.words,
                           &value, sizeof(value))) {
        return E_TMOUT;
    }
    *out = value;
    return E_OK;
}

int barcode_live_control_publish(const barcode_live_control_t *control)
{
    if (!control) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    if (control->mode > BC_LIVE_PERMITTED) {
        return E_PAR;
    }
    app_board_publish(&control_board.sequence, control_board.words,
                      control, sizeof(*control));
    return E_OK;
}

int barcode_live_control_try_read(barcode_live_control_t *out,
                                  uint32_t *publication_tag)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    const uint32_t before =
        __atomic_load_n(&control_board.sequence, __ATOMIC_SEQ_CST);
    barcode_live_control_t value;
    if (!app_board_try_read(&control_board.sequence, control_board.words,
                           &value, sizeof(value)) ||
        __atomic_load_n(&control_board.sequence, __ATOMIC_SEQ_CST) != before) {
        return E_TMOUT;
    }
    *out = value;
    if (publication_tag) {
        *publication_tag = before;
    }
    return E_OK;
}

#if BC_LIVE_SOURCE_HOST_TEST
/* Test-only interrupted-write injection. No record setter is exported in
 * production: S3 remains the sole record producer, never the vehicle. */
void barcode_live_test_publish(const barcode_live_record_t *value)
{
    app_board_publish(&record_board.sequence, record_board.words,
                      value, sizeof(*value));
}
#endif

#if !APP_TEST && !APP_BENCH && !BC_LIVE_SOURCE_HOST_TEST
#include "barcode_live_producer.h"
#include "../subsystem2/motion.h"
#include "../platform/drv_time.h"
#define FRAME_ROWS 8u
#define EDGE_ROWS 24u

typedef struct {
    uint32_t index, epoch, sequence, adc_us, motor_us;
    int32_t counts[2];
    uint16_t raw;
    uint8_t black, reserved;
} edge_row_t;
typedef struct {
    uint32_t epoch, samples, cpu_us, max_us, frames, attempts, edges;
    uint32_t max_control_ms, max_record_ms, failures, edge_lost;
    uint32_t active_samples, active_cpu_us, active_max_us;
} summary_t;
static barcode_live_producer_t producer;
static barcode_live_input_t input;
static motion_status_t motor;
static APP_BOARD(barcode_live_frame_t) frames[FRAME_ROWS];
static APP_BOARD(edge_row_t) edges[EDGE_ROWS];
static APP_BOARD(summary_t) summary_board;
static summary_t summary;
static uint32_t frame_count, edge_count;
static edge_row_t pending_edge;
static bool edge_pending, edge_black, trace;
static uint16_t trace_rise, trace_fall;
static uint32_t trace_epoch, trace_sequence, last_adc_us;
static bool adc_seen;
/* DUMP has ONE consumer. Scratch is static, not on the LINE or dump stack. */
static barcode_live_frame_t dump_frame;
static edge_row_t dump_edge;
static summary_t dump_summary;
_Static_assert(sizeof(producer) + sizeof(input) + sizeof(motor) +
    sizeof(frames) + sizeof(edges) + sizeof(summary_board) + sizeof(summary) +
    sizeof(pending_edge) + sizeof(dump_frame) + sizeof(dump_edge) +
    sizeof(dump_summary) + 192u <= 4096u, "Live storage including boards");

static void resolve_edge(void)
{
    if (!edge_pending || !input.motor_ok || input.motor_faults != 0u ||
        input.motor_us - pending_edge.adc_us >= UINT32_C(0x80000000)) {
        return;
    }
    if (input.motor_us - pending_edge.adc_us > 30000u) {
        ++summary.edge_lost;
    } else {
        pending_edge.index = ++edge_count;
        pending_edge.motor_us = input.motor_us;
        pending_edge.counts[0] = motor.counts[0];
        pending_edge.counts[1] = motor.counts[1];
        const unsigned int at = (edge_count - 1u) % EDGE_ROWS;
        app_board_publish(&edges[at].sequence, edges[at].words,
                          &pending_edge, sizeof(pending_edge));
    }
    edge_pending = false;
}

static void after_window_edge(uint32_t us, uint16_t raw)
{
    resolve_edge();
    if (!trace || (producer.bound && !producer.occupied) ||
        input.control.epoch != trace_epoch ||
        raw == 0u || raw >= 4095u) {
        return;
    }
    const bool black = edge_black ? raw > trace_fall : raw >= trace_rise;
    if (black == edge_black) {
        return;
    }
    edge_black = black;
    if (edge_pending) {
        ++summary.edge_lost;
        return;
    }
    pending_edge = (edge_row_t){.epoch = trace_epoch,
        .sequence = trace_sequence, .adc_us = us, .raw = raw,
        .black = black ? 1u : 0u};
    edge_pending = true;
    resolve_edge();
}

void barcode_live_source_sample(uint32_t us, uint16_t raw,
                                const barcode_cal_report_t *calibration,
                                uint32_t missed, uint32_t dropped)
{
    if (!ready) {
        return;
    }
    const uint32_t start = drv_time_us();
    input.calibration = calibration;
    input.adc_us = us;
    input.raw = raw;
    input.missed = missed;
    input.dropped = dropped;
    input.control_ok = barcode_live_control_try_read(&input.control,
                                                     &input.tag) == E_OK;
    input.motor_ok = motion_snapshot_try_read(&motor, &input.motor_us) == E_OK;
    input.left = (uint32_t)motor.counts[0];
    input.right = (uint32_t)motor.counts[1];
    input.motor_faults = motor.faults;
    /* Pair clocks exactly: low-us minus (ms*1000) gives elapsed phase,
     * including a scheduling delay between reads and both timer wraps. */
    const uint32_t ms = drv_time_ms();
    input.now_us = drv_time_us();
    input.now_ms = ms + (input.now_us - ms * 1000u) / 1000u;
    if (input.control_ok && input.control.epoch != summary.epoch) {
        summary = (summary_t){.epoch = input.control.epoch};
        trace = false;
        edge_pending = false;
    }
    const uint32_t adc_gap = us - last_adc_us;
    const bool gap = adc_seen && (adc_gap == 0u || adc_gap > 1500u);
    adc_seen = true;
    last_adc_us = us;
    if (gap || (producer.counters_seen && (missed != producer.missed ||
         dropped != producer.dropped)) || raw == 0u || raw >= 4095u ||
        !input.motor_ok || input.motor_faults != 0u || !input.control_ok) {
        if (trace) {
            ++summary.edge_lost;
        }
        trace = false;
        edge_pending = false;
    }
    after_window_edge(us, raw);
    barcode_live_producer_observe(&producer, &input);
    if (producer.record_pending) {
        barcode_live_control_t current;
        const bool unchanged = barcode_live_control_try_read(&current, 0)
            == E_OK && current.epoch == producer.slot.epoch &&
            current.eligibility_revision ==
                producer.slot.eligibility_revision &&
            current.revision_ms == producer.context.revision_ms &&
            current.mode != BC_LIVE_DISABLED &&
            current.mode <= BC_LIVE_PERMITTED &&
            drv_time_us() - input.now_us <=
                (BC_LIVE_CONTROL_MAX_AGE_MS - producer.control_age_ms) *
                1000u &&
            drv_time_us() - producer.window.result.read_us <=
                BC_LIVE_RECORD_MAX_AGE_MS * 1000u;
        if (unchanged) {
            const uint32_t extra_ms = (drv_time_us() - input.now_us) / 1000u;
            producer.event.record_age_ms += extra_ms;
            producer.event.control_age_ms += extra_ms;
            if (producer.event.record_age_ms > producer.max_record_age_ms) {
                producer.max_record_age_ms = producer.event.record_age_ms;
            }
            if (producer.event.control_age_ms > producer.max_control_age_ms) {
                producer.max_control_age_ms = producer.event.control_age_ms;
            }
            app_board_publish(&record_board.sequence, record_board.words,
                              &producer.slot, sizeof(producer.slot));
            trace = true;
            trace_epoch = producer.slot.epoch;
            trace_sequence = producer.slot.sequence;
            trace_rise = producer.window.rise;
            trace_fall = producer.window.fall;
            edge_black = false;
        } else {
            /* Never wait for an ACK to a record that was NOT published. */
            producer.occupied = false;
            producer.window_live = false;
            producer.bound = false;
            producer.event.reason = BP_CONTEXT;
            --producer.publications;
            ++producer.failures;
        }
        producer.record_pending = false;
    }
    if (producer.event_pending) {
        producer.event.index = frame_count + 1u;
        const unsigned int at = frame_count++ % FRAME_ROWS;
        app_board_publish(&frames[at].sequence, frames[at].words,
                          &producer.event, sizeof(producer.event));
        producer.event_pending = false;
    }
    summary.frames = producer.publications;
    summary.attempts = frame_count;
    summary.edges = edge_count;
    summary.max_control_ms = producer.max_control_age_ms;
    summary.max_record_ms = producer.max_record_age_ms;
    summary.failures = producer.failures;
    if ((summary.samples + 1u) % 100u == 0u || producer.occupied) {
        /* CPU rows cover completed samples, including this board's cost in
         * the next row. No instrumentation cost is silently excluded. */
        app_board_publish(&summary_board.sequence, summary_board.words,
                          &summary, sizeof(summary));
    }
    ++summary.samples;
    const uint32_t cost = drv_time_us() - start;
    summary.cpu_us += cost;
    if (cost > summary.max_us) {
        summary.max_us = cost;
    }
    if (input.control.mode != BC_LIVE_DISABLED) {
        ++summary.active_samples;
        summary.active_cpu_us += cost;
        if (cost > summary.active_max_us) {
            summary.active_max_us = cost;
        }
    }
}

void barcode_live_source_dump(void)
{
    if (!app_board_try_read(&summary_board.sequence, summary_board.words,
                            &dump_summary, sizeof(dump_summary))) {
        tm_printf((UB *)"[BC_LIVE] no coherent producer summary\n");
        return;
    }
    tm_printf((UB *)"[BC_LIVE] SHADOW epoch %lu samples %lu frames %lu "
              "attempts %lu failures %lu\n",
              (unsigned long)dump_summary.epoch,
              (unsigned long)dump_summary.samples,
              (unsigned long)dump_summary.frames,
              (unsigned long)dump_summary.attempts,
              (unsigned long)dump_summary.failures);
    tm_printf((UB *)"[BC_CPU] total_us %lu samples %lu max_us %lu "
              "eligible_control_age_max_ms %lu publish_age_max_ms %lu\n",
              (unsigned long)dump_summary.cpu_us,
              (unsigned long)dump_summary.samples,
              (unsigned long)dump_summary.max_us,
              (unsigned long)dump_summary.max_control_ms,
              (unsigned long)dump_summary.max_record_ms);
    tm_printf((UB *)"[BC_CPU_ELIGIBLE] total_us %lu samples %lu max_us %lu\n",
              (unsigned long)dump_summary.active_cpu_us,
              (unsigned long)dump_summary.active_samples,
              (unsigned long)dump_summary.active_max_us);
    const uint32_t first = dump_summary.attempts > FRAME_ROWS ?
                          dump_summary.attempts - FRAME_ROWS : 0u;
    for (uint32_t i = first; i < dump_summary.attempts; ++i) {
        const unsigned int at = i % FRAME_ROWS;
        if (!app_board_try_read(&frames[at].sequence, frames[at].words,
                                &dump_frame, sizeof(dump_frame)) ||
            dump_frame.index != i + 1u) {
            continue;
        }
        const barcode_live_frame_t *f = &dump_frame;
        tm_printf((UB *)"[BC_FRAME] epoch %lu seq %lu letter %c cmd %u "
                  "reason %lu offered %u ages_ms %lu/%lu spinW %lu leadW %lu "
                  "delta %ld rise %u fall %u valid0 SHADOW\n",
                  (unsigned long)f->record.epoch,
                  (unsigned long)f->record.sequence,
                  f->record.letter ? (int)f->record.letter : '-',
                  (unsigned int)f->record.command, (unsigned long)f->reason,
                  f->reason == BW_OK && f->record.sequence != 0u ? 1u : 0u,
                  (unsigned long)f->control_age_ms,
                  (unsigned long)f->record_age_ms,
                  (unsigned long)f->spin_white,
                  (unsigned long)f->leading_white,
                  (long)((int32_t)f->leading_white - (int32_t)f->spin_white),
                  (unsigned int)f->rise, (unsigned int)f->fall);
        tm_printf((UB *)"[BC_REF] epoch %lu seq %lu start_us %lu "
                  "edge_us %lu bracket_us %lu end %ld/%ld "
                  "eligible_us %lu read_ms %lu counts %ld/%ld\n",
                  (unsigned long)f->record.epoch,
                  (unsigned long)f->record.sequence,
                  (unsigned long)f->start_us, (unsigned long)f->edge_us,
                  (unsigned long)f->frame_us,
                  (long)f->record.frame_end_counts[0],
                  (long)f->record.frame_end_counts[1],
                  (unsigned long)f->eligible_us,
                  (unsigned long)f->record.read_ms,
                  (long)f->record.read_counts[0],
                  (long)f->record.read_counts[1]);
    }
    const uint32_t edge_first = dump_summary.edges > EDGE_ROWS ?
                               dump_summary.edges - EDGE_ROWS : 0u;
    tm_printf((UB *)"[BC_EDGES] total %lu retained %lu lost %lu\n",
              (unsigned long)dump_summary.edges,
              (unsigned long)(dump_summary.edges - edge_first),
              (unsigned long)dump_summary.edge_lost);
    for (uint32_t i = edge_first; i < dump_summary.edges; ++i) {
        const unsigned int at = i % EDGE_ROWS;
        if (!app_board_try_read(&edges[at].sequence, edges[at].words,
                                &dump_edge, sizeof(dump_edge)) ||
            dump_edge.index != i + 1u) {
            continue;
        }
        tm_printf((UB *)"[BC_EDGE] epoch %lu seq %lu index %lu "
                  "adc_us %lu bracket_us %lu counts %ld/%ld "
                  "raw %u black %u\n",
                  (unsigned long)dump_edge.epoch,
                  (unsigned long)dump_edge.sequence,
                  (unsigned long)dump_edge.index,
                  (unsigned long)dump_edge.adc_us,
                  (unsigned long)dump_edge.motor_us,
                  (long)dump_edge.counts[0], (long)dump_edge.counts[1],
                  (unsigned int)dump_edge.raw, (unsigned int)dump_edge.black);
    }
}
#else
void barcode_live_source_sample(uint32_t us, uint16_t raw,
                                const barcode_cal_report_t *calibration,
                                uint32_t missed, uint32_t dropped)
{
    (void)us;
    (void)raw;
    (void)calibration;
    (void)missed;
    (void)dropped;
}
void barcode_live_source_dump(void) {}
#endif
