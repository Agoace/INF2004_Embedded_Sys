/* Host producer double; record/control TYPES come from the canonical header. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "junction_live.h"

static barcode_live_record_t slot;
static barcode_live_control_t published;
static uint32_t publication;
static unsigned int reads;
static int read_error;
static uint32_t fixture_clock;
static uint32_t read_clock_advance;
static uint32_t producer_serial;
static uint32_t race_epoch;
static bool occupied;

static bool producer_put(uint32_t epoch);

uint32_t drv_time_ms(void)
{
    return fixture_clock;
}
static jlive_t state;
static ja_t action;
static lp_t line;
static ja_sample_t sample;

int barcode_live_source_try_read(barcode_live_record_t *out)
{
    ++reads;
    if (read_error != 0) {
        return read_error;
    }
    *out = slot;
    if (race_epoch != 0u) {
        const uint32_t epoch = race_epoch;
        race_epoch = 0u;
        assert(producer_put(epoch));
    }
    fixture_clock += read_clock_advance;
    return 0;
}

int barcode_live_control_publish(const barcode_live_control_t *value)
{
    published = *value;
    publication += 2u;
    return 0;
}

static bool producer_put(uint32_t epoch)
{
    if (occupied && published.ack_sequence != slot.sequence) {
        return false;
    }
    occupied = false;
    if (published.mode == BC_LIVE_DISABLED) {
        return false;
    }
    if (++producer_serial == 0u) {
        ++producer_serial;
    }
    slot.epoch = epoch;
    slot.sequence = producer_serial;
    slot.read_ms = fixture_clock - 5u;
    slot.eligibility_revision = published.eligibility_revision;
    occupied = true;
    return true;
}

static void tick(uint32_t now, int32_t count, bool black)
{
    fixture_clock = now;
    const int32_t counts[2] = {count, count};
    line.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN;
    if (black) {
        line.flags |= LP_FLAG_JUNCTION;
    }
    line.norm[0] = black ? 800u : 600u;
    line.norm[1] = black ? 800u : 300u;
    sample.line_stamp += 10000u;
    jlive_tick(&state, &line, &sample, counts, now, &action,
                  action.phase == JA_PENDING || ja_active(&action));
}

static void init(void)
{
    slot = (barcode_live_record_t){0};
    reads = 0u;
    read_error = 0;
    read_clock_advance = 0u;
    race_epoch = 0u;
    producer_serial = 1u;
    occupied = true;
    publication = 0u;
    ja_init(&action, 131, 1u);
    sample = (ja_sample_t){.valid = true};
    jlive_start(&state, 1u, 100u);
    jlive_route(&state, VSM_MOD_LINE, 101u);
    tick(110u, 0, false);
    tick(120u, 20, false);
    tick(130u, 100, false);
    assert(state.clear);
    slot = (barcode_live_record_t){.epoch = 1u, .sequence = 1u,
        .read_ms = 125u, .read_counts = {73, 74}, .command = JA_LEFT,
        .valid = 1u, .source = BC_LIVE_SOURCE,
        .frame_end_counts = {-100, -100},
        .eligibility_revision = state.control.eligibility_revision,
        .letter = 'A', .direction = 1u, .flags = BC_LIVE_REQUIRED,
        .version = BC_LIVE_VERSION};
}

static bool receive(ja_command_t *out)
{
    barcode_live_record_t copy;
    const int status = barcode_live_source_try_read(&copy);
    return jlive_read(&state, status == 0 ? &copy : NULL, state.now, out);
}

static void unchanged_read(void)
{
    ja_command_t out;
    memset(&out, 0x5a, sizeof(out));
    const ja_command_t before = out;
    assert(!receive(&out));
    assert(memcmp(&out, &before, sizeof(out)) == 0);
}

static void coherent(void)
{
    init();
    ja_command_t out = {0};
    assert(receive(&out) == (BARCODE_LIVE_ENABLE != 0));
    assert(reads == 1u);
    if (BARCODE_LIVE_ENABLE) {
        assert(out.epoch == slot.epoch && out.sequence == slot.sequence);
        assert(out.read_ms == 125u && out.read_counts[0] == 73);
        assert(out.read_counts[1] == 74 && out.source == 2u && out.valid);
        assert(ja_offer(&action, &out));
        jlive_disposition(&state, out.sequence, true);
        assert(published.mode == BC_LIVE_DISABLED);
        assert(published.ack_sequence == 1u);
    } else {
        assert(state.report.reason == JLIVE_SHADOW);
        assert(state.control.ack_sequence == 1u);
        assert(action.phase == JA_IDLE);
    }
    unchanged_read();
    init();
    read_error = -50;
    unchanged_read();
    assert(state.control.ack_sequence == 0u);
    assert(!receive(NULL));
}

static void invalid(void)
{
    for (unsigned int i = 0u; i < 9u; ++i) {
        init();
        switch (i) {
        case 0: slot.version = 2u; break;
        case 1: slot.source = 1u; break;
        case 2: slot.reserved = 1u; break;
        case 3: slot.valid = 2u; break;
        case 4: slot.command = 5u; break;
        case 5: slot.letter = '*'; break;
        case 6: slot.direction = 0u; break;
        case 7: slot.direction = 3u; break;
        default: slot.read_ms = 131u; break;
        }
        unchanged_read();
        assert(state.report.reason == JLIVE_INVALID);
        assert(state.control.ack_sequence == 1u);
    }
    init();
    slot.flags &= (uint8_t)~BC_LIVE_OFFSET_VERIFIED;
    unchanged_read();
    init();
    slot.command = JA_NONE;
    slot.valid = 0u;
    slot.letter = 'Z';
    unchanged_read();
    assert(state.control.ack_sequence == 1u);
}

static void age(void)
{
    init();
    tick(200u, 200, false);
    slot.read_ms = 150u;
    ja_command_t out = {0};
    assert(receive(&out) == (BARCODE_LIVE_ENABLE != 0));
    assert(state.report.max_delivery_ms == 50u);
    init();
    tick(200u, 200, false);
    slot.read_ms = 149u;
    unchanged_read();
    assert(state.report.reason == JLIVE_LATE);
    assert(state.report.max_delivery_ms == 51u);
}

static void epoch(void)
{
    init();
    slot.epoch = 3u;
    unchanged_read();
    assert(state.control.ack_sequence == 1u);
    unchanged_read();
    assert(state.reasons[JLIVE_OLD_RUN] == 1u);
    jlive_start(&state, 3u, 200u);
    assert(published.epoch == 3u && published.ack_sequence == 0u);
    assert(published.mode == BC_LIVE_DISABLED);
}

static void route(void)
{
    init();
    const uint32_t original = slot.eligibility_revision;
    jlive_route(&state, VSM_MOD_SEARCH, 140u);
    jlive_route(&state, VSM_MOD_LINE, 150u);
    tick(180u, 200, false);
    slot.read_ms = 175u;
    assert(state.control.eligibility_revision != original);
    unchanged_read();
    assert(state.report.reason == JLIVE_CROSSED);
    init();
    /* An unread record cannot become current by attaching a newer token. */
    slot.read_ms = state.control.revision_ms;
    unchanged_read();
    assert(state.report.reason == JLIVE_CROSSED);
    for (uint8_t id = VSM_MOD_SCAN; id <= VSM_MOD_DETOUR; ++id) {
        init();
        jlive_route(&state, id, 140u);
        unchanged_read();
        assert(published.mode == BC_LIVE_DISABLED);
    }
}

static void encounter(void)
{
    init();
    const uint32_t before = state.control.eligibility_revision;
    tick(140u, 150, true);
    assert(state.control.revision_ms == 140u);
    assert(state.control.eligibility_revision == before);
    assert(state.edge[0] == 150 && state.edge[1] == 150);
    unchanged_read();
    tick(150u, 225, true);
    assert(state.control.eligibility_revision == before + 1u);
    tick(160u, 400, true);
    assert(state.control.eligibility_revision == before + 1u);
    assert(state.report.encounter_counts[0] == 150);
    /* Fresh clear baseline is required for a second encounter. */
    tick(170u, 450, false);
    tick(180u, 530, false);
    tick(190u, 550, true);
    tick(200u, 625, true);
    assert(state.control.eligibility_revision == before + 2u);
    init();
    state.clear = false; /* re-entry/start without observer baseline */
    tick(140u, 150, true);
    slot.read_ms = 135u;
    unchanged_read();
    assert(state.report.reason == JLIVE_CROSSED);
    assert(published.mode == BC_LIVE_DISABLED);
}

static void motion(void)
{
    init();
    const uint32_t revision = state.control.eligibility_revision;
    tick(140u, 150, true);
    sample.line_stamp -= 10000u; /* next tick repeats previous observation */
    tick(150u, 225, true);
    assert(state.control.eligibility_revision == revision);
    init();
    tick(140u, 150, true);
    tick(150u, 150, true);
    tick(160u, 250, true);
    assert(state.control.eligibility_revision == revision);
    init();
    tick(140u, 150, true);
    tick(150u, 100, true);
    tick(160u, 250, true);
    assert(state.control.eligibility_revision == revision);
}

static void pending(void)
{
    init();
    ja_command_t command = {.epoch = 1u, .sequence = 42u,
        .read_ms = 125u, .read_counts = {73, 74}, .command = JA_LEFT,
        .source = 2u, .valid = 1u};
    assert(ja_offer(&action, &command));
    tick(140u, 150, true);
    assert(action.phase == JA_UNUSED && !action.result.claimed);
    init();
    state.clear = false;
    assert(ja_offer(&action, &command));
    tick(140u, 150, true);
    assert(action.phase == JA_UNUSED);
    assert(state.reasons[JLIVE_UNARMED] == 1u);
    init();
    assert(ja_offer(&action, &command));
    action.clear = true;
    tick(140u, 150, true);
    assert(action.phase == JA_PENDING);
    const int32_t counts[2] = {150, 150};
    jlive_route(&state, VSM_MOD_SEARCH, 150u);
    ja_pending_route(&action, VSM_MOD_SEARCH, counts, 150u);
    jlive_route(&state, VSM_MOD_LINE, 160u);
    ja_pending_route(&action, VSM_MOD_LINE, counts, 160u);
    assert(action.phase == JA_PENDING);
    assert(action.command.read_ms == 125u && action.command.sequence == 42u);
    assert(!action.clear && action.command.read_counts[0] == 73);
    ja_pending_route(&action, VSM_MOD_SCAN, counts, 170u);
    assert(action.phase == JA_UNUSED);
    init();
    command.source = 1u;
    assert(ja_offer(&action, &command));
    tick(140u, 150, true);
    assert(action.phase == JA_PENDING); /* FAKE1 never acquires live guard. */
}

static void guard(void)
{
    if (!BARCODE_LIVE_ENABLE) {
        init();
        slot.valid = 0u;
        slot.flags |= BC_LIVE_SHADOW;
        unchanged_read();
        assert(!state.waiting && state.control.ack_sequence == 1u);
        return;
    }
    init();
    slot.frame_end_counts[0] = 100;
    slot.frame_end_counts[1] = 100;
    unchanged_read();
    assert(state.waiting && state.control.ack_sequence == 0u);
    tick(800u, 120, false);
    ja_command_t out = {0};
    assert(receive(&out)); /* receipt age != waiting lifetime */
    assert(state.report.max_delivery_ms == 5u);
    jlive_disposition(&state, out.sequence, true);
    assert(published.mode == 0u && published.ack_sequence == 1u);
    init();
    slot.frame_end_counts[0] = 100;
    slot.frame_end_counts[1] = 100;
    unchanged_read();
    tick(140u, 150, true);
    unchanged_read();
    assert(!state.waiting && state.control.ack_sequence == 1u);
}

static void heartbeat(void)
{
    init();
    const uint32_t first = publication;
    tick(140u, 100, false);
    assert(publication != first);
    assert(published.epoch == 1u);
    assert(published.mode == (BARCODE_LIVE_ENABLE ? 2u : 1u));
    jlive_stop(&state, 150u);
    assert(published.epoch == 0u && published.mode == 0u);
    jlive_report_t r;
    assert(jlive_report(&state, &r));
    assert(r.reason == JLIVE_STOP && r.control.epoch == 1u);
    assert(!jlive_report(&state, &r));
}

static void wrap(void)
{
    init();
    state.previous[0] = INT32_MAX - 150;
    state.previous[1] = INT32_MAX - 150;
    const uint32_t revision = state.control.eligibility_revision;
    tick(140u, INT32_MAX - 30, true);
    const int32_t wrapped = (int32_t)((uint32_t)(INT32_MAX - 30) + 75u);
    tick(150u, wrapped, true);
    assert(state.control.eligibility_revision == revision + 1u);
    init();
    state.control.revision_ms = UINT32_MAX - 30u;
    slot.read_ms = UINT32_MAX - 10u;
    tick(5u, 200, false);
    ja_command_t out;
    assert(receive(&out) == (BARCODE_LIVE_ENABLE != 0));
    assert(state.report.max_delivery_ms == 16u);
}

static void ramp(void)
{
    init();
    const uint32_t revision = state.control.eligibility_revision;
    const int32_t counts[2] = {150, 150};
    line.flags |= LP_FLAG_JUNCTION;
    line.norm[0] = 650u;
    line.norm[1] = 650u;
    sample.line_stamp += 10000u;
    jlive_tick(&state, &line, &sample, counts, 140u, &action, false);
    assert(state.clear && !state.candidate);
    tick(150u, 160, true);
    tick(160u, 235, true);
    assert(state.edge[0] == 160);
    assert(state.control.eligibility_revision == revision + 1u);
    tick(170u, 250, false);
    tick(180u, 330, false);
    /* The whole black/disabled interval cannot be missed by a backlogged
     * first bar which attaches the current revision on later processing. */
    assert(state.control.revision_ms == 180u);
}

static void freshness(void)
{
    init();
    ja_command_t out = {0};
    assert(!jlive_read(&state, &slot, 141u, &out));
    assert(state.report.reason == JLIVE_LINE_STALE);
    assert(state.reasons[JLIVE_LINE_STALE] == 1u);
    assert(state.report.max_delivery_ms == 16u);
}

static void counters(void)
{
    init();
    slot.version = 2u;
    unchanged_read();
    unchanged_read();
    assert(state.reasons[JLIVE_INVALID] == 1u);
    jlive_counts_t result;
    memset(&result, 0x5a, sizeof(result));
    const jlive_counts_t before = result;
    assert(!jlive_counts(&state, &result));
    assert(memcmp(&result, &before, sizeof(result)) == 0);
    read_error = -50;
    for (unsigned int i = 0u; i < 65540u; ++i) {
        unchanged_read();
    }
    assert(state.reasons[JLIVE_READ_FAILURE] == UINT16_MAX);
    jlive_stop(&state, 200u);
    assert(jlive_counts(&state, &result));
    assert(result.epoch == 1u && result.reason[JLIVE_INVALID] == 1u);
    assert(result.reason[JLIVE_STOP] == 1u);
    assert(jlive_counts(&state, &result)); /* stopped DUMP can resend */
    jlive_start(&state, 3u, 210u);
    assert(state.reasons[JLIVE_INVALID] == 0u);
    assert(state.reasons[JLIVE_READ_FAILURE] == 0u);
}

static void epoch_slot(void)
{
    init();
    const int32_t counts[2] = {100, 100};
    ja_init(&action, 131, 3u);
    junction_source_start(0u, 3u, 200u, counts);
    junction_source_tick(VSM_MOD_LINE, &line, &sample, counts, 210u, &action);
    fixture_clock = 220u;
    assert(!producer_put(3u)); /* retained old epoch slot still occupied */
    ja_command_t out;
    assert(!junction_source_read(&out));
    assert(published.ack_sequence == 1u && slot.epoch == 1u);
    assert(producer_put(1u)); /* a NEW serial but still wrong epoch */
    assert(slot.sequence == 2u);
    assert(!junction_source_read(&out));
    assert(published.ack_sequence == 2u);
    assert(producer_put(3u));
    assert(slot.sequence == 3u);
    const bool admitted = junction_source_read(&out);
    assert(admitted == (BARCODE_LIVE_ENABLE != 0));
    if (admitted) {
        assert(out.epoch == 3u && out.sequence == 3u);
        assert(ja_offer(&action, &out));
        junction_source_offer_result(out.sequence, true);
    }
    assert(published.ack_sequence == 3u);
    occupied = false; /* producer observes terminalACK before STOP */
    ja_finish(&action, counts, 230u);
    junction_source_stop(230u);
    ja_init(&action, 131, 5u);
    junction_source_start(0u, 5u, 300u, counts);
    sample.line_stamp += 10000u;
    junction_source_tick(VSM_MOD_LINE, &line, &sample, counts, 310u, &action);
    fixture_clock = 320u;
    /* Reader copies old epoch X; writer then publishes current X+1 before
     * that reader's terminalACK. The oldACK cannot retire the new slot. */
    race_epoch = 5u;
    assert(!junction_source_read(&out));
    assert(slot.sequence == 4u && slot.epoch == 5u);
    assert(published.ack_sequence == 3u && occupied);
    junction_source_acknowledge(3u);
    assert(!producer_put(5u));
    const bool current = junction_source_read(&out);
    assert(current == (BARCODE_LIVE_ENABLE != 0));
    if (current) {
        assert(out.epoch == 5u && out.sequence == 4u);
        assert(ja_offer(&action, &out));
        junction_source_offer_result(out.sequence, true);
    }
    assert(published.ack_sequence == 4u);
    junction_source_acknowledge(3u);
    assert(published.ack_sequence == 4u);
}

static void adapter(void)
{
    init();
    const int32_t counts[2] = {100, 100};
    junction_source_start(0u, 3u, 200u, counts);
    junction_source_tick(VSM_MOD_LINE, &line, &sample, counts, 210u, &action);
    slot.epoch = 3u;
    slot.sequence = 7u;
    slot.valid = 0u;
    slot.read_ms = 235u;
    slot.eligibility_revision = published.eligibility_revision;
    sample.line_stamp += 10000u;
    junction_source_tick(VSM_MOD_LINE, &line, &sample, counts, 230u, &action);
    fixture_clock = 230u;
    read_clock_advance = 10u;
    ja_command_t out;
    assert(!junction_source_read(&out));
    assert(junction_source_acknowledged() == 7u);
    jlive_report_t report;
    assert(junction_source_report(&report));
    assert(report.max_delivery_ms == 5u); /* clock AFTER coherent copy */
    junction_source_acknowledge(1u); /* delayed consumer ACK cannot regress */
    assert(junction_source_acknowledged() == 7u);
    junction_source_stop(250u);
    assert(published.mode == 0u && published.epoch == 0u);
}

int main(int argc, char **argv)
{
    struct test {const char *name; void (*run)(void);} tests[] = {
        {"coherent", coherent}, {"invalid", invalid}, {"age", age},
        {"epoch", epoch}, {"route", route}, {"encounter", encounter},
        {"motion", motion}, {"pending", pending}, {"guard", guard},
        {"heartbeat", heartbeat}, {"wrap", wrap}, {"ramp", ramp},
        {"freshness", freshness}, {"counters", counters},
        {"epoch_slot", epoch_slot}, {"adapter", adapter}};
    for (unsigned int i = 0u; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (argc == 1 || strcmp(argv[1], tests[i].name) == 0) {
            tests[i].run();
        }
    }
    return 0;
}
