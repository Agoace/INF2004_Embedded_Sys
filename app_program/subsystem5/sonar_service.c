/*
 * sonar_service.c - application telemetry and service interface.
 */
#include "subsystem5.h"

/* Task timing and resource limits. */
#define SONAR_WATCH_SLACK_MS       20u
#define SONAR_REQUEST_BUFFER_BYTES 256
#define SONAR_RELEASE_MS           10u
#define SONAR_TASK_PRIORITY        12
#define SONAR_MIN_ANGLE_DEG        (-30)
#define SONAR_MAX_ANGLE_DEG        30
#define SONAR_MIN_INTERVAL_MS      60u

#include "sonar_service.h"
#include "drv_ultrasonic.h"
#include "../platform/drv_time.h"
/* Only aligned 32-bit atomic loads/stores: Cortex-M0+ needs no libatomic
 * read-modify-write helper. A reader never waits for the lower-priority writer.
 * All shared words are atomic, including the payload (no C data races). */
#define SNAPSHOT_WORDS                                                         \
    ((sizeof(sonar_snapshot_t) + sizeof(uint32_t) - 1u) / sizeof(uint32_t))
static uint32_t board_guard;
static uint32_t board_words[SNAPSHOT_WORDS];
static uint32_t history_count;
static uint32_t history_guard;
static uint32_t history_words[SONAR_HISTORY_CAPACITY][SNAPSHOT_WORDS];

/*
 * Read one aligned board word with the required compiler ordering.
 */
static uint32_t load_word(const uint32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_SEQ_CST);
}

/*
 * Store one aligned board word with the required compiler ordering.
 */
static void store_word(uint32_t *p, uint32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_SEQ_CST);
}

/*
 * Copy a sonar snapshot into its aligned-word board storage.
 */
static void store_snapshot(uint32_t *words, const sonar_snapshot_t *value)
{
    uint32_t copy[SNAPSHOT_WORDS] = {0};
    for (unsigned int i = 0u; i < sizeof(*value); ++i) {
        ((unsigned char *)copy)[i] = ((const unsigned char *)value)[i];
    }
    for (unsigned int i = 0u; i < SNAPSHOT_WORDS; ++i) {
        store_word(&words[i], copy[i]);
    }
}

/*
 * Copy aligned board words into the caller-owned snapshot.
 */
static void load_snapshot(const uint32_t *words, sonar_snapshot_t *value)
{
    uint32_t copy[SNAPSHOT_WORDS];
    for (unsigned int i = 0u; i < SNAPSHOT_WORDS; ++i) {
        copy[i] = load_word(&words[i]);
    }
    for (unsigned int i = 0u; i < sizeof(*value); ++i) {
        ((unsigned char *)value)[i] = ((const unsigned char *)copy)[i];
    }
}

/*
 * Publish a sonar snapshot and append completed results to history.
 */
static void board_publish(const sonar_snapshot_t *value, bool append)
{
    const uint32_t guard = load_word(&board_guard);
    store_word(&board_guard, guard + 1u);
    store_snapshot(board_words, value);
    store_word(&board_guard, guard + 2u);
    if (append) {
        const uint32_t hguard = load_word(&history_guard);
        store_word(&history_guard, hguard + 1u);
        const uint32_t count = load_word(&history_count);
        store_snapshot(history_words[count % SONAR_HISTORY_CAPACITY], value);
        store_word(&history_count, count + 1u);
        store_word(&history_guard, hguard + 2u);
    }
}

static ID mutex;
static ID requests;
static ID results;
static ID cyclic;
static ID task;
static bool ready;
static bool physical;
static bool await_ack;
static bool have_generation;
static bool once_reserved;
static bool admission_ready;
static bool cyclic_running;
static uint32_t admitted_generation;
static sonar_request_t current;
static sonar_request_t inflight;
static sonar_snapshot_t snapshot;
static uint32_t next_ms;

/*
 * Send one non-retained observation; failure does not affect driving.
 */
static void publish(uint8_t status, ultrasonic_result_t echo, uint32_t now)
{
    if (tk_loc_mtx(mutex, TMO_FEVR) < E_OK) {
        return;
    }
    snapshot.generation = current.generation;
    snapshot.id = current.id;
    snapshot.angle = current.angle;
    snapshot.echo = echo;
    snapshot.status = status;
    snapshot.pending = false;
    /* A delayed service must not relabel an old echo as a fresh observation. */
    snapshot.timestamp_ms = status == SONAR_ECHO ? snapshot.trigger_ms : now;
    snapshot.deadline_ms = current.mode == SONAR_WATCH
                               ? snapshot.timestamp_ms +
                                     (uint32_t)current.interval_ms * 2u +
                                     SONAR_WATCH_SLACK_MS
                               : current.deadline_ms;
    snapshot.valid =
        status == SONAR_ECHO &&
        ultrasonic_result_status(echo) == ULTRASONIC_STATUS_VALID &&
        (int32_t)(snapshot.deadline_ms - now) >= 0;
    ++snapshot.result_sequence;
    board_publish(&snapshot, true);
    if (current.generation == admitted_generation &&
        current.mode == SONAR_ONCE) {
        once_reserved = false;
    }
    (void)tk_unl_mtx(mutex);
    (void)tk_set_flg(results, 1u);
}

/*
 * Apply a service request without blocking the vehicle task.
 */
static void accept(const sonar_request_t *r, uint32_t now)
{
    if (have_generation && (int32_t)(r->generation - current.generation) < 0) {
        return;
    }
    if (r->mode == SONAR_ACK) {
        if (r->generation == current.generation && r->id == current.id) {
            await_ack = false;
        }
        return;
    }
    const bool new_generation =
        !have_generation || r->generation != current.generation;
    if (!new_generation && current.mode == SONAR_ONCE &&
        r->mode != SONAR_STOP) {
        /* One outstanding one-shot; do not silently replace it. */
        return;
    }
    current = *r;
    have_generation = true;
    await_ack = false;
    if (tk_loc_mtx(mutex, TMO_FEVR) >= E_OK) {
        snapshot.generation = r->generation;
        snapshot.id = r->id;
        snapshot.angle = r->angle;
        snapshot.valid = false;
        snapshot.pending = false;
        snapshot.status = SONAR_EMPTY;
        board_publish(&snapshot, false);
        (void)tk_clr_flg(results, ~1u);
        (void)tk_unl_mtx(mutex);
    }
    if (r->mode == SONAR_STOP) {
        publish(SONAR_CANCELLED, ULTRASONIC_RESULT_NONE, now);
    }
    next_ms = r->not_before_ms != 0u
                  ? r->not_before_ms
                  : now + (r->mode == SONAR_WATCH ? r->interval_ms : 0u);
}

/*
 * Own sonar requests, result collection and service deadlines.
 */
static void sonar_task(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    for (;;) {
        if (tk_slp_tsk(TMO_FEVR) < E_OK) {
            continue;
        }
        (void)tk_can_wup(TSK_SELF);
        sonar_request_t r;
        while (tk_rcv_mbf(requests, &r, TMO_POL) == (INT)sizeof(r)) {
            accept(&r, drv_time_ms());
        }
        uint32_t now = drv_time_ms();
        if (drv_ultrasonic_service()) {
            if (physical && inflight.generation == current.generation &&
                inflight.id == current.id && current.mode != SONAR_STOP) {
                if ((int32_t)(now - inflight.deadline_ms) <= 0 ||
                    current.mode == SONAR_WATCH) {
                    publish(SONAR_ECHO, drv_ultrasonic_latest(), now);
                } else {
                    publish(SONAR_DEADLINE, ULTRASONIC_RESULT_NONE, now);
                }
                await_ack = current.mode == SONAR_WATCH;
                if (current.mode == SONAR_ONCE) {
                    current.mode = SONAR_STOP;
                }
            }
            physical = false;
        }
        if (current.mode == SONAR_ONCE &&
            (int32_t)(now - current.deadline_ms) >= 0) {
            publish(SONAR_DEADLINE, ULTRASONIC_RESULT_NONE, now);
            current.mode = SONAR_STOP;
        }
        if ((current.mode != SONAR_ONCE && current.mode != SONAR_WATCH) ||
            physical || await_ack || (int32_t)(now - next_ms) < 0) {
            continue;
        }
        if (!drv_ultrasonic_trigger()) {
            continue;
        }
        inflight = current;
        physical = true;
        next_ms = now + current.interval_ms;
        if (tk_loc_mtx(mutex, TMO_FEVR) >= E_OK) {
            snapshot.trigger_ms = now;
            snapshot.pending = true;
            ++snapshot.ping_sequence;
            snapshot.ping_generation = current.generation;
            snapshot.ping_id = current.id;
            snapshot.ping_angle = current.angle;
            board_publish(&snapshot, false);
            (void)tk_unl_mtx(mutex);
        }
        /* STUCK_HIGH is an immediate result, exactly as in the reference. */
        if (!drv_ultrasonic_busy()) {
            publish(SONAR_ECHO, drv_ultrasonic_latest(), now);
            physical = false;
            await_ack = current.mode == SONAR_WATCH;
            if (current.mode == SONAR_ONCE) {
                current.mode = SONAR_STOP;
            }
        }
    }
}

/*
 * Wake the sonar service from its periodic cyclic handler.
 */
static void release(void *ctx)
{
    (void)ctx;
    if (task > 0) {
        (void)tk_wup_tsk(task);
    }
}

/*
 * Create sonar request, result and periodic-release kernel objects.
 */
int sonar_service_init(void)
{
    if (ready) {
        return E_OK;
    }
    const T_CMTX m = {.mtxatr = TA_INHERIT};
    mutex = tk_cre_mtx(&m);
    if (mutex < 0) {
        return (int)mutex;
    }
    const T_CMBF b = {.mbfatr = TA_TFIFO,
                      .bufsz = SONAR_REQUEST_BUFFER_BYTES,
                      .maxmsz = (INT)sizeof(sonar_request_t)};
    requests = tk_cre_mbf(&b);
    if (requests < 0) {
        return (int)requests;
    }
    const T_CFLG f = {.flgatr = TA_TFIFO, .iflgptn = 0u};
    results = tk_cre_flg(&f);
    if (results < 0) {
        return (int)results;
    }
    T_CCYC c = {.cycatr = TA_HLNG,
                .cychdr = (FP)release,
                .cyctim = SONAR_RELEASE_MS,
                .cycphs = SONAR_RELEASE_MS};
#if TK_SUPPORT_SMP
    c.cycatr |= TA_ASSPRC;
    c.assprc = TP_PRC1;
#endif
    cyclic = tk_cre_cyc(&c);
    if (cyclic < 0) {
        return (int)cyclic;
    }
    const int er = drv_ultrasonic_init();
    if (er >= E_OK) {
        ready = true;
    }
    return er;
}

/*
 * Start the sonar task and periodic release source.
 */
int sonar_service_start(void)
{
    if (!ready || task > 0) {
        return E_OBJ;
    }
    task = app_start_task((FP)sonar_task, SONAR_TASK_PRIORITY, TP_PRC1,
                          "sonar_task");
    if (task < 0) {
        return (int)task;
    }
    drv_ultrasonic_listen((int)task);
    const ER er = tk_sta_cyc(cyclic);
    cyclic_running = er >= E_OK;
    return (int)er;
}

/*
 * Report whether the sonar task has started.
 */
bool sonar_service_running(void)
{
    return ready && task > 0 && cyclic_running;
}

/*
 * Validate and queue a sonar request without waiting for the owner.
 */
int sonar_request(const sonar_request_t *r)
{
    if (!sonar_service_running()) {
        return E_OBJ;
    }
    if (!r || r->mode > SONAR_ACK || r->angle < SONAR_MIN_ANGLE_DEG ||
        r->angle > SONAR_MAX_ANGLE_DEG ||
        (r->mode == SONAR_WATCH && r->interval_ms < SONAR_MIN_INTERVAL_MS)) {
        return E_PAR;
    }
    const ER lock = tk_loc_mtx(mutex, TMO_POL);
    if (lock < 0) {
        return (int)lock;
    }
    if ((admission_ready &&
         (int32_t)(r->generation - admitted_generation) < 0) ||
        (once_reserved && r->generation == admitted_generation &&
         r->mode != SONAR_STOP && r->mode != SONAR_ACK)) {
        (void)tk_unl_mtx(mutex);
        return E_OBJ;
    }
    const ER er = tk_snd_mbf(requests, r, (INT)sizeof(*r), TMO_POL);
    if (er >= E_OK && r->mode != SONAR_ACK) {
        admitted_generation = r->generation;
        admission_ready = true;
        once_reserved = r->mode == SONAR_ONCE;
    }
    (void)tk_unl_mtx(mutex);
    if (er >= E_OK) {
        (void)tk_wup_tsk(task);
    }
    return (int)er;
}

/*
 * Attempt one read of the public sonar board without waiting.
 */
int sonar_snapshot_try_read(sonar_snapshot_t *out)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    const uint32_t guard = load_word(&board_guard);
    if ((guard & 1u) != 0u) {
        return E_TMOUT;
    }
    sonar_snapshot_t copy;
    load_snapshot(board_words, &copy);
    if (load_word(&board_guard) != guard) {
        return E_TMOUT;
    }
    *out = copy;
    return E_OK;
}

/*
 * Read the next retained result and report any evicted history entries.
 */
int sonar_history_read(uint32_t *cursor, sonar_history_entry_t *out,
                       uint32_t *missed)
{
    if (!cursor || !out || !missed) {
        return E_PAR;
    }
    *missed = 0u;
    if (!ready) {
        return E_OBJ;
    }
    const uint32_t guard = load_word(&history_guard);
    if ((guard & 1u) != 0u) {
        return E_TMOUT;
    }
    const uint32_t count = load_word(&history_count);
    const uint32_t distance = count - *cursor;
    sonar_history_entry_t copy;
    uint32_t lost = 0u;
    if (distance != 0u) {
        lost = distance > SONAR_HISTORY_CAPACITY
                   ? distance - SONAR_HISTORY_CAPACITY
                   : 0u;
        copy.sequence = *cursor + lost + 1u;
        load_snapshot(
            history_words[(copy.sequence - 1u) % SONAR_HISTORY_CAPACITY],
            &copy.result);
    }
    if (load_word(&history_guard) != guard) {
        return E_TMOUT;
    }
    if (distance == 0u) {
        return E_NOEXS;
    }
    *out = copy;
    *cursor = copy.sequence;
    *missed = lost;
    return E_OK;
}

/*
 * Copy the current vehicle-facing sonar result and freshness flag.
 */
int sonar_result_read(sonar_snapshot_t *out, bool *is_new)
{
    if (!ready) {
        return E_OBJ;
    }
    if (!out || !is_new) {
        return E_PAR;
    }
    const ER er = tk_loc_mtx(mutex, TMO_FEVR);
    if (er < 0) {
        return (int)er;
    }
    *out = snapshot;
    UINT bits = 0u;
    *is_new =
        tk_wai_flg(results, 1u, TWF_ORW | TWF_BITCLR, &bits, TMO_POL) >= E_OK;
    return (int)tk_unl_mtx(mutex);
}
