/* B2: compile the actual helper task and creator lifecycle, not the broken
 * full-suite stubs. Model kernel task states and inject syscall failures.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define E_OK 0
#define E_SYS (-5)
#define E_OBJ (-41)
#define E_LIMIT (-34)
#define TA_HLNG 1u
#define TA_RNG0 0u
#define TA_ASSPRC 0x10000u
#define TTS_RDY 2u
#define TTS_DMT 16u

typedef int ER;
typedef int ID;
typedef int INT;
typedef uint32_t UW;
typedef unsigned int UINT;
typedef unsigned char UB;
typedef UW T_SPLOCK;
typedef void (*FP)(INT, void *);
typedef struct {
    UW tskatr;
    void *exinf;
    FP task;
    int itskpri;
    UW stksz;
#if TK_SUPPORT_SMP
    UW assprc;
#endif
} T_CTSK;
typedef struct {
    UINT tskstat;
} T_RTSK;

static ER ISpinLock(T_SPLOCK *lock, UINT *saved);
static ER ISpinTryLock(T_SPLOCK *lock, UINT *saved);
static ER ISpinUnlock(T_SPLOCK *lock, UINT saved);
static ID tk_cre_tsk(const T_CTSK *task);
static ER tk_sta_tsk(ID id, INT code);
static ER tk_ref_tsk(ID id, T_RTSK *status);
static ER tk_ter_tsk(ID id);
static ER tk_del_tsk(ID id);
static ER tk_dly_tsk(UW ticks);
static void tk_ext_tsk(void);
#if TK_SUPPORT_SMP
static INT tk_get_prc(void);
static UW app_prc_core0(void);
#endif
static int host_printf(const char *format, ...)
    __attribute__((format(printf, 1, 2)));
/* Cast only the first argument: the kernel console uses an unsigned string. */
#define tm_printf(...) host_printf((const char *)__VA_ARGS__)

#include "lifecycle.inc"

typedef enum {
    NORMAL,
    EXIT_PENDING,
    CREATE_FAIL,
    START_FAIL,
    WAIT_REF_FAIL,
    CLEANUP_REF_FAIL,
    EXIT_TIMEOUT,
    DELETE_FAIL,
    TERMINATE_FAIL,
    STOP_OBJ_ACTIVE,
    POST_STOP_REF_FAIL,
    RUN_DELAY_FAIL,
    WAIT_DELAY_FAIL,
    HOLDER_LOCK_FAIL,
    WRONG_CORE
} scenario_t;

static scenario_t scenario;
static T_CTSK created;
static UINT state;
static UINT irq_mask;
static bool exists;
static bool inside_holder;
static unsigned int creates;
static unsigned int starts;
static unsigned int refs;
static unsigned int stops;
static unsigned int deletes;
static unsigned int delays;
static unsigned int exits;
static jmp_buf holder_exit;
static char report[2048];
static size_t report_length;

static ER ISpinLock(T_SPLOCK *lock, UINT *saved)
{
    assert(*lock == 0u);
    if (scenario == HOLDER_LOCK_FAIL && inside_holder) {
        return E_SYS;
    }
    *saved = irq_mask;
    irq_mask = 1u;
    *lock = 1u;
    return E_OK;
}

static ER ISpinTryLock(T_SPLOCK *lock, UINT *saved)
{
    return ISpinLock(lock, saved);
}

static ER ISpinUnlock(T_SPLOCK *lock, UINT saved)
{
    assert(*lock == 1u && irq_mask == 1u);
    *lock = 0u;
    irq_mask = saved;
    return E_OK;
}

static ID tk_cre_tsk(const T_CTSK *task)
{
    assert(irq_mask == 0u && !exists);
    ++creates;
    assert(task->itskpri == 31 && task->stksz == IL_STACK_BYTES);
#if TK_SUPPORT_SMP
    assert((task->tskatr & TA_ASSPRC) != 0u);
    assert(task->assprc == app_prc_core0());
#else
    assert((task->tskatr & TA_ASSPRC) == 0u);
#endif
    if (scenario == CREATE_FAIL) {
        return E_LIMIT;
    }
    created = *task;
    exists = true;
    state = TTS_DMT;
    return 7;
}

static ER tk_sta_tsk(ID id, INT code)
{
    assert(id == 7 && code == 0 && exists && state == TTS_DMT);
    assert(irq_mask == 0u);
    ++starts;
    if (scenario == START_FAIL || scenario == CLEANUP_REF_FAIL) {
        return E_SYS;
    }
    state = TTS_RDY;
    return E_OK;
}

static ER tk_ref_tsk(ID id, T_RTSK *status)
{
    assert(id == 7 && exists && irq_mask == 0u);
    ++refs;
    if ((scenario == WAIT_REF_FAIL || scenario == CLEANUP_REF_FAIL) &&
        refs == 1u) {
        return E_SYS;
    }
    if (scenario == POST_STOP_REF_FAIL && stops != 0u) {
        return E_SYS;
    }
    if (scenario == EXIT_PENDING && refs == 2u) {
        /* The first poll saw done=1 with exit still pending. */
        assert(il_done == 1u && state == TTS_RDY);
        state = TTS_DMT;
    }
    status->tskstat = state;
    return E_OK;
}

static ER tk_ter_tsk(ID id)
{
    assert(id == 7 && exists && !inside_holder && irq_mask == 0u);
    assert(stress_lock == 0u);
    ++stops;
    if (scenario == TERMINATE_FAIL) {
        return E_SYS;
    }
    if (state == TTS_DMT || scenario == STOP_OBJ_ACTIVE) {
        return E_OBJ;
    }
    state = TTS_DMT;
    return E_OK;
}

static ER tk_del_tsk(ID id)
{
    /* This assertion detects the old delete-on-done/active-timeout bug. */
    assert(id == 7 && exists && state == TTS_DMT && irq_mask == 0u);
    ++deletes;
    if (scenario == DELETE_FAIL) {
        return E_SYS;
    }
    exists = false;
    return E_OK;
}

static bool never_runs(void)
{
    return scenario == EXIT_TIMEOUT || scenario == TERMINATE_FAIL ||
           scenario == STOP_OBJ_ACTIVE || scenario == POST_STOP_REF_FAIL;
}

static ER tk_dly_tsk(UW ticks)
{
    assert(ticks == 1u && irq_mask == 0u);
    ++delays;
    if (scenario == RUN_DELAY_FAIL ||
        (scenario == WAIT_DELAY_FAIL && delays > 1u)) {
        return E_SYS;
    }
    if (state == TTS_RDY && exits == 0u && !never_runs()) {
        inside_holder = true;
        if (setjmp(holder_exit) == 0) {
            created.task(0, created.exinf);
            assert(false && "raw task entry returned instead of exiting");
        }
        inside_holder = false;
    }
    return E_OK;
}

static void tk_ext_tsk(void)
{
    assert(inside_holder && irq_mask == 0u && stress_lock == 0u);
    assert(il_done == 1u);
    ++exits;
    state = (scenario == EXIT_PENDING || scenario == WAIT_DELAY_FAIL) ?
            TTS_RDY : TTS_DMT;
    longjmp(holder_exit, 1);
}

#if TK_SUPPORT_SMP
static INT tk_get_prc(void)
{
    return scenario == WRONG_CORE ? 2 : 1;
}

static UW app_prc_core0(void)
{
    return 1u;
}
#endif

static int host_printf(const char *format, ...)
{
    assert(irq_mask == 0u);
    va_list arguments;
    va_start(arguments, format);
    const int count = vsnprintf(report + report_length,
                               sizeof(report) - report_length,
                               format, arguments);
    va_end(arguments);
    assert(count >= 0 && (size_t)count < sizeof(report) - report_length);
    report_length += (size_t)count;
    return count;
}

static void run_case(scenario_t value)
{
    scenario = value;
    stress_lock = 0u;
    stress_errors = 0u;
    irq_mask = 0u;
    exists = false;
    inside_holder = false;
    creates = starts = refs = stops = deletes = delays = exits = 0u;
    report_length = 0u;
    report[0] = '\0';
    interleave_run();
    assert(irq_mask == 0u && stress_lock == 0u && !inside_holder);
    assert(delays <= IL_ROUNDS + IL_WAIT);
    if (value == NORMAL || value == EXIT_PENDING) {
        assert(stress_errors == 0u && exits == 1u);
        assert(deletes == 1u && stops == 0u && !exists);
        assert(strstr(report, "PASS") != NULL);
    } else {
        assert(stress_errors != 0u && strstr(report, "PASS") == NULL);
    }
    if (value == CREATE_FAIL || value == WRONG_CORE) {
        assert(!exists && starts == 0u && deletes == 0u);
    } else if (value == DELETE_FAIL || value == POST_STOP_REF_FAIL) {
        assert(exists && state == TTS_DMT);
    } else if (value == TERMINATE_FAIL || value == STOP_OBJ_ACTIVE) {
        assert(exists && state == TTS_RDY && deletes == 0u);
    } else {
        assert(!exists && deletes == 1u);
    }
    if (value == EXIT_TIMEOUT) {
        assert(stops == 1u && delays == IL_ROUNDS + IL_WAIT);
    }
    if (value == EXIT_PENDING) {
        assert(refs >= 3u);   /* Includes confirmation after pending exit. */
    }
}

int main(void)
{
    for (int value = NORMAL; value <= HOLDER_LOCK_FAIL; ++value) {
        run_case((scenario_t)value);
    }
#if TK_SUPPORT_SMP
    run_case(WRONG_CORE);
#endif
    printf("B2 PASS: SMP=%d, exit/pending/timeout and failure cleanup\n",
           TK_SUPPORT_SMP);
    return 0;
}
