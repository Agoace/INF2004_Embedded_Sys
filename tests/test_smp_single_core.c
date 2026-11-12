/* M7: links the real UP shim. Only processor instructions are modelled. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <tk/smp.h>

static UINT mask;
static unsigned int barriers, restores, waits;
static UW *isr_address;
static T_SPLOCK *release_address;
static T_SPLOCK *wait_address;
static UINT wait_mask;

UINT shim_test_irq_get(void)
{
    return mask;
}

void shim_test_irq_disable(void)
{
    mask = 1u;
}

void shim_test_irq_set(UINT value)
{
    assert(value <= 1u);
    if (value == 0u && release_address != NULL) {
        assert(*release_address == 0u); /* Release before enabling IRQs. */
        assert(barriers >= 2u);
    }
    mask = value;
    ++restores;
    if (value == 0u && isr_address != NULL) {
        *isr_address = 999u;
        isr_address = NULL;
    }
}
void shim_test_barrier(void)
{
    ++barriers;
}

void shim_test_relax(void)
{
    assert(wait_address != NULL && mask == wait_mask && waits == 0u);
    ++waits;
    /* Deterministic progress, not a real UP task schedule. */
    *wait_address = 0u;
}

static void lock_case(const char *name)
{
    for (UINT initial = 0u; initial <= 1u; ++initial) {
        mask = initial;
        barriers = 0u;
        T_SPLOCK lock = 0u;
        UINT saved = 55u;
        if (strcmp(name, "init") == 0) {
            lock = 8u;
            assert(InitSpinLock(&lock) == E_OK && lock == 0u);
            assert(barriers >= 2u);
        } else if (strcmp(name, "plain_claim") == 0) {
            assert(SpinLock(&lock) == E_OK && lock == 1u);
        } else if (strcmp(name, "plain_try") == 0) {
            assert(SpinTryLock(&lock) == E_OK && lock == 1u);
        } else if (strcmp(name, "plain_busy") == 0) {
            lock = 1u;
            assert(SpinTryLock(&lock) == E_BUSY && lock == 1u);
        } else if (strcmp(name, "plain_unlock") == 0) {
            lock = 1u;
            assert(SpinUnlock(&lock) == E_OK && lock == 0u);
            assert(barriers >= 2u);
        } else if (strcmp(name, "irq_claim") == 0) {
            assert(ISpinLock(&lock, &saved) == E_OK && lock == 1u);
            assert(saved == initial && mask == 1u && barriers >= 2u);
            mask = initial; /* This case tests acquisition, not release. */
        } else if (strcmp(name, "irq_try") == 0) {
            assert(ISpinTryLock(&lock, &saved) == E_OK && lock == 1u);
            assert(saved == initial && mask == 1u && barriers >= 2u);
            mask = initial;
        } else if (strcmp(name, "irq_busy") == 0) {
            lock = 1u;
            assert(ISpinTryLock(&lock, &saved) == E_BUSY && lock == 1u);
            assert(saved == 55u); /* A failed try never publishes a token. */
        } else if (strcmp(name, "irq_unlock") == 0) {
            lock = 1u;
            mask = 1u;
            release_address = &lock;
            assert(ISpinUnlock(&lock, initial) == E_OK && lock == 0u);
            assert(barriers >= 2u);
            release_address = NULL;
        } else {
            assert(false);
        }
        assert(mask == initial);
    }
}

static void nested(void)
{
    T_SPLOCK first = 0u, second = 0u;
    UINT outer, inner;
    assert(ISpinLock(&first, &outer) == E_OK && first == 1u && outer == 0u);
    assert(ISpinTryLock(&first, &inner) == E_BUSY && mask == 1u);
    assert(ISpinLock(&second, &inner) == E_OK && inner == 1u);
    UW counter = 0u;
    assert(atomic_inc(&counter) == 1u && mask == 1u);
    assert(first == 1u && second == 1u);
    assert(ISpinUnlock(&second, inner) == E_OK && mask == 1u);
    assert(ISpinUnlock(&first, outer) == E_OK && mask == 0u);
    /* Match set_primask exactly even if the current mask differs. */
    first = 1u;
    assert(ISpinUnlock(&first, 1u) == E_OK && mask == 1u);
}

static void wait_case(void)
{
    for (UINT initial = 0u; initial <= 1u; ++initial) {
        T_SPLOCK lock = 1u;
        mask = initial;
        wait_address = &lock;
        wait_mask = initial;
        waits = 0u;
        assert(SpinLock(&lock) == E_OK && lock == 1u && waits == 1u);
        assert(mask == initial);
        wait_mask = 1u;
        waits = 0u;
        UINT saved;
        assert(ISpinLock(&lock, &saved) == E_OK && waits == 1u);
        assert(saved == initial && mask == 1u && lock == 1u);
        assert(ISpinUnlock(&lock, saved) == E_OK && mask == initial);
    }
}

static void isr_case(const char *name)
{
    UW value = 5u, result;
    isr_address = &value;
    if (strcmp(name, "inc_isr") == 0) {
        result = atomic_inc(&value);
        assert(result == 6u);
    } else if (strcmp(name, "dec_isr") == 0) {
        result = atomic_dec(&value);
        assert(result == 4u);
    } else if (strcmp(name, "add_isr") == 0) {
        result = atomic_add(&value, 3u);
        assert(result == 8u);
    } else {
        result = atomic_sub(&value, 3u);
        assert(result == 2u);
    }
    assert(value == 999u && isr_address == NULL && mask == 0u);
    assert(restores == 1u && barriers >= 2u);
}

static void old_values(void)
{
    for (unsigned int operation = 0u; operation < 5u; ++operation) {
        UW value = 5u, previous;
        mask = 0u;
        isr_address = &value;
        switch (operation) {
        case 0: previous = atomic_xchg(&value, 8u); break;
        case 1: previous = atomic_cmpxchg(&value, 8u, 5u); break;
        case 2: previous = atomic_cmpxchg(&value, 8u, 7u); break;
        case 3: previous = atomic_bitset(&value, 8u); break;
        default: previous = atomic_bitclr(&value, 1u); break;
        }
        assert(previous == 5u && value == 999u && mask == 0u);
    }
}

static void wrap(void)
{
    for (UINT initial = 0u; initial <= 1u; ++initial) {
        mask = initial;
        UW value = UINT32_MAX;
        assert(atomic_inc(&value) == 0u && value == 0u);
        assert(atomic_dec(&value) == UINT32_MAX && value == UINT32_MAX);
        assert(atomic_add(&value, 2u) == 1u && value == 1u);
        assert(atomic_sub(&value, 2u) == UINT32_MAX);
        assert(atomic_xchg(&value, 5u) == UINT32_MAX);
        assert(atomic_cmpxchg(&value, 8u, 5u) == 5u && value == 8u);
        assert(atomic_cmpxchg(&value, 7u, 5u) == 8u && value == 8u);
        assert(atomic_bitset(&value, 3u) == 8u && value == 11u);
        assert(atomic_bitclr(&value, 2u) == 11u && value == 9u);
        assert(mask == initial);
    }
}

static void invalid(void)
{
    T_SPLOCK lock = 0u;
    UINT saved = 77u;
    assert(InitSpinLock(NULL) == E_PAR && SpinLock(NULL) == E_PAR);
    assert(SpinTryLock(NULL) == E_PAR && SpinUnlock(NULL) == E_PAR);
    assert(ISpinLock(NULL, &saved) == E_PAR);
    assert(ISpinTryLock(NULL, &saved) == E_PAR);
    assert(ISpinLock(&lock, NULL) == E_PAR);
    assert(ISpinTryLock(&lock, NULL) == E_PAR);
    assert(ISpinUnlock(NULL, 0u) == E_PAR);
    assert(saved == 77u && lock == 0u && restores == 0u && mask == 0u);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *name = argv[1];
    if (strcmp(name, "nested") == 0) {
        nested();
    } else if (strcmp(name, "wait") == 0) {
        wait_case();
    } else if (strstr(name, "_isr") != NULL) {
        isr_case(name);
    } else if (strcmp(name, "old_values") == 0) {
        old_values();
    } else if (strcmp(name, "wrap") == 0) {
        wrap();
    } else if (strcmp(name, "null") == 0) {
        invalid();
    } else if (strcmp(name, "barrier") == 0) {
        mp_memory_barrir();
        assert(barriers == 1u && mask == 0u);
    } else {
        lock_case(name);
    }
    return 0;
}
