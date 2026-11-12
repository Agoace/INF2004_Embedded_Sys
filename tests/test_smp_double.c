/* Contract tests for the shared host double, not for RP2040 lock timing. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <tk/smp.h>

static UINT expected_wait_mask;
static unsigned int waits;

static void release_remote(T_SPLOCK *lock)
{
    assert(test_smp_irq_mask == expected_wait_mask && *lock == 1u);
    ++waits;
    assert(SpinUnlock(lock) == E_OK);
}

static void locks(void)
{
    for (UINT mask = 0u; mask <= 1u; ++mask) {
        test_smp_irq_mask = mask;
        T_SPLOCK lock = 7u;
        assert(InitSpinLock(&lock) == E_OK && lock == 0u);
        assert(SpinLock(&lock) == E_OK && lock == 1u);
        assert(test_smp_irq_mask == mask);
        assert(SpinTryLock(&lock) == E_BUSY && lock == 1u);
        UINT saved = 55u;
        assert(ISpinTryLock(&lock, &saved) == E_BUSY);
        assert(saved == 55u && test_smp_irq_mask == mask && lock == 1u);
        assert(SpinUnlock(&lock) == E_OK && test_smp_irq_mask == mask);
        assert(ISpinLock(&lock, &saved) == E_OK);
        assert(saved == mask && test_smp_irq_mask == 1u && lock == 1u);
        UINT failed = 66u;
        assert(ISpinTryLock(&lock, &failed) == E_BUSY && failed == 66u);
        assert(test_smp_irq_mask == 1u && lock == 1u);
        assert(ISpinUnlock(&lock, saved) == E_OK);
        assert(test_smp_irq_mask == mask && lock == 0u);
        assert(ISpinTryLock(&lock, &saved) == E_OK && saved == mask);
        assert(ISpinUnlock(&lock, saved) == E_OK);
        assert(test_smp_irq_mask == mask && lock == 0u);
    }
}

static void nested(void)
{
    test_smp_irq_mask = 0u;
    T_SPLOCK first = 0u, second = 0u;
    UINT outer, inner;
    assert(ISpinLock(&first, &outer) == E_OK && outer == 0u);
    assert(ISpinLock(&second, &inner) == E_OK && inner == 1u);
    UW value = 3u;
    assert(atomic_inc(&value) == 4u && test_smp_irq_mask == 1u);
    assert(ISpinUnlock(&second, inner) == E_OK && test_smp_irq_mask == 1u);
    assert(ISpinUnlock(&first, outer) == E_OK && test_smp_irq_mask == 0u);
}

static void atomics(void)
{
    for (UINT mask = 0u; mask <= 1u; ++mask) {
        test_smp_irq_mask = mask;
        UW value = UINT32_MAX;
        assert(atomic_inc(&value) == 0u && value == 0u);
        assert(atomic_dec(&value) == UINT32_MAX && value == UINT32_MAX);
        assert(atomic_add(&value, 2u) == 1u && value == 1u);
        assert(atomic_sub(&value, 2u) == UINT32_MAX && value == UINT32_MAX);
        assert(atomic_xchg(&value, 5u) == UINT32_MAX && value == 5u);
        /* Public argument order is (address, replacement, compare). */
        assert(atomic_cmpxchg(&value, 9u, 5u) == 5u && value == 9u);
        assert(atomic_cmpxchg(&value, 4u, 5u) == 9u && value == 9u);
        assert(atomic_bitset(&value, 6u) == 9u && value == 15u);
        assert(atomic_bitclr(&value, 3u) == 15u && value == 12u);
        assert(atomic_sub(&value, 0x80000000u) == 0x8000000cu);
        mp_memory_barrir();
        assert(test_smp_irq_mask == mask);
    }
}

static void wait_paths(void)
{
    test_smp_wait_hook = release_remote;
    for (UINT mask = 0u; mask <= 1u; ++mask) {
        test_smp_irq_mask = mask;
        T_SPLOCK lock = 1u;
        expected_wait_mask = mask;
        assert(SpinLock(&lock) == E_OK);
        assert(test_smp_irq_mask == mask && lock == 1u);
        assert(SpinUnlock(&lock) == E_OK);
        lock = 1u;
        expected_wait_mask = 1u;
        UINT saved;
        assert(ISpinLock(&lock, &saved) == E_OK && saved == mask);
        assert(test_smp_irq_mask == 1u && lock == 1u);
        assert(ISpinUnlock(&lock, saved) == E_OK);
        assert(test_smp_irq_mask == mask);
    }
    assert(waits == 4u);
    test_smp_wait_hook = NULL;
}

static void invalid(void)
{
    T_SPLOCK lock = 0u;
    UINT saved = 99u;
    for (UINT mask = 0u; mask <= 1u; ++mask) {
        test_smp_irq_mask = mask;
        assert(InitSpinLock(NULL) == E_PAR);
        assert(SpinLock(NULL) == E_PAR && SpinTryLock(NULL) == E_PAR);
        assert(SpinUnlock(NULL) == E_PAR);
        assert(ISpinLock(NULL, &saved) == E_PAR && saved == 99u);
        assert(ISpinTryLock(NULL, &saved) == E_PAR && saved == 99u);
        assert(ISpinLock(&lock, NULL) == E_PAR);
        assert(ISpinTryLock(&lock, NULL) == E_PAR);
        assert(ISpinUnlock(NULL, mask) == E_PAR);
        assert(test_smp_irq_mask == mask && lock == 0u);
    }
}

int main(int argc, char **argv)
{
    test_smp_reset();
    if (argc == 1 || strcmp(argv[1], "locks") == 0) {
        locks();
    }
    if (argc == 1 || strcmp(argv[1], "nested") == 0) {
        nested();
    }
    if (argc == 1 || strcmp(argv[1], "atomics") == 0) {
        atomics();
    }
    if (argc == 1 || strcmp(argv[1], "wait") == 0) {
        wait_paths();
    }
    if (argc == 1 || strcmp(argv[1], "invalid") == 0) {
        invalid();
    }
    assert(test_smp_saves == test_smp_restores);
    return 0;
}
