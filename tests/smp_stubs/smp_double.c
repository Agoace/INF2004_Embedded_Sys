/* M1: sequential host model of the kernel's 32-bit SMP utility API.
 * GNU atomics provide word ownership; IRQ state is local to the host thread.
 * No claim is made about RP2040 wait times or scheduler behaviour.
 */
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <tk/smp.h>

_Static_assert(sizeof(UW) == 4u, "kernel word must be 32 bits");
_Thread_local UINT test_smp_irq_mask;
_Thread_local unsigned int test_smp_saves;
_Thread_local unsigned int test_smp_restores;
void (*test_smp_wait_hook)(T_SPLOCK *lock);

UINT test_smp_irq_save(void)
{
    const UINT saved = test_smp_irq_mask;
    test_smp_irq_mask = 1u;
    ++test_smp_saves;
    return saved;
}

void test_smp_irq_restore(UINT saved)
{
    assert(saved <= 1u);
    test_smp_irq_mask = saved;
    ++test_smp_restores;
}

void test_smp_reset(void)
{
    test_smp_irq_mask = 0u;
    test_smp_saves = 0u;
    test_smp_restores = 0u;
    test_smp_wait_hook = NULL;
}

static void aligned(const volatile UW *address)
{
    assert(address != NULL && ((uintptr_t)address & 3u) == 0u);
}

static ER claim(T_SPLOCK *lock)
{
    aligned(lock);
    const UINT saved = test_smp_irq_save();
    UW expected = 0u;
    const bool success = __atomic_compare_exchange_n(
        lock, &expected, 1u, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
    return success ? E_OK : E_BUSY;
}

static void release(T_SPLOCK *lock)
{
    aligned(lock);
    const UINT saved = test_smp_irq_save();
    __atomic_store_n(lock, 0u, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
}

ER InitSpinLock(T_SPLOCK *lock)
{
    if (lock == NULL) {
        return E_PAR;
    }
    release(lock);
    return E_OK;
}

ER SpinTryLock(T_SPLOCK *lock)
{
    return lock == NULL ? E_PAR : claim(lock);
}

ER SpinLock(T_SPLOCK *lock)
{
    if (lock == NULL) {
        return E_PAR;
    }
    while (claim(lock) != E_OK) {
        assert(test_smp_wait_hook != NULL && "blocking on a held host lock");
        test_smp_wait_hook(lock);
    }
    return E_OK;
}

ER SpinUnlock(T_SPLOCK *lock)
{
    if (lock == NULL) {
        return E_PAR;
    }
    release(lock);
    return E_OK;
}

ER ISpinLock(T_SPLOCK *lock, UINT *saved)
{
    if (lock == NULL || saved == NULL) {
        return E_PAR;
    }
    *saved = test_smp_irq_save();
    return SpinLock(lock);
}

ER ISpinTryLock(T_SPLOCK *lock, UINT *saved)
{
    if (lock == NULL || saved == NULL) {
        return E_PAR;
    }
    const UINT previous = test_smp_irq_save();
    const ER result = claim(lock);
    if (result == E_OK) {
        *saved = previous;
    } else {
        test_smp_irq_restore(previous);
    }
    return result;
}

ER ISpinUnlock(T_SPLOCK *lock, UINT saved)
{
    if (lock == NULL) {
        return E_PAR;
    }
    release(lock);
    test_smp_irq_restore(saved);
    return E_OK;
}

UW atomic_add(UW *address, UW value)
{
    aligned(address);
    const UINT saved = test_smp_irq_save();
    const UW result = __atomic_add_fetch(address, value, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
    return result;
}

UW atomic_sub(UW *address, UW value)
{
    return atomic_add(address, 0u - value);
}

UW atomic_inc(UW *address)
{
    return atomic_add(address, 1u);
}

UW atomic_dec(UW *address)
{
    return atomic_sub(address, 1u);
}

UW atomic_xchg(UW *address, UW value)
{
    aligned(address);
    const UINT saved = test_smp_irq_save();
    const UW previous = __atomic_exchange_n(address, value, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
    return previous;
}

UW atomic_cmpxchg(UW *address, UW value, UW compare)
{
    aligned(address);
    const UINT saved = test_smp_irq_save();
    (void)__atomic_compare_exchange_n(address, &compare, value, false,
                                     __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
    return compare;
}

UW atomic_bitset(UW *address, UW pattern)
{
    aligned(address);
    const UINT saved = test_smp_irq_save();
    const UW previous = __atomic_fetch_or(address, pattern, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
    return previous;
}

UW atomic_bitclr(UW *address, UW pattern)
{
    aligned(address);
    const UINT saved = test_smp_irq_save();
    const UW previous = __atomic_fetch_and(address, ~pattern, __ATOMIC_SEQ_CST);
    test_smp_irq_restore(saved);
    return previous;
}

void mp_memory_barrir(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}
