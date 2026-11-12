/*
 * UP implementation of the public SMP utility API (review M7).
 *
 * Only the claim/RMW is IRQ-masked for plain locks. Like the SMP kernel,
 * plain locks remain preemptible while held: same-core preempting holders
 * must use ISpinLock instead. Lock words are aligned 32-bit values, 0/1.
 * ISpinLock retains the caller's saved mask until ISpinUnlock releases.
 * All publication barriers are real DMB instructions, not compiler hints.
 *
 * This unit is empty in SMP builds; the kernel supplies those definitions.
 */
#include <tk/tkernel.h>
#include <tk/smp.h>

#if !TK_SUPPORT_SMP
#include "smp_single_core_arch.h"

static UINT shim_irq_save(void)
{
    const UINT saved = shim_arch_irq_get();
    shim_arch_irq_disable();
    return saved;
}

static void shim_irq_restore(UINT saved)
{
    shim_arch_irq_set(saved);
}

/* Caller masks interrupts. No other UP task or ordinary IRQ can change the
 * word between the read and store. This is not cross-core RMW exclusion. */
static ER shim_claim_masked(T_SPLOCK *lock)
{
    ER result = E_BUSY;
    shim_arch_barrier();
    if (*lock == 0u) {
        *lock = 1u;
        result = E_OK;
    }
    shim_arch_barrier();
    return result;
}

static ER shim_try_claim(T_SPLOCK *lock)
{
    const UINT saved = shim_irq_save();
    const ER result = shim_claim_masked(lock);
    shim_irq_restore(saved);
    return result;
}

/* Aligned stores are indivisible, as in knl_arch_atomic_store. Do not change
 * the caller's mask: release is also used by the plain lock API. */
static void shim_release(T_SPLOCK *lock)
{
    shim_arch_barrier();
    *lock = 0u;
    shim_arch_barrier();
}

ER InitSpinLock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    shim_release(lock);
    return E_OK;
}

ER SpinLock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    while (shim_try_claim(lock) != E_OK) {
        shim_arch_relax();
    }
    return E_OK;
}

ER SpinTryLock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    return shim_try_claim(lock);
}

ER SpinUnlock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    shim_release(lock);
    return E_OK;
}

ER ISpinLock(T_SPLOCK *lock, UINT *intsts)
{
    if (!lock || !intsts) {
        return E_PAR;
    }
    *intsts = shim_irq_save();
    while (shim_claim_masked(lock) != E_OK) {
        shim_arch_relax();
    }
    return E_OK;
}

ER ISpinTryLock(T_SPLOCK *lock, UINT *intsts)
{
    if (!lock || !intsts) {
        return E_PAR;
    }
    const UINT saved = shim_irq_save();
    const ER result = shim_claim_masked(lock);
    if (result == E_OK) {
        *intsts = saved;
    } else {
        shim_irq_restore(saved);
    }
    return result;
}

ER ISpinUnlock(T_SPLOCK *lock, UINT intsts)
{
    if (!lock) {
        return E_PAR;
    }
    shim_release(lock);
    shim_irq_restore(intsts);
    return E_OK;
}

static UINT shim_atomic_enter(void)
{
    const UINT saved = shim_irq_save();
    shim_arch_barrier();
    return saved;
}

static void shim_atomic_leave(UINT saved)
{
    shim_arch_barrier();
    shim_irq_restore(saved);
}

/* Capture return values before restoring IRQs: an ISR can update the same
 * word as soon as it becomes runnable. Arithmetic wraps modulo 2^32. */
UW atomic_inc(UW *addr)
{
    return atomic_add(addr, 1u);
}

UW atomic_dec(UW *addr)
{
    return atomic_sub(addr, 1u);
}

UW atomic_add(UW *addr, UW val)
{
    const UINT saved = shim_atomic_enter();
    const UW result = *addr + val;
    *addr = result;
    shim_atomic_leave(saved);
    return result;
}

UW atomic_sub(UW *addr, UW val)
{
    const UINT saved = shim_atomic_enter();
    const UW result = *addr - val;
    *addr = result;
    shim_atomic_leave(saved);
    return result;
}

UW atomic_xchg(UW *addr, UW val)
{
    const UINT saved = shim_atomic_enter();
    const UW old = *addr;
    *addr = val;
    shim_atomic_leave(saved);
    return old;
}

UW atomic_cmpxchg(UW *addr, UW val, UW cmp)
{
    const UINT saved = shim_atomic_enter();
    const UW old = *addr;
    if (old == cmp) {
        *addr = val;
    }
    shim_atomic_leave(saved);
    return old;
}

UW atomic_bitset(UW *addr, UW setptn)
{
    const UINT saved = shim_atomic_enter();
    const UW old = *addr;
    *addr = old | setptn;
    shim_atomic_leave(saved);
    return old;
}

UW atomic_bitclr(UW *addr, UW clrptn)
{
    const UINT saved = shim_atomic_enter();
    const UW old = *addr;
    *addr = old & ~clrptn;
    shim_atomic_leave(saved);
    return old;
}

/* Retain the spelling of the public SMP T-Kernel specification. */
void mp_memory_barrir(void)
{
    shim_arch_barrier();
}
#endif /* !TK_SUPPORT_SMP */
