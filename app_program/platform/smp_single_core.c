/*
 * smp_single_core.c - the SMP utility API for single-core builds.
 *
 * The kernel supplies SpinLock, ISpinLock and friends only when CNF_SMP is set,
 * because on one core the kernel itself needs no cross-core primitive. The
 * migration's application code calls them in every profile so that one source
 * tree serves both: two-core images get the real implementation, and this file
 * supplies the single-core equivalents.
 *
 * The semantics are deliberately the ones the port already uses in its own
 * single-core paths (see hw_claim_lock() in libwifi/.../cyw43_utk_compat.c):
 *
 *   - ISpinLock saves PRIMASK and disables interrupts, ISpinUnlock restores it.
 *     That is what "the holder is not preemptible" means on one core, and it is
 *     the same guarantee the two-core build gets from the hardware spinlock plus
 *     the interrupt-disable window.
 *   - SpinLock disables interrupts and SpinUnlock re-enables them. These two
 *     carry no saved state, so they cannot nest correctly; the code only uses
 *     them where no handler shares the data, which is why the interrupt-
 *     preserving pair exists for everything else.
 *   - SpinTryLock cannot fail on one core: with interrupts masked by the caller
 *     or with no second core, there is no other holder to race.
 *
 * This file is empty except in single-core builds, so a two-core image links
 * exactly one definition of each symbol.
 */
#include <tk/tkernel.h>
#include <tk/smp.h>

#if !TK_SUPPORT_SMP

ER InitSpinLock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    *lock = 0u;
    return E_OK;
}

ER SpinLock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    __asm__ volatile("cpsid i" ::: "memory");
    return E_OK;
}

ER SpinTryLock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    __asm__ volatile("cpsid i" ::: "memory");
    return E_OK;
}

ER SpinUnlock(T_SPLOCK *lock)
{
    if (!lock) {
        return E_PAR;
    }
    __asm__ volatile("cpsie i" ::: "memory");
    return E_OK;
}

ER ISpinLock(T_SPLOCK *lock, UINT *intsts)
{
    if (!lock || !intsts) {
        return E_PAR;
    }
    __asm__ volatile("mrs %0, primask" : "=r"(*intsts));
    __asm__ volatile("cpsid i" ::: "memory");
    return E_OK;
}

ER ISpinTryLock(T_SPLOCK *lock, UINT *intsts)
{
    /* Nothing else can hold it on one core, so this is the same as ISpinLock. */
    return ISpinLock(lock, intsts);
}

ER ISpinUnlock(T_SPLOCK *lock, UINT intsts)
{
    if (!lock) {
        return E_PAR;
    }
    if (0u == intsts) {
        __asm__ volatile("cpsie i" ::: "memory");
    }
    return E_OK;
}

#endif /* !TK_SUPPORT_SMP */
