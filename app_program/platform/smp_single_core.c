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

/*
 * The atomic family. The kernel's whole SMP utility body, atomics included, sits
 * inside `#if TK_SUPPORT_SMP` in kernel/tkernel/smp_lock.c, so single-core images
 * need these too once application code uses them in every profile. On one core an
 * interrupt-disable window is the equivalent guarantee, and the PRIMASK save and
 * restore keeps a caller's existing mask intact.
 *
 * The return values mirror smp_lock.c exactly: inc/dec/add/sub and bitset/bitclr
 * return the NEW value (bitset/bitclr the PREVIOUS one, as the kernel does), xchg
 * and cmpxchg return the OLD value, and cmpxchg stores only when the old value
 * equals `cmp`.
 */
static UINT shim_irq_save(void)
{
    UINT primask;
    __asm__ volatile("mrs %0, primask" : "=r"(primask));
    __asm__ volatile("cpsid i" ::: "memory");
    return primask;
}

static void shim_irq_restore(UINT primask)
{
    if (0u == primask) {
        __asm__ volatile("cpsie i" ::: "memory");
    }
}

UW atomic_inc(UW *addr)
{
    const UINT primask = shim_irq_save();
    ++*addr;
    shim_irq_restore(primask);
    return *addr;
}

UW atomic_dec(UW *addr)
{
    const UINT primask = shim_irq_save();
    --*addr;
    shim_irq_restore(primask);
    return *addr;
}

UW atomic_add(UW *addr, UW val)
{
    const UINT primask = shim_irq_save();
    *addr += val;
    shim_irq_restore(primask);
    return *addr;
}

UW atomic_sub(UW *addr, UW val)
{
    const UINT primask = shim_irq_save();
    *addr -= val;
    shim_irq_restore(primask);
    return *addr;
}

UW atomic_xchg(UW *addr, UW val)
{
    const UINT primask = shim_irq_save();
    const UW old = *addr;
    *addr = val;
    shim_irq_restore(primask);
    return old;
}

UW atomic_cmpxchg(UW *addr, UW val, UW cmp)
{
    const UINT primask = shim_irq_save();
    const UW old = *addr;
    if (old == cmp) {
        *addr = val;
    }
    shim_irq_restore(primask);
    return old;
}

UW atomic_bitset(UW *addr, UW setptn)
{
    const UINT primask = shim_irq_save();
    const UW old = *addr;
    *addr = old | setptn;
    shim_irq_restore(primask);
    return old;
}

UW atomic_bitclr(UW *addr, UW clrptn)
{
    const UINT primask = shim_irq_save();
    const UW old = *addr;
    *addr = old & ~clrptn;
    shim_irq_restore(primask);
    return old;
}

/* The spelling is retained from the public SMP T-Kernel specification. */
void mp_memory_barrir(void)
{
    __asm__ volatile("" ::: "memory");
}

#endif /* !TK_SUPPORT_SMP */
