/* Host-only SMP API double. Ownership semantics match the SMP kernel,
 * including in UP tests: this does not copy the known UP shim defects.
 */
#ifndef TEST_SMP_H
#define TEST_SMP_H
#include <tk/tkernel.h>
#ifndef E_BUSY
#define E_BUSY (-65)
#endif
#ifndef E_TMOUT
#define E_TMOUT (-50)
#endif

typedef volatile UW T_SPLOCK;
extern _Thread_local UINT test_smp_irq_mask;
extern _Thread_local unsigned int test_smp_saves;
extern _Thread_local unsigned int test_smp_restores;
/* Optional deterministic release of a simulated remote holder. Without a
 * hook, blocking on a held lock fails an assertion, not a CTest hang. */
extern void (*test_smp_wait_hook)(T_SPLOCK *lock);
UINT test_smp_irq_save(void);
void test_smp_irq_restore(UINT saved);
void test_smp_reset(void);
ER InitSpinLock(T_SPLOCK *lock);
ER SpinLock(T_SPLOCK *lock);
ER SpinTryLock(T_SPLOCK *lock);
ER SpinUnlock(T_SPLOCK *lock);
ER ISpinLock(T_SPLOCK *lock, UINT *saved);
ER ISpinTryLock(T_SPLOCK *lock, UINT *saved);
ER ISpinUnlock(T_SPLOCK *lock, UINT saved);
UW atomic_inc(UW *address);
UW atomic_dec(UW *address);
UW atomic_add(UW *address, UW value);
UW atomic_sub(UW *address, UW value);
UW atomic_xchg(UW *address, UW value);
UW atomic_cmpxchg(UW *address, UW value, UW compare);
UW atomic_bitset(UW *address, UW pattern);
UW atomic_bitclr(UW *address, UW pattern);
void mp_memory_barrir(void);
#endif
