/* RP2040 instructions used by the UP shim; no public lock logic here. */
#ifndef SMP_SINGLE_CORE_ARCH_H
#define SMP_SINGLE_CORE_ARCH_H

#if defined(APP_SMP_SHIM_HOST_TEST)
#if defined(__arm__) || defined(__thumb__)
#error "Host shim hooks must not be used in firmware"
#endif
extern UINT shim_test_irq_get(void);
extern void shim_test_irq_disable(void);
extern void shim_test_irq_set(UINT mask);
extern void shim_test_barrier(void);
extern void shim_test_relax(void);
#define shim_arch_irq_get shim_test_irq_get
#define shim_arch_irq_disable shim_test_irq_disable
#define shim_arch_irq_set shim_test_irq_set
#define shim_arch_barrier shim_test_barrier
#define shim_arch_relax shim_test_relax
#else
static inline UINT shim_arch_irq_get(void)
{
    UINT mask;
    __asm__ volatile("mrs %0, primask" : "=r"(mask));
    return mask;
}
static inline void shim_arch_irq_disable(void)
{
    __asm__ volatile("cpsid i" ::: "memory");
}
static inline void shim_arch_irq_set(UINT mask)
{
    __asm__ volatile("msr primask, %0" :: "r"(mask) : "memory");
}
static inline void shim_arch_barrier(void)
{
    __asm__ volatile("dmb" ::: "memory");
}
static inline void shim_arch_relax(void)
{
    __asm__ volatile("" ::: "memory");
}
#endif
#endif
