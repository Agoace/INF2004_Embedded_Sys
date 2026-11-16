/*
 * kernel_lock_stats.c - P1 wrapper around the kernel lock diagnostics.
 * See kernel_lock_stats.h.
 */
#include <tk/tkernel.h>
#include "kernel_lock_stats.h"
#if TK_SUPPORT_SMP
#include "../../kernel/knlinc/smp_lock.h"
#endif

int kls_reset(void)
{
#if TK_SUPPORT_SMP
    knl_kernel_lock_reset_stats();
    return (int)E_OK;
#else
    return (int)E_NOSPT;
#endif
}

int kls_read(rt_bkl_t *out)
{
    if (out == 0) {
        return (int)E_PAR;
    }
#if TK_SUPPORT_SMP
    KNL_LOCK_DIAG d;
    knl_kernel_lock_audit(&d);
    out->valid = 1u;
    out->acquisitions = (uint32_t)d.acquisitions;
    out->contentions = (uint32_t)d.contentions;
    out->max_wait_us = (uint32_t)d.max_wait_us;
    out->max_hold_us = (uint32_t)d.max_hold_us;
    out->max_depth = (uint32_t)d.max_depth;
    out->detached = (uint32_t)d.detached_sections;
    return (int)E_OK;
#else
    out->valid = 0u;
    return (int)E_NOSPT;
#endif
}
