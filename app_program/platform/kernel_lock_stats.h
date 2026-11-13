/*
 * kernel_lock_stats.h - P1 report-only access to the SMP big-kernel-lock
 * statistics. The only application file that includes the kernel header is
 * kernel_lock_stats.c. Single-core builds return E_NOSPT.
 */
#ifndef KERNEL_LOCK_STATS_H
#define KERNEL_LOCK_STATS_H

#include "run_timing.h"

/* Clear the counters and maxima. A core-1 update can race the clear;
 * the figures are for reports only. Returns E_OK or E_NOSPT. */
int kls_reset(void);
/* Copy the figures into out (valid = 1; run_ms is not set).
 * Returns E_OK, E_PAR or E_NOSPT (out->valid = 0). */
int kls_read(rt_bkl_t *out);

#ifdef KLS_HOST_TEST
/* Host double controls (tests/smp_stubs/kernel_lock_double.c). */
#define TEST_KLS_ACQUISITIONS 4242u
extern unsigned int test_kls_resets;
#endif

#endif
