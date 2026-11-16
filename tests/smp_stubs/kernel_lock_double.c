/* P1 host double for app_program/platform/kernel_lock_stats.c. */
#define KLS_HOST_TEST 1
#include <tk/tkernel.h>
#include "../../app_program/platform/kernel_lock_stats.h"

unsigned int test_kls_resets;

int kls_reset(void)
{
    ++test_kls_resets;
    return (int)E_OK;
}

int kls_read(rt_bkl_t *out)
{
    if (out == 0) {
        return (int)E_PAR;
    }
    *out = (rt_bkl_t){.valid = 1u, .acquisitions = TEST_KLS_ACQUISITIONS};
    return (int)E_OK;
}
