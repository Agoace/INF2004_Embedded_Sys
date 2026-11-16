/* Placement seam for subsystem tests that do not link the stack registry.
 * Registry tests link the real app_prc_core0/1 implementation instead.
 */
#include <tk/tkernel.h>
#ifndef APP_FORCE_CORE0
#define APP_FORCE_CORE0 0
#endif
UW app_prc_core0(void)
{
    return TP_PRC1;
}

UW app_prc_core1(void)
{
#if TK_SUPPORT_SMP && TK_MAX_CORE > 1 && !APP_FORCE_CORE0
    return TP_PRC1 + 1u;
#else
    return TP_PRC1;
#endif
}
