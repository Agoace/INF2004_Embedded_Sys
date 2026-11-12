#ifndef TEMPLATE_TEST_TKERNEL_H
#define TEMPLATE_TEST_TKERNEL_H
/* Use the same kernel ABI double as the real app stack registry. */
#include "../../mtk_stubs/tk/tkernel.h"
#define USE_OBJECT_NAME 1
#define TA_RNG1 0x100u
#define TRUE 1
#define FALSE 0
typedef int BOOL;
void template_mock_di(UINT *irq);
void template_mock_ei(UINT irq);
#endif
