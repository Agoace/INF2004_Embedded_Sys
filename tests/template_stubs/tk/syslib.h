#ifndef TEMPLATE_TEST_SYSLIB_H
#define TEMPLATE_TEST_SYSLIB_H
#include "tkernel.h"
/* Model the RP2040 32-bit SP rather than truncate a 64-bit host pointer. */
uint32_t template_mock_stack_pointer(unsigned int level);
#define __builtin_frame_address(level) template_mock_stack_pointer(level)
#endif
