#ifndef PIL_FAKE_CHIP_RESET_H
#define PIL_FAKE_CHIP_RESET_H
#include <stdint.h>
typedef struct { volatile uint32_t chip_reset; } vreg_and_chip_reset_hw_t;
extern vreg_and_chip_reset_hw_t pil_test_chip_reset;
#define vreg_and_chip_reset_hw (&pil_test_chip_reset)
#endif
