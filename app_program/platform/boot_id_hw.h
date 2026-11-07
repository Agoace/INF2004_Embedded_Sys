/* boot_id_hw.h - bounded ROSC sampling, with no SDK runtime dependency. */
#ifndef BOOT_ID_HW_H
#define BOOT_ID_HW_H

#include <stdbool.h>
#include <stdint.h>

/* Return a nonzero random label, or false after a bounded source failure. */
bool boot_id_hw_generate(uint32_t *out);

#endif
