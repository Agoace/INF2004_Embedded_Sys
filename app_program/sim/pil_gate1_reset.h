#ifndef PIL_GATE1_RESET_H
#define PIL_GATE1_RESET_H
#include <stdint.h>
#define PIL_GATE1_RESET_LINE 122u
typedef struct {
    uint32_t reason, chip_reset, scratch[8];
} pil_gate1_reset_t;
/* Read-only SDK boundary; call before native startup changes scratch state. */
void pil_gate1_reset_snapshot(pil_gate1_reset_t *out);
/* Fixed122 bytes, including literal LF; no NUL, CRC or D1 envelope. */
void pil_gate1_reset_format(const pil_gate1_reset_t *reset, uint32_t nonce,
                            uint8_t out[PIL_GATE1_RESET_LINE]);
#endif
