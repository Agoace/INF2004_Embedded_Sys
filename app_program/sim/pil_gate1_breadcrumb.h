#ifndef PIL_GATE1_BREADCRUMB_H
#define PIL_GATE1_BREADCRUMB_H
#include <stdint.h>
#define PIL_BC_WORDS 28u
#define PIL_BC_LINE 281u
#define PIL_BC_RAM_BUDGET 800u
#define PIL_BC_MAGIC UINT32_C(0x50424731)
#define PIL_BC_PRODUCER UINT32_C(0x00010001)
#define PIL_BC_SALT UINT32_C(0x9e3779b9)
extern volatile uint32_t pil_bc_live[PIL_BC_WORDS];
extern volatile uint32_t pil_bc_active;
void pil_gate1_bc_begin(uint32_t nonce, uint32_t reason);
void pil_gate1_bc_pil(uint32_t state);
void pil_gate1_bc_tick(uint32_t entry);
const uint8_t *pil_gate1_bc_line(void);
uint32_t pil_gate1_bc_validate(const uint32_t words[PIL_BC_WORDS],
                               uint32_t reason);
void __wrap_drv_watchdog_feed(void);
#endif
