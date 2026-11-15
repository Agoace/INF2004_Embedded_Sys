#ifndef PIL_GATE1_RUNTIME_H
#define PIL_GATE1_RUNTIME_H
#include <stdint.h>
/* Core0 native hooks, used only by the explicit Gate1 image. */
void pil_gate1_native_grid(uint32_t at_us);
void pil_gate1_native_alarm(void);
uint32_t pil_gate1_native_entry(void);
void pil_gate1_motor_seen(int16_t left, int16_t right, uint16_t servo);
void pil_gate1_motor_stopped(void);
#endif
