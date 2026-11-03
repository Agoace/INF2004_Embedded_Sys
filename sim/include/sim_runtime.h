#ifndef SIM_RUNTIME_H
#define SIM_RUNTIME_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
extern uint16_t sim_floor[3];
uint64_t sim_now(void);
void sim_event(const char *kind, const char *fields, ...)
    __attribute__((format(printf, 2, 3)));
void sim_event_at(uint64_t time_us, const char *kind, const char *fields, ...)
    __attribute__((format(printf, 3, 4)));
int sim_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
_Noreturn void sim_error(const char *message);
int firmware_main(void);
#endif
