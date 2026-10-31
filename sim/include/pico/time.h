#ifndef SIM_PICO_TIME_H
#define SIM_PICO_TIME_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint64_t us; } absolute_time_t;
absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t time);
uint32_t time_us_32(void);
bool time_reached(absolute_time_t deadline);
absolute_time_t delayed_by_us(absolute_time_t time, uint64_t us);
absolute_time_t make_timeout_time_ms(uint32_t ms);
void sleep_ms(uint32_t ms);
void sleep_us(uint64_t us);
#endif
