#ifndef SIM_DEVICES_H
#define SIM_DEVICES_H
#include <stdint.h>
#include <stdio.h>
void sim_devices_read(FILE *input);
uint64_t sim_devices_deadline(void);
void sim_devices_tick(void);
void sim_devices_close(void);
#endif
