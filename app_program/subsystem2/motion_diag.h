/* Optional timing observations. No scheduling or motor-control decisions. */
#ifndef MOTION_DIAG_H
#define MOTION_DIAG_H
#ifndef APP_DIAG
#define APP_DIAG 0
#endif
#define MOTION_DIAG_IDLE        0u
#define MOTION_DIAG_CALIBRATE   1u
#define MOTION_DIAG_START_DELAY 2u
#define MOTION_DIAG_RUN         3u
#define MOTION_DIAG_STOP        4u
#if APP_DIAG
#include <stdint.h>
void motion_diag_phase(uint32_t phase);
void motion_diag_dump(int active);
void motion_diag_print(void);
#else
#define motion_diag_phase(phase) ((void)0)
#define motion_diag_dump(active) ((void)0)
#define motion_diag_print()      ((void)0)
#endif
#endif
