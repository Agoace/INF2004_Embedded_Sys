#ifndef VEHICLE_TASK_H
#define VEHICLE_TASK_H
#include <stdint.h>
#include "../subsystem4/imu_hump.h"
#include "../subsystem4/imu_hump_diag.h"
int vehicle_task_start(void);
/* One bounded atomic load, independent of MQTT. Odd during the entire
 * CALIBRATE spin/pause/abort/home sequence; changes on every episode. */
uint32_t vehicle_calibration_epoch(void);
/* One bounded atomic load. Odd from run start until the run summary values
 * are taken; changes on every run. TEST/BENCH return 0. */
uint32_t vehicle_run_epoch(void);
/* Diagnostic-only context; LINE epoch is 0 outside active LINE. */
uint32_t vehicle_run_start_ms(void);
uint32_t vehicle_line_run_epoch(void);
/* Hump result of the last finished run (USB DUMP). E_OK copy, E_NOEXS no
 * run yet, E_TMOUT busy, E_PAR null. Never waits. */
int vehicle_hump_try_read(imu_hump_summary_t *out);
int vehicle_hump_diag_try_read(imu_hump_diag_report_t *out);
#endif
