#ifndef VEHICLE_TASK_H
#define VEHICLE_TASK_H
#include <stdint.h>
int vehicle_task_start(void);
/* One bounded atomic load, independent of MQTT. Odd during the entire
 * CALIBRATE spin/pause/abort/home sequence; changes on every episode. */
uint32_t vehicle_calibration_epoch(void);
#endif
