#ifndef LINE_SERVICE_H
#define LINE_SERVICE_H
#include "line_position.h"
#include "drv_line.h"
typedef struct {
    lp_t line;
    uint16_t raw[3], raw_bc, norm_peak; /* raw[2] stays zero for pure line logic */
    vc_event_t event;
    bool valid;
    uint32_t timestamp_us, sequence, processed, cpu_us, sample_max_us;
    drv_line_stats_t acquisition;
} line_snapshot_t;
int line_service_init(void);
int line_service_start(void);
bool line_service_running(void);
/* Short inherited-mutex copy; only vehicle consumes the latched event. */
int line_snapshot_read(line_snapshot_t *out, bool consume_event);
/* Transport-independent bounded copy; never consumes events or waits.
 * E_TMOUT busy, E_OBJ unready; failed output unchanged. */
int line_snapshot_try_read(line_snapshot_t *out);
int line_calibration_begin(void);
lp_calib_result_t line_calibration_end(void);
int line_calibration_abort(void); /* retain prior learned ends */
int line_tracking_reset(void);
#endif
