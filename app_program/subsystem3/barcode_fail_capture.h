/* Opt-in ROBOT diagnostic hooks. No public board/value layout changes. */
#ifndef BARCODE_FAIL_CAPTURE_H
#define BARCODE_FAIL_CAPTURE_H
#include "barcode_window.h"
#include "barcode_live_producer.h"
void bcfc_epoch(uint32_t epoch);
/* Source poll: control mode and drv_time_us at that poll. */
void bcfc_control(uint32_t mode, uint32_t now_us);
void bcfc_observe(const barcode_window_t *w, barcode_window_adc_t a,
                  float mm, const barcode_window_motor_t *upper);
void bcfc_start(const barcode_window_t *w, barcode_window_adc_t a,
                float mm, const barcode_window_motor_t *upper);
void bcfc_bad_adc(barcode_window_adc_t a, uint16_t fault);
void bcfc_reject(const barcode_window_t *w, uint32_t reason);
void bcfc_close(const barcode_window_t *w);
void bcfc_terminal(const barcode_live_frame_t *frame);
void bcfc_dump_begin(void);
void bcfc_dump(uint32_t epoch, uint32_t index);
void bcfc_dump_remaining(void);
#endif
