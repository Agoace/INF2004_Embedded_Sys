#ifndef CYW43_UTK_H
#define CYW43_UTK_H

#include <stdint.h>

typedef struct {
    volatile int32_t init_result;
    volatile uint32_t init_complete;
    volatile uint32_t ready;
    volatile uint32_t scan_active;
    volatile uint32_t scan_complete;
    volatile uint32_t scan_count;
    volatile int32_t strongest_rssi;
    volatile uint32_t poll_count;
    volatile uint32_t owner_prc;
    uint8_t mac[6];
    /* Only meaningful on a NET=lwip build -- see cyw43_utk_task(). link_status
     * mirrors cyw43_tcpip_link_status()'s CYW43_LINK_* values (cyw43.h):
     * negative = failed, 0 = down, 1 = joining, 2 = no IP, 3 = UP. */
    volatile int32_t link_status;
    volatile uint32_t ip_addr;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
void cyw43_utk_task(int32_t stacd, void *exinf);
/* TM_MQTT only: Wi-Fi task context, after sys_check_timeouts(), link UP.
 * Only this task may call lwIP. An application may override the weak demo. */
void cyw43_utk_app_poll(void);

/* Wall time, including interruption/preemption. Not CPU load. gap threshold
 * is strictly >10000 us. All copies and updates take one cross-core spinlock,
 * because the poll path and the readers can run on different cores after the
 * placement move; a core-local interrupt mask protects neither. */
typedef struct {
    uint32_t polls;
    uint32_t gaps;
    uint32_t gap_max_us;
    uint32_t gaps_over_10000;
    uint32_t work_max_us;
    uint32_t work_total_us;
    uint32_t probe_max_us;
} T_CYW43_TIMING;

int32_t cyw43_utk_timing_read(T_CYW43_TIMING *out);
int32_t cyw43_utk_timing_window_begin(void);
int32_t cyw43_utk_timing_window_end(T_CYW43_TIMING *out);
int32_t cyw43_utk_start_status(void);
/* Internal Wi-Fi task instrumentation, not application scheduling hooks. */
void cyw43_utk_poll_begin(void);
void cyw43_utk_poll_end(void);

#endif
