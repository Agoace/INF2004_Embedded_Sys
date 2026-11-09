/* Actual TEST observer with encoder, sensor and button doubles. */
#include "../app_program/subsystem3/barcode_capture.h"
#include "../app_program/subsystem3/barcode_test.h"
#include "../app_program/subsystem3/line_service.h"
#include "../app_program/subsystem2/motion.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *scenario;
static uint32_t now;
static uint32_t first_drive_ms;
static uint32_t second_press_ms;
static uint32_t stop_attempt_ms;
static uint32_t held_done_ms;
static int16_t command;
static int16_t sampled_speed;
static int64_t position_scaled;
static unsigned int ranges;
static unsigned int results;
static unsigned int frames;
static unsigned int starts;
static unsigned int dumps;
static unsigned int stop_calls;
static unsigned int estop_calls;
static unsigned int adc_rows;
static bool button_ready;
static bool injected;
static bool failed_stop;
static bool start_refused;
static jmp_buf finished;

static bool mode(const char *name)
{
    return strcmp(scenario, name) == 0;
}

static uint16_t optical(void)
{
    int64_t x = position_scaled / 12414 - 110000;
    if (x < 0) {
        return 160u;
    }
    static const char *const chars[] = {"nwnnwnwnn", "nnwnwwnnn", "nwnnwnwnn"};
    for (unsigned int c = 0u; c < 3u; ++c) {
        for (unsigned int i = 0u; i < 9u; ++i) {
            const int width = chars[c][i] == 'w' ? 17500 : 5800;
            if (x < width) {
                return (i & 1u) == 0u ? 1500u : 160u;
            }
            x -= width;
        }
        if (c < 2u) {
            if (x < 5800) {
                return 160u;
            }
            x -= 5800;
        }
    }
    return 160u;
}

static void advance(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; ++i) {
        ++now;
        if (!mode("timeout")) {
            position_scaled += (int64_t)command * 12414;
        }
        if (now % 10u == 0u) {
            sampled_speed = command;
        }
        const uint16_t raw =
            barcode_capture_active() ? optical() : (uint16_t)(160u + now % 8u);
        barcode_capture_observe(now * 1000u, raw);
    }
}

uint32_t drv_time_us(void)
{
    return now * 1000u;
}

uint32_t drv_time_ms(void)
{
    return now;
}

int motion_set_speed(int16_t left, int16_t right)
{
    assert(left == right && (left == 100 || left == -100));
    assert(now >= 3800u && barcode_capture_active() && ranges == 3u);
    if (mode("command_fail") && !injected) {
        injected = true;
        return E_SYS;
    }
    if (command == 0) {
        ++starts;
        if (starts == 1u) {
            assert(left == 100);
            first_drive_ms = now;
        } else {
            assert(starts == 2u && left == -100 && dumps == 1u);
        }
    }
    command = left;
    return E_OK;
}

int motion_stop(void)
{
    ++stop_calls;
    if (mode("stop_retry") && starts > 0u && !failed_stop) {
        if (stop_attempt_ms == 0u) {
            stop_attempt_ms = now;
        }
        if (now - stop_attempt_ms < 100u) {
            return E_SYS;
        }
        failed_stop = true;
    }
    command = 0;
    return E_OK;
}

int motion_estop(void)
{
    ++estop_calls;
    assert(mode("stop_retry") && command != 0 && dumps == 0u);
    return E_SYS;
}

int motion_snapshot_try_read(motion_status_t *out, uint32_t *stamp)
{
    if (mode("missing_motor") && starts > 0u && now - first_drive_ms >= 100u &&
        now - first_drive_ms < 300u) {
        return E_TMOUT;
    }
    memset(out, 0, sizeof(*out));
    out->counts[0] = (int32_t)(position_scaled / 1000000);
    out->counts[1] = out->counts[0];
    out->speed_mm_s[0] = sampled_speed;
    out->speed_mm_s[1] = sampled_speed;
    out->duty_permille[0] = sampled_speed == 0 ? 0 : 200;
    out->duty_permille[1] = out->duty_permille[0];
    out->watchdog_armed = true;
    if (mode("moving_start") && starts == 0u && now >= 3800u && now < 3900u) {
        out->speed_mm_s[0] = 20;
        out->speed_mm_s[1] = 20;
    }
    if (mode("motion_fault") && starts > 0u && now - first_drive_ms >= 100u) {
        out->faults = 1u;
    }
    *stamp = now / 10u * 10000u;
    if (mode("stale_motor") && starts > 0u && now - first_drive_ms >= 100u &&
        now - first_drive_ms < 300u) {
        *stamp -= 40000u;
    }
    return E_OK;
}

int line_snapshot_try_read(line_snapshot_t *out)
{
    if (mode("sensor_fault") && starts > 0u && now - first_drive_ms >= 100u) {
        return E_TMOUT;
    }
    memset(out, 0, sizeof(*out));
    out->timestamp_us = drv_time_us();
    out->valid = true;
    out->raw_bc = optical();
    return E_OK;
}

void drv_line_stats(drv_line_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    if (mode("acquisition_loss") && starts > 0u &&
        now - first_drive_ms >= 100u) {
        out->dropped = 1u;
    }
}

void barcode_hw_button_init(void)
{
    button_ready = true;
}

bool barcode_hw_button_down(void)
{
    assert(button_ready);
    if (starts > 0u && now - first_drive_ms >= 100u &&
        now - first_drive_ms < 160u) {
        return true; /* GP20 must not restart ranges during a pass. */
    }
    static const uint32_t presses[] = {100u, 1300u, 2500u};
    for (size_t i = 0u; i < sizeof(presses) / sizeof(presses[0]); ++i) {
        if (now >= presses[i] && now < presses[i] + 60u) {
            return true;
        }
    }
    return false;
}

bool barcode_hw_drive_down(void)
{
    assert(button_ready);
    if (now < 200u) {
        return true; /* A button held at boot cannot start motion. */
    }
    if (mode("held") && first_drive_ms != 0u) {
        return true;
    }
    if (now >= 500u && now < 560u) {
        return true; /* Cannot drive before all three contrast ranges. */
    }
    if (now >= 3800u && now < 3860u) {
        return true;
    }
    if (mode("button_stop") && starts > 0u && now - first_drive_ms >= 200u &&
        now - first_drive_ms < 260u) {
        return true;
    }
    return second_press_ms > 0u && now >= second_press_ms &&
           now < second_press_ms + 60u;
}

ID tk_get_tid(void)
{
    return 2;
}

ID tk_cre_cyc(const T_CCYC *c)
{
    assert(c->cyctim == 10u && c->cycphs == 10u && c->cychdr != NULL);
    return 3;
}

ER tk_sta_cyc(ID id)
{
    assert(id == 3);
    return E_OK;
}

ER tk_wup_tsk(ID id)
{
    assert(id == 2);
    return E_OK;
}

INT tk_can_wup(ID id)
{
    assert(id == TSK_SELF);
    if (mode("missed_wake") && starts > 0u && !injected &&
        now - first_drive_ms >= 100u) {
        injected = true;
        return 1;
    }
    return 0;
}

ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == 100u && now < 40000u);
    if (held_done_ms > 0u && now - held_done_ms > 200u) {
        assert(starts == 1u && dumps == 1u && command == 0);
        longjmp(finished, 1);
    }
    advance(10u);
    if (mode("wake_fault") && starts > 0u && !injected &&
        now - first_drive_ms >= 100u) {
        injected = true;
        return E_TMOUT;
    }
    if (mode("capacity") && starts > 0u && !injected) {
        injected = true;
        for (uint32_t i = 0u; i < BC_MAX_SAMPLES * 2u; ++i) {
            barcode_capture_observe((now + i) * 1000u, optical());
        }
    }
    return E_OK;
}

ER tk_dly_tsk(RELTIM ms)
{
    if (mode("wake_fault") && ms == 10u) {
        assert(command != 0);
    } else {
        assert(command == 0);
    }
    advance(ms);
    return E_OK;
}

int tm_printf(const UB *format, ...)
{
    char text[640];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(text, sizeof(text), (const char *)format, args);
    va_end(args);
    if (strstr(text, "START REFUSED") != NULL) {
        start_refused = true;
    }
    if (mode("moving_start") && start_refused &&
        strstr(text, "[BC] STOP CONFIRMED; next") != NULL) {
        assert(starts == 0u && command == 0 && stop_calls > 2u);
        assert(strstr(text, "inhibited 1") != NULL && dumps == 0u);
        longjmp(finished, 1);
    }
    if (strstr(text, "[BC_RANGE]") != NULL) {
        ++ranges;
    }
    if (strstr(text, "[BC_FRAME]") != NULL) {
        assert(command == 0 && !barcode_capture_active());
        assert(strstr(text, "text \"F\" cmd 0") != NULL);
        ++frames;
    }
    if (strstr(text, "[BC_RESULT]") != NULL) {
        assert(command == 0 && !barcode_capture_active() && ranges == 3u);
        ++results;
    }
    if (strstr(text, "[BC_DUMP] BEGIN") != NULL) {
        assert(command == 0 && sampled_speed == 0 && !barcode_capture_active());
        assert(results == (dumps + 1u) * 2u);
        adc_rows = 0u;
    }
    if (strncmp(text, "BC_ADC,", 7u) == 0 &&
        strncmp(text, "BC_ADC,index", 12u) != 0) {
        assert(command == 0 && !barcode_capture_active());
        ++adc_rows;
    }
    if (strstr(text, "[BC_DUMP] END") != NULL) {
        ++dumps;
        assert(adc_rows > 0u && stop_calls > 2u);
    }
    if (strstr(text, "[BC] STOP CONFIRMED; next") != NULL && dumps > 0u) {
        assert(command == 0 && !barcode_capture_active());
        if (mode("normal") && dumps == 1u) {
            assert(strstr(text, "inhibited 0") != NULL && frames == 2u);
            second_press_ms = now + 100u;
        } else if (mode("held")) {
            held_done_ms = now;
        } else {
            if (mode("normal")) {
                assert(dumps == 2u && starts == 2u && frames == 4u);
                assert(position_scaled / 12414 >= -2000 &&
                       position_scaled / 12414 <= 2000);
            } else if (mode("stop_retry")) {
                assert(estop_calls >= 5u && failed_stop);
                assert(strstr(text, "inhibited 1") != NULL);
            } else if (!mode("button_stop")) {
                assert(strstr(text, "inhibited 1") != NULL);
            }
            longjmp(finished, 1);
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    scenario = argc > 1 ? argv[1] : "normal";
    if (setjmp(finished) == 0) {
        barcode_test();
        assert(0 && "observer must wait for explicit operator action");
    }
    return 0;
}
