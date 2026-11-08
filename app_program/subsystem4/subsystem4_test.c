/* The priority-14 IMU task owns the bus/core. TEST only observes it.
 * IMU_STREAM additionally offers an operator-triggered bounded drive job;
 * no terrain result commands motion. VIN/lifted-first checks are required. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "subsystem4.h"
#if APP_TEST
#include "imu_service.h"
#include "imu_hw.h"
#include "imu_mark.h"
#include "imu_drive.h"
#include "../platform/drv_time.h"
#include "../platform/app_task.h"
#include "../subsystem2/motion.h"
#include <math.h>

static ID runner;

static void release(void *context)
{
    (void)context;
    (void)tk_wup_tsk(runner);
}

/* Console-only decimal formatting; keep all formatting out of IMU work.
 * Clamp the DISPLAY conversion only, not the core's relative height. */
static void print_mm(const char *name, float mm)
{
    const float magnitude = fabsf(mm);
    const uint32_t scaled = !isfinite(magnitude) ? 0u :
        magnitude >= 2147483.0f ? UINT32_C(2147483000) :
        (uint32_t)(magnitude * 1000.0f);

    tm_printf((UB *)" %s %s%lu.%03lu", name, mm < 0.0f ? "-" : "",
              (unsigned long)(scaled / 1000u),
              (unsigned long)(scaled % 1000u));
}

static void report(const imu_snapshot_t *snapshot)
{
    const imu_core_output_t *v = &snapshot->value;

    tm_printf((UB *)"[S4] t_ms %lu valid %u motion %u "
              "raw_mg %ld %ld %ld cal_mg %ld %ld %ld "
              "raw_ddeg %ld %ld pitch_ddeg %ld roll_ddeg %ld "
              "filt_ddeg %ld %ld corrected_ddeg %ld "
              "v_mm_s %ld a_mm_s2 %ld cal %u faults %lu",
              (unsigned long)snapshot->timestamp_ms,
              (unsigned int)v->valid, (unsigned int)v->motion_valid,
              (long)snapshot->raw.accel_mg[0],
              (long)snapshot->raw.accel_mg[1],
              (long)snapshot->raw.accel_mg[2],
              (long)v->accel_mg[0], (long)v->accel_mg[1],
              (long)v->accel_mg[2], (long)v->raw_pitch_ddeg,
              (long)v->raw_roll_ddeg, (long)v->pitch_ddeg,
              (long)v->roll_ddeg, (long)v->filtered_pitch_ddeg,
              (long)v->filtered_roll_ddeg, (long)v->corrected_pitch_ddeg,
              (long)v->speed_mm_s, (long)v->forward_accel_mm_s2,
              (unsigned int)v->calibrated,
              (unsigned long)snapshot->probe.faults);
    print_mm("travel_mm", v->travel_mm);
    print_mm("distance_mm", v->distance_mm);
    print_mm("height_mm", v->height_mm);
    print_mm("peak_mm", v->peak_mm);
    print_mm("zR_mm", v->right_mm);
    print_mm("zR_avg_mm", v->right_smoothed_mm);
    print_mm("zR_peak_mm", v->right_peak_mm);
    print_mm("zL_mm", v->left_mm);
    print_mm("zL_peak_mm", v->left_peak_mm);
    print_mm("zC_mm", v->caster_mm);
    print_mm("zC_peak_mm", v->caster_peak_mm);
    print_mm("left_residual_mm", v->left_residual_mm);
    tm_printf((UB *)" eligible %u corrected_roll_ddeg %ld",
              (unsigned int)v->support_peak_eligible,
              (long)v->corrected_roll_ddeg);
    tm_printf((UB *)"\n");
}

#if APP_IMU_STREAM
/* This is the ONLY drive client here, compiled out of ordinary TEST/robot.
 * VIN and a clear test area are operator preconditions, not sensed inputs. */
static void drive_update(imu_drive_t *job, bool *stop_pending, bool press,
                          bool observed, const imu_snapshot_t *s,
                          bool wake_fault)
{
    motion_status_t motor;
    uint32_t stamp;
    const bool motor_ok = motion_snapshot_try_read(&motor, &stamp) == E_OK;
    const uint32_t now = drv_time_us();
    const bool imu_ok = observed && s->value.valid && s->value.calibrated &&
        s->value.motion_valid && s->raw.mag_valid && s->probe.faults == 0u &&
        (uint32_t)(now - s->timestamp_us) <= 30000u;
    const bool fault = *stop_pending || wake_fault || !imu_ok || !motor_ok ||
        (motor_ok && ((uint32_t)(now - stamp) > 30000u ||
                     motor.faults != 0u || !motor.watchdog_armed));
    const bool was_active = job->active;
    const int16_t speed = imu_drive_step(job, drv_time_ms(),
        observed ? s->value.travel_mm : 0.0f, press, fault);

    if (job->active) {
        if (motion_set_speed(speed, speed) < E_OK) {
            imu_drive_cancel(job);
        }
    }
    if (!job->active && (was_active || press || *stop_pending)) {
        const bool was_pending = *stop_pending;
        bool stopped = motion_stop() >= E_OK;

        if (!stopped) {
            stopped = motion_estop() >= E_OK;
        }
        *stop_pending = !stopped;
        if (!stopped && !was_pending) {
            tm_printf((UB *)"[S4_DRIVE] STOP UNCONFIRMED; retrying; "
                      "remove VIN if wheels move\n");
        }
    }
    if (was_active != job->active || press) {
        tm_printf((UB *)"[S4_DRIVE] active %u reason %u",
                  (unsigned int)job->active, (unsigned int)job->reason);
        print_mm("travel_mm", job->distance_mm);
        tm_printf((UB *)"\n");
    }
}
#endif

static void diagnostics(const imu_snapshot_t *s)
{
    const imu_core_output_t *v = &s->value;

    tm_printf((UB *)"[S4_QUALITY] refresh_rejects %lu tilt_ddeg %ld "
              "peak_rejects %lu omega_mrad_s %ld alpha_mrad_s2 %ld\n",
              (unsigned long)v->refresh_rejects,
              (long)v->refresh_tilt_ddeg,
              (unsigned long)v->support_peak_rejects,
              (long)(v->yaw_rate_rad_s * 1000.0f),
              (long)(v->yaw_accel_rad_s2 * 1000.0f));
    tm_printf((UB *)"[S4_DIAG] cal_count %lu rejects %lu "
              "bias_mg %ld %ld %ld std_mg %ld %ld %ld "
              "reads %lu faults %lu nack %lu timeout %lu id %lu cfg %lu "
              "ctrl1 %02x ctrl4 %02x mag_valid %u "
              "read_us %lu read_max_us %lu read_total_us %lu "
              "work_us %lu work_max_us %lu work_total_us %lu "
              "core_us %lu core_max_us %lu core_total_us %lu "
              "gap_max_us %lu missed %lu wake_lost %lu "
              "samples %lu invalid %lu motion_misses %lu gaps %lu\n",
              (unsigned long)v->calibrations,
              (unsigned long)v->calibration_rejects,
              (long)v->bias_mg[0], (long)v->bias_mg[1],
              (long)v->bias_mg[2], (long)v->still_std_mg[0],
              (long)v->still_std_mg[1], (long)v->still_std_mg[2],
              (unsigned long)s->probe.reads,
              (unsigned long)s->probe.faults,
              (unsigned long)s->probe.nacks,
              (unsigned long)s->probe.timeouts,
              (unsigned long)s->probe.identity_errors,
              (unsigned long)s->probe.config_errors,
              (unsigned int)s->probe.ctrl1, (unsigned int)s->probe.ctrl4,
              (unsigned int)s->raw.mag_valid,
              (unsigned long)s->probe.last_us,
              (unsigned long)s->probe.max_us,
              (unsigned long)s->probe.total_us,
              (unsigned long)s->work_last_us,
              (unsigned long)s->work_max_us,
              (unsigned long)s->work_total_us,
              (unsigned long)s->compute_last_us,
              (unsigned long)s->compute_max_us,
              (unsigned long)s->compute_total_us,
              (unsigned long)s->gap_max_us,
              (unsigned long)s->missed_wakes,
              (unsigned long)s->wake_lost,
              (unsigned long)v->samples,
              (unsigned long)v->invalid_samples,
              (unsigned long)v->motion_misses,
              (unsigned long)v->gap_resets);
}

EXPORT void subsystem4_test(void)
{
    ID cyclic;
    imu_snapshot_t snapshot;
#if APP_IMU_STREAM
    imu_mark_t marker = {0};
    imu_mark_t drive_button = {0};
    imu_drive_t drive = {0};
    bool stop_pending = false;
    uint32_t printed_mark = 0u;
#endif
    T_CCYC config = {.cycatr = TA_HLNG, .cychdr = (FP)release,
                     .cyctim = 20u, .cycphs = 20u};

#if TK_SUPPORT_SMP
    config.cycatr |= TA_ASSPRC;
    config.assprc = TP_PRC1;
#endif
    if (motion_stop() < E_OK) {
        tm_printf((UB *)"[TEST] S4 FAIL: cannot request zero motor duty\n");
        return;
    }
#if APP_IMU_STREAM
    imu_hw_mark_init();
    imu_hw_drive_init();
    /* A held button across boot must not become a new drive command. */
    drive_button.sample = imu_hw_drive_down();
    drive_button.stable = drive_button.sample;
    drive_button.changed_at = drv_time_ms();
    tm_printf((UB *)"[TEST] VIN REQUIRED for drive. Lift wheels first. "
              "GP21: equal 100 mm/s, 600 mm /10 s limit; "
              "second press or ANY fault stops. GP20 MARK unchanged.\n");
    tm_printf((UB *)"[TEST] IMU_STREAM: 50 Hz until power-off; "
              "GP20 MARK resets height/peak/distance on next IMU tick. "
              "Other tests/destructive watchdog reset skipped.\n");
#endif
    tm_printf((UB *)"[TEST] S4 observe only; no gyro. +X right +Y forward "
              "+Z up. Boot LEVEL and still >=1 s; refresh tilt guard 1 deg. "
              "Zero duty is COAST (both bridge inputs low), not braking.\n");
    tm_printf((UB *)"[TEST] S4 GP0/1 I2C: remove UART adapter. "
              "Angles ddeg=0.1 degree; signed relative height in mm. "
              "No terrain detection, MQTT/KR1 fields or driving action.\n");
    runner = tk_get_tid();
    cyclic = tk_cre_cyc(&config);
    if (cyclic < E_OK || tk_sta_cyc(cyclic) < E_OK) {
        tm_printf((UB *)"[TEST] S4 FAIL: cannot start 20 ms observer cyclic\n");
        if (cyclic >= E_OK) {
            (void)tk_del_cyc(cyclic);
        }
        return;
    }
    for (unsigned int cycle = 0u;
#if APP_IMU_STREAM
         ;
#else
         cycle < 1500u;
#endif
         ++cycle) {
        if (tk_slp_tsk(100u) < E_OK) {
            tm_printf((UB *)"[TEST] S4 FAIL: observer wake timeout\n");
            break;
        }
        const INT extra = tk_can_wup(TSK_SELF);

#if APP_IMU_STREAM
        const bool drive_press = imu_mark_update(&drive_button,
            imu_hw_drive_down(), drv_time_ms());
        if (imu_mark_update(&marker, imu_hw_mark_down(), drv_time_ms())) {
            (void)imu_request_mark();
        }
#endif
        const bool observed = imu_snapshot_try_read(&snapshot) == E_OK;

#if APP_IMU_STREAM
        drive_update(&drive, &stop_pending, drive_press, observed,
                      &snapshot, extra > 0);
#else
        (void)extra;
#endif
        if (!observed) {
            tm_printf((UB *)"[S4] snapshot unavailable\n");
            continue;
        }
#if APP_IMU_STREAM
        if (snapshot.marks != printed_mark) {
            printed_mark = snapshot.marks;
            tm_printf((UB *)"[S4] MARK %lu\n", (unsigned long)printed_mark);
        }
        report(&snapshot);
#else
        if (cycle % 5u == 4u) {
            report(&snapshot);
        }
#endif
        if (cycle % 50u == 49u) {
            diagnostics(&snapshot);
            app_task_report();
        }
    }
#if APP_IMU_STREAM
    if (drive.active || stop_pending) {
        do {
            drive_update(&drive, &stop_pending, false, false, &snapshot,
                          true);
            if (stop_pending) {
                (void)tk_slp_tsk(100u);
                (void)tk_can_wup(TSK_SELF);
            }
        } while (stop_pending);
    }
#endif
    (void)tk_stp_cyc(cyclic);
    (void)tk_del_cyc(cyclic);
    (void)tk_can_wup(TSK_SELF);
    tm_printf((UB *)"[TEST] S4 operator judgement required; "
              "the observe-only IMU task continues\n");
}
#endif
