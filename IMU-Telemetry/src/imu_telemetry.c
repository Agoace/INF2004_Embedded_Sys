/**
 * @file imu_telemetry.c
 * @brief Standalone pico_w firmware entry point for the IMU producer.
 *
 * The main loop owns the 13 ms cadence with absolute-time scheduling, drains
 * the pull queue, and prints one diagnostic line at 10 Hz. Nothing is printed
 * from an interrupt: this image uses no interrupt handler at all.
 *
 * Every printed value is scaled to an integer. Integer-only formatting keeps
 * the console free of float support requirements and keeps the 1 ms tick
 * cheap.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "imu_config.h"
#include "imu_core.h"
#include "imu_hal.h"

/** Milliseconds to wait for the USB console to enumerate before printing. */
#define IMU_BANNER_DELAY_MS 1500u

/** Tenths of a degree, or tenths of a millimetre, from a float. */
static long
tenths (float value)
{
    return (long) lroundf(value * 10.0f);
}

/** Milli-units, from a float in base units. */
static long
milli (float value)
{
    return (long) lroundf(value * 1000.0f);
}

static const char *
crosscheck_name (imu_crosscheck_t check)
{
    switch (check)
    {
        case IMU_CROSSCHECK_AGREE:      return "agree";
        case IMU_CROSSCHECK_DISAGREE:   return "disagree";
        case IMU_CROSSCHECK_UNKNOWN:    return "unknown";
        default:                        return "?";
    }
}

static void
print_events (void)
{
    imu_event_t ev;

    while (imu_event_pop(&ev))
    {
        printf("evt=%s seq=%u detail=%u value=%d\n",
               imu_event_name(ev.id), (unsigned) ev.seq, (unsigned) ev.detail,
               (int) ev.value);
    }
}

static void
print_diag (uint32_t now_ms)
{
    imu_diag_t diag;
    imu_snapshot_t snapshot;

    imu_diag_read(&diag);

    if (!imu_snapshot_read(&snapshot))
    {
        return;
    }

    /* Acceleration in milli-g, magnetism in milli-gauss, angles in tenths. */
    long const hump_tenths = tenths(diag.hump_mm);
    long const hump_abs    = (hump_tenths < 0) ? -hump_tenths : hump_tenths;

    printf("t=%lums acc=%ld,%ld,%ld|%ld,%ld,%ldmg "
           "mag=%ld,%ld,%ld|%ld,%ld,%ldmG "
           "pitch=%ldddg head=%ldddg turn=%lddps terr=%s "
           "h=%s%ld.%ldmm hi=%umm dd=%dmm enc=%d/%d x=%s "
           "seq=%u w0=%08" PRIx32 " w1=%08" PRIx32 " "
           "ev=%lu/%lu/%lu i2c=%lu inv=%lu st=%lu cal=%lu rec=%lu "
           "drdy=%lu snap=%lu dist=%lumm\n",
           (unsigned long) now_ms,
           milli(diag.accel_raw_g[0]), milli(diag.accel_raw_g[1]),
           milli(diag.accel_raw_g[2]),
           milli(diag.accel_filt_g[0]), milli(diag.accel_filt_g[1]),
           milli(diag.accel_filt_g[2]),
           milli(diag.mag_raw_gauss[0]), milli(diag.mag_raw_gauss[1]),
           milli(diag.mag_raw_gauss[2]),
           milli(diag.mag_filt_gauss[0]), milli(diag.mag_filt_gauss[1]),
           milli(diag.mag_filt_gauss[2]),
           tenths(diag.pitch_deg), tenths(diag.heading_deg),
           (long) lroundf(diag.turn_rate_dps),
           imu_terrain_name(diag.terrain),
           (hump_tenths < 0) ? "-" : "", hump_abs / 10, hump_abs % 10,
           (unsigned) diag.highest_hump_mm,
           (int) diag.distance_delta_mm,
           (int) diag.encoder_left_delta, (int) diag.encoder_right_delta,
           crosscheck_name(diag.turn_crosscheck),
           (unsigned) diag.sequence,
           (uint32_t) snapshot.word[0], (uint32_t) snapshot.word[1],
           (unsigned long) diag.events_posted, (unsigned long) diag.events_dropped,
           (unsigned long) diag.queue_high_water,
           (unsigned long) diag.i2c_errors, (unsigned long) diag.invalid_samples,
           (unsigned long) diag.self_test_failures,
           (unsigned long) diag.calibration_rejects,
           (unsigned long) diag.recoveries, (unsigned long) diag.missed_drdy,
           (unsigned long) diag.snapshot_failures,
           (unsigned long) diag.run_distance_mm);
}

int
main (void)
{
    stdio_init_all();

    (void) imu_hal_bus_init();

    sleep_ms(IMU_BANNER_DELAY_MS);

    printf("\nimu_telemetry: LSM303DLHC IMU producer, %u ms cadence\n",
           (unsigned) IMU_CFG_PERIOD_MS);
    printf("I2C0 GP%u/GP%u, DRDY GP%u, accel 0x%02X, mag 0x%02X\n",
           (unsigned) IMU_CFG_SDA_GPIO, (unsigned) IMU_CFG_SCL_GPIO,
           (unsigned) IMU_CFG_DRDY_GPIO, (unsigned) IMU_CFG_ACCEL_ADDR,
           (unsigned) IMU_CFG_MAG_ADDR);
    printf("Diagnostics only: this image is an IMU producer, not the Manager.\n");

    uint32_t now = imu_hal_now_ms();

    imu_init(now);

    uint32_t next_step = now;
    uint32_t last_diag = now;

    for (;;)
    {
        now = imu_hal_now_ms();

        if ((int32_t) (now - next_step) >= 0)
        {
            imu_step(now);

            next_step += IMU_CFG_PERIOD_MS;

            /* A long stall resynchronises instead of replaying missed steps. */
            if ((int32_t) (now - next_step) >= 0)
            {
                next_step = now + IMU_CFG_PERIOD_MS;
            }
        }

        if ((now - last_diag) >= IMU_CFG_DIAG_PERIOD_MS)
        {
            last_diag = now;
            print_events();
            print_diag(now);
        }

        sleep_ms(1);
    }
}
