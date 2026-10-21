/**
 * @file line_motion_bridge.c
 * @brief See line_motion_bridge.h. Owned by the Line & Barcode Task context:
 *        call these functions from that task only.
 */
#include "line_motion_bridge.h"

#include "encoder.h"
#include "motion_control.h"
#include "motion_units.h"

static int8_t      g_table[5] = { -110, -50, 0, 50, 110 };
static int8_t      g_last_steer;
static odometer_t  g_odo_left;
static odometer_t  g_odo_right;

line_status_t
line_bridge_classify (uint8_t black_pattern, uint8_t unknown_mask,
                      uint8_t fault_mask, int8_t * p_error)
{
    *p_error = 0;

    if (0u != fault_mask)
    {
        return LINE_STATUS_FAULT;
    }

    if (0u != unknown_mask)
    {
        return LINE_STATUS_UNKNOWN;
    }

    switch (black_pattern & 0x7u)
    {
        case 1u: *p_error = -2; return LINE_STATUS_TRACKING;   /* L     */
        case 3u: *p_error = -1; return LINE_STATUS_TRACKING;   /* L + C */
        case 2u: *p_error = 0;  return LINE_STATUS_TRACKING;   /* C     */
        case 6u: *p_error = 1;  return LINE_STATUS_TRACKING;   /* C + R */
        case 4u: *p_error = 2;  return LINE_STATUS_TRACKING;   /* R     */
        case 0u: return LINE_STATUS_LOST;
        case 7u: return LINE_STATUS_JUNCTION;
        default: return LINE_STATUS_SPLIT;                     /* 5     */
    }
}

int8_t
line_bridge_steer_from_error (int8_t error)
{
    if (error < -2)
    {
        error = -2;
    }
    else if (error > 2)
    {
        error = 2;
    }

    return g_table[error + 2];
}

void
line_bridge_set_steer_table (const int8_t table[5])
{
    for (uint32_t i = 0u; i < 5u; i++)
    {
        g_table[i] = table[i];
    }
}

line_status_t
line_bridge_update (uint8_t black_pattern, uint8_t unknown_mask,
                    uint8_t fault_mask)
{
    int8_t error;
    line_status_t const status =
        line_bridge_classify(black_pattern, unknown_mask, fault_mask, &error);

    switch (status)
    {
        case LINE_STATUS_TRACKING:
        {
            g_last_steer = line_bridge_steer_from_error(error);
            motion_set_steer(g_last_steer);
            break;
        }

        case LINE_STATUS_LOST:
        {
            motion_set_steer(g_last_steer);
            break;
        }

        case LINE_STATUS_FAULT:
        {
            /* Do not refresh: Motion holds after the stale timeout. */
            break;
        }

        default:
        {
            motion_set_steer(0);
            break;
        }
    }

    return status;
}

void
line_bridge_mark (void)
{
    odometer_mark(&g_odo_left,  encoder_count(MOTOR_LEFT));
    odometer_mark(&g_odo_right, encoder_count(MOTOR_RIGHT));
}

int32_t
line_bridge_travel_mm (void)
{
    int32_t const l = odometer_update(&g_odo_left,  encoder_count(MOTOR_LEFT));
    int32_t const r = odometer_update(&g_odo_right, encoder_count(MOTOR_RIGHT));

    return motion_counts_to_mm((l + r) / 2);
}

int32_t
line_bridge_speed_mm_s (void)
{
    uint32_t w1;
    uint32_t w2;

    motion_get_snapshot(&w1, &w2);

    int32_t const l = msnap_unpack_speed(w1, MSNAP_W1_L_SHIFT);
    int32_t const r = msnap_unpack_speed(w1, MSNAP_W1_R_SHIFT);

    return motion_cpt_to_mm_s((l + r) / 2);
}
