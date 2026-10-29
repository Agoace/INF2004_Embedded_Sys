/**
 * @file imu_snapshot.c
 * @brief Two-word snapshot packing. No packed structs, no compiler bit-fields.
 */
#include "imu_snapshot.h"

/* Word 1 field shifts. */
#define HEADING_SHIFT     0u
#define HEADING_MASK      0x00000FFFu
#define PITCH_SHIFT       12u
#define TURN_SHIFT        22u
#define SIGN10_MASK       0x3FFu
#define SIGN10_SIGN       0x200u
#define SIGN10_FULL       0x400

/* Word 2 field shifts. */
#define HUMP_SHIFT        0u
#define HUMP_MASK         0x0000FFFFu
#define TERRAIN_SHIFT     16u
#define TERRAIN_MASK      0x7u
#define ACCEL_VALID_BIT   19u
#define MAG_VALID_BIT     20u
#define CALIBRATED_BIT    21u
#define SELFTEST_BIT      22u
#define FAULT_BIT         23u
#define SEQ_SHIFT         24u
#define SEQ_MASK          0xFFu

/** Clamp to the 10-bit signed field range and return its two's complement. */
static uint32_t
encode_signed10 (int32_t value)
{
    if (value < -SIGN10_FULL / 2)
    {
        value = -SIGN10_FULL / 2;               /* -512 */
    }

    if (value > (SIGN10_FULL / 2) - 1)
    {
        value = (SIGN10_FULL / 2) - 1;          /* 511 */
    }

    return ((uint32_t) value) & SIGN10_MASK;
}

/** Sign-extend a 10-bit two's complement field. */
static int32_t
decode_signed10 (uint32_t field)
{
    int32_t const raw = (int32_t) (field & SIGN10_MASK);

    return ((0u != (raw & SIGN10_SIGN)) ? (raw - SIGN10_FULL) : raw);
}

void
imu_snapshot_pack (const imu_snapshot_fields_t * fields, imu_snapshot_t * out)
{
    uint32_t w1 = 0u;
    uint32_t w2 = 0u;

    w1 |= ((uint32_t) fields->heading_ddeg) & HEADING_MASK;
    w1 |= encode_signed10(fields->pitch_ddeg) << PITCH_SHIFT;
    w1 |= encode_signed10(fields->turn_rate_dps) << TURN_SHIFT;

    w2 |= ((uint32_t) fields->highest_hump_mm) & HUMP_MASK;
    w2 |= (((uint32_t) fields->terrain) & TERRAIN_MASK) << TERRAIN_SHIFT;

    if (fields->accel_valid)
    {
        w2 |= 1u << ACCEL_VALID_BIT;
    }

    if (fields->mag_valid)
    {
        w2 |= 1u << MAG_VALID_BIT;
    }

    if (fields->calibrated)
    {
        w2 |= 1u << CALIBRATED_BIT;
    }

    if (fields->selftest_passed)
    {
        w2 |= 1u << SELFTEST_BIT;
    }

    if (fields->fault_latched)
    {
        w2 |= 1u << FAULT_BIT;
    }

    w2 |= (((uint32_t) fields->sequence) & SEQ_MASK) << SEQ_SHIFT;

    out->word[0] = w1;
    out->word[1] = w2;
}

void
imu_snapshot_unpack (const imu_snapshot_t * snap, imu_snapshot_fields_t * fields)
{
    uint32_t const w1 = snap->word[0];
    uint32_t const w2 = snap->word[1];

    fields->heading_ddeg   = (uint16_t) ((w1 >> HEADING_SHIFT) & HEADING_MASK);
    fields->pitch_ddeg     = (int16_t) decode_signed10(w1 >> PITCH_SHIFT);
    fields->turn_rate_dps  = (int16_t) decode_signed10(w1 >> TURN_SHIFT);

    fields->highest_hump_mm = (uint16_t) ((w2 >> HUMP_SHIFT) & HUMP_MASK);
    fields->terrain = (imu_terrain_t)
        ((w2 >> TERRAIN_SHIFT) & TERRAIN_MASK);

    fields->accel_valid     = (0u != (w2 & (1u << ACCEL_VALID_BIT)));
    fields->mag_valid       = (0u != (w2 & (1u << MAG_VALID_BIT)));
    fields->calibrated      = (0u != (w2 & (1u << CALIBRATED_BIT)));
    fields->selftest_passed = (0u != (w2 & (1u << SELFTEST_BIT)));
    fields->fault_latched   = (0u != (w2 & (1u << FAULT_BIT)));
    fields->sequence        = (uint8_t) ((w2 >> SEQ_SHIFT) & SEQ_MASK);
}
