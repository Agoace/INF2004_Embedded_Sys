/* Exclusive BARCODE_CAPTURE operator settings; not normal robot settings. */
#ifndef BARCODE_SETTINGS_H
#define BARCODE_SETTINGS_H

#ifndef BARCODE_PASS_DISTANCE_MM
#define BARCODE_PASS_DISTANCE_MM 600u
#endif
#ifndef BARCODE_PASS_SPEED_MM_S
#define BARCODE_PASS_SPEED_MM_S 100u
#endif
/* Physical run-space limit. Never send a distance above 620 mm. */
#define BARCODE_BOUNDED_DISTANCE_MM                                            \
    (BARCODE_PASS_DISTANCE_MM > 620u ? 620u : BARCODE_PASS_DISTANCE_MM)
_Static_assert(BARCODE_PASS_DISTANCE_MM >= 50 &&
                   BARCODE_BOUNDED_DISTANCE_MM >= 50u,
               "Barcode pass distance must be at least 50 mm");
_Static_assert(BARCODE_PASS_SPEED_MM_S >= 25u &&
                   BARCODE_PASS_SPEED_MM_S <= 200u,
               "Barcode pass speed must be 25..200 mm/s");
#define BARCODE_COUNTS_PER_M 12414u
#endif
