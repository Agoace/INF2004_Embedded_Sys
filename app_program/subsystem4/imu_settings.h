/* Observe-only geometry/settings. Millimetres; hump event thresholds
 * report only and never command driving.
 * Track is the calibrated effective track, not the rough 115 mm measure. */
#ifndef IMU_SETTINGS_H
#define IMU_SETTINGS_H
#ifndef IMU_TRACK_MM
#define IMU_TRACK_MM 111.0f
#endif
#ifndef IMU_WHEELBASE_MM
#define IMU_WHEELBASE_MM 80.0f
#endif
#ifndef IMU_FORWARD_MM
#define IMU_FORWARD_MM 60.0f
#endif
#ifndef IMU_HEIGHT_MM
#define IMU_HEIGHT_MM 130.0f
#endif
#ifndef IMU_REFRESH_TILT_DEG
#define IMU_REFRESH_TILT_DEG 1.0f
#endif
#ifndef IMU_SUPPORT_TOLERANCE_MM
#define IMU_SUPPORT_TOLERANCE_MM 2.0f
#endif
#define IMU_SUPPORT_WINDOW_SAMPLES 10u
/* Wide diagnostic tolerance, never a moving-height/peak gate. */
#ifndef IMU_ROLL_PITCH_TOLERANCE_DEG
#define IMU_ROLL_PITCH_TOLERANCE_DEG 10.0f
#endif
/* Step 3 hump events (imu_hump.h). Thresholds come from the step 2d floor
 * run: flat 200 ms median max 0.6 mm, single max 1.2 mm, sd <= 0.68 mm.
 * Distances are encoder travel, so the rules do not depend on speed. */
#define IMU_HUMP_SOURCE_CORRECTED 1
#define IMU_HUMP_SOURCE_FILTERED 2
#ifndef IMU_HUMP_SOURCE
#define IMU_HUMP_SOURCE IMU_HUMP_SOURCE_CORRECTED
#endif
#ifndef IMU_HUMP_ON_MM
#define IMU_HUMP_ON_MM 3.0f
#endif
#ifndef IMU_HUMP_OFF_MM
#define IMU_HUMP_OFF_MM 1.5f
#endif
#ifndef IMU_HUMP_ON_TRAVEL_MM
#define IMU_HUMP_ON_TRAVEL_MM 30.0f
#endif
#ifndef IMU_HUMP_OFF_TRAVEL_MM
#define IMU_HUMP_OFF_TRAVEL_MM 30.0f
#endif
#ifndef IMU_HUMP_MAX_TRAVEL_MM
#define IMU_HUMP_MAX_TRAVEL_MM 600.0f
#endif
#ifndef IMU_HUMP_GATE_LATERAL_MG
#define IMU_HUMP_GATE_LATERAL_MG 40.0f
#endif
#ifndef IMU_HUMP_GATE_YAW_RAD_S
#define IMU_HUMP_GATE_YAW_RAD_S 1.5f
#endif
#ifndef IMU_HUMP_MIN_SPEED_MM_S
#define IMU_HUMP_MIN_SPEED_MM_S 30.0f
#endif
/* A board copy older than this is never used for a value. */
#ifndef IMU_HUMP_MAX_AGE_MS
#define IMU_HUMP_MAX_AGE_MS 50u
#endif
/* Height does not enter the planar yaw correction. Pitch/roll angular
 * motion cannot be measured independently without a gyro. */
#endif
