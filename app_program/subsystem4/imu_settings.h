/* Observe-only geometry/settings. Millimetres; no terrain thresholds.
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
/* Height does not enter the planar yaw correction. Pitch/roll angular
 * motion cannot be measured independently without a gyro. */
#endif
