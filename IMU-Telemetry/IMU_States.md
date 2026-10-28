# INIT / CALIBRATING
Initialize the accelerometer and magnetometer on I2C0 GP0/GP1 and use DRDY on GP6.
Configure sampling and filters, perform the sensor self-test and calibrate IMU bias while stationary.
Post the IMU self-test and calibration results to mbf_ctrl_evt.
# HEALTHY PERIODIC ACTIVITIES
Every 13 ms in every healthy state, read the IMU, apply calibration/filtering, calculate wrap-safe heading change and turn rate, cross-check turn rate with the Motor 1/Motor 2 encoder count difference, monitor sensor health, and write pitch, heading, turn rate and highest hump peak to robot_snapshot_t under mtx_snapshot.
# IDLE / LEVEL
Continue 13 ms sampling at level pitch.
Set the current hump height to zero; clear the highest hump peak only when RUN becomes set for a new run.
On a confirmed climb, post HUMP_START and enter CLIMBING.
# CLIMBING
Accumulate hump height as h += Δd × sin(pitch), using encoder distance increments.
Track the current peak.
Enter CREST when the climb flattens.
If level is confirmed before a valid hump develops, discard the false-start peak, post HUMP_END and return to IDLE / LEVEL.
CREST
Record the hump peak and retain the highest peak for the run.
Enter DESCENDING when descending pitch is detected.
If level is confirmed without a descent phase, post HUMP_END and return to IDLE / LEVEL.
# DESCENDING
Continue h += Δd × sin(pitch).
On confirmed level, post HUMP_END with the measured peak and retain the highest hump peak for the run.
# COLLISION
Collision detection is active in every healthy terrain state. Debounce/latch the detection and post COLLISION once per detected episode rather than every 13 ms sample.
# SENSOR_FAULT
Stop using invalid sensor data and post SENSOR_FAULT to mbf_ctrl_evt.
Retry I2C checks; when communication recovers, return to INIT / CALIBRATING for self-test and calibration. Global FAULT clearance remains owned by the Vehicle Controller.
The GY-511 has no gyroscope; turn rate is derived from wrap-safe magnetometer heading change per IMU period and cross-checked with encoder difference.
