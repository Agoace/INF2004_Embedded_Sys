# Subsystem 4 test log

## GY-511 steps 0 and 1

Use the [operator tilt procedure](../../docs/IMU_BRINGUP.md#operator-tilt-check-not-run).
Source credit: Jaison, `origin/jaison-imu` `b2a6bb6`.

| ID | Check | Result |
| --- | --- | --- |
| S4-U01 | TEST, wheels lifted: 30 s of 10 ms polling, USB each 100 ms; confirm accel +X right/+Y forward/+Z up, approximately 1 g when stationary, pitch nose-up positive, roll right-up positive; save accel mg, magnetic milli-gauss, gravity, tilt, per-read us and max, cycle gaps/missed wakes, fault/NACK/timeout/identity/config counters; power off and repeat with absent sensor, USB and motion must stay responsive | Not run |
| S4-U02 | Cold power boot with IMU on GP0/GP1 and USB only: successful USB console and sensor configuration, no UART mux; repeat on MQTT-TEST | Not run |

Actual I2C read time per 10 ms cycle: **Not measured**. Record the range and
maximum from S4-U01, not the 405 us ideal-wire estimate in the design note.
The TEST output prints data and counters, not an automatic hardware PASS.
No stationary calibration, filtering, hump threshold or robot integration
has been tested or implemented in this step.
