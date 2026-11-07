# Subsystem 4 test log

## GY-511 steps 0 and 1

Use the [operator tilt procedure](../../docs/IMU_BRINGUP.md#operator-tilt-check-not-run).
Source credit: Jaison, `origin/jaison-imu` `b2a6bb6`.

| ID | Check | Result |
| --- | --- | --- |
| S4-U01 | TEST, wheels lifted: 30 s of 10 ms polling, USB each 100 ms; confirm axes, gravity, tilt, bus/read timing and absent-device behavior | Partial hardware evidence on 02f7c37: two windows of 2999 reads, bus/config faults zero; read 529–542 us, max 624 us, gap max 18975 us, missed zero. Mag changes with tilt; side roll captured. Nose up/down and absent device not captured. Static magnitude 1371–1391 mg is unresolved; not a full PASS. See [investigation](../../docs/IMU_SCALE_INVESTIGATION.md). |
| S4-U03 | TEST IMU_STREAM=1: only S4 streams until power-off, USB line every 100 ms, no S2 motor command/destructive watchdog test; GP20 MARK once per stable press; record six faces, nose up/down, both sides, saved 57/88 setup register values, all counters and new read_us | Not run |
| S4-U02 | Cold power boot with IMU on GP0/GP1 and USB only: successful USB console and sensor configuration, no UART mux; repeat on MQTT-TEST | Not run |

Actual read wall time on 02f7c37: **529–542 us, maximum 624 us** per nominal
10 ms cycle. The continuous stream build is not yet measured. Mag is still
read each cycle, so the same two-transfer workload is retained.
The TEST output prints data and counters, not an automatic hardware PASS.
No stationary calibration, filtering, hump threshold or robot integration
has been tested or implemented in this step.
