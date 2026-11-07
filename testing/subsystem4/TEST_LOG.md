# Subsystem 4 test log

## GY-511 steps 0 and 1

Use the [operator tilt procedure](../../docs/IMU_BRINGUP.md#operator-tilt-check-not-run).
Source credit: Jaison, `origin/jaison-imu` `b2a6bb6`.

| ID | Check | Result |
| --- | --- | --- |
| S4-U01 | TEST, wheels lifted: 30 s of 10 ms polling, USB each 100 ms; confirm axes, gravity, tilt, bus/read timing and absent-device behavior | Partial hardware evidence on 02f7c37: two windows of 2999 reads, bus/config faults zero; read 529–542 us, max 624 us, gap max 18975 us, missed zero. Mag changes with tilt; side roll captured. Sign portion PASS via the later 6ed89ad stream: nose up +Y, right side up +X, top up +Z confirms identity map. Static magnitude 1371–1391 mg is explained by positive Z offset, not uniform gain. Absent-device check remains Not run; not a full S4-U01 PASS. See [investigation](../../docs/IMU_SCALE_INVESTIGATION.md). |
| S4-U03 | TEST IMU_STREAM=1: only S4 streams until power-off, USB line every 100 ms, no S2 motor command/destructive watchdog test; GP20 MARK once per stable press; record six faces, nose up/down, both sides, saved 57/88 setup register values, all counters and new read_us | PASS on 6ed89ad, USB only: 16129 reads, fault/NACK/timeout/identity/config zero, ctrl1 57 ctrl4 88; read about 530–566 us, max 592 us, gap max 17946 us, missed zero. Six debounced MARKs, no repeats. Six-face means and offset conclusion are in the investigation below. |
| S4-U02 | Cold power boot with IMU on GP0/GP1 and USB only: successful USB console and sensor configuration, no UART mux; repeat on MQTT-TEST | Not run |

Actual read wall time on 02f7c37: **529–542 us, maximum 624 us** per nominal
10 ms cycle. On 6ed89ad continuous stream: **about 530–566 us, maximum
592 us**. Mag is still read each cycle; the two-transfer workload is unchanged.

Six-face means (samples +3..+25 after each MARK, hand-held, mg):

| MARK | Pose | X | Y | Z | Magnitude |
| --- | --- | ---: | ---: | ---: | ---: |
| M1 | Top up | -0.1 | 82 | 1376 | 1378 |
| M2 | Upside down | -128 | 124 | -776 | 797 |
| M3 | Nose up | 37 | 1116 | 459 | |
| M4 | Nose down | -44 | -943 | 322 | |
| M5 | Right side up | 1012 | 107 | 374 | |
| M6 | Left side up | -1022 | 115 | 315 | |

Z offset is about +300 to +350 mg (opposed-face half-sum +300); X/Y/Z
half-differences are about 1017/1030/1076 mg/g, not jig-exact gain estimates.
Y half-sum about +87 mg matches level Y=82 mg; a 180-degree yaw flip is
needed before attributing raw level pitch to mounting. See
[scale investigation](../../docs/IMU_SCALE_INVESTIGATION.md) for the per-axis
bias-removal requirement and the corrected 0.90–1.10 g gate ordering.
The TEST output prints data and counters, not an automatic hardware PASS.
No stationary calibration, filtering, hump threshold or robot integration
has been tested or implemented in this step.
