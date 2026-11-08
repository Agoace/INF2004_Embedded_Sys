# GY-511 acceleration scale investigation

## Operator evidence: 02f7c37

Two USB-only TEST windows each produced 2999 complete reads. Fault, NACK,
timeout, identity and configuration counters were zero. Read wall time was
529–542 us, maximum 624 us per nominal 10 ms cycle; maximum cycle gap was
18975 us and no queued wake was missed. Magnetic data was valid and changed
with tilt (reported X range 249–328 milli-gauss).

At rest the acceleration magnitude was 1371–1391 mg, with Z about 1375 mg,
Y about 85 mg and X near zero. Side tilts gave roll about +170/-195 tenths
of a degree and X about +400/-440 mg. The operator missed the short windows;
nose-up/down positions were not captured. This is partial evidence, not an
axis calibration or a complete S4-U01 pass. The private raw log stays private.

## Six-face stream evidence: 6ed89ad

S4-U03 **PASS**, USB-only continuous stream: 16129 successful reads; fault,
NACK, timeout, identity and configuration counters all zero. Saved setup
readbacks were `ctrl1=57`, `ctrl4=88`. Read wall time was about 530–566 us,
maximum 592 us per nominal 10 ms cycle. Maximum cycle gap was 17946 us,
missed wakes zero. Six GP20 markers were debounced with no repeats.

Means below use samples +3 through +25 after each MARK. Positions were
hand-held, not jig-exact. Values are uncorrected mg, rounded as reported by
the operator; a blank magnitude means it was not supplied.

| MARK | Position | X mg | Y mg | Z mg | Magnitude mg |
| --- | --- | ---: | ---: | ---: | ---: |
| M1 | Top up | -0.1 | 82 | 1376 | 1378 |
| M2 | Upside down | -128 | 124 | -776 | 797 |
| M3 | Nose up | 37 | 1116 | 459 | |
| M4 | Nose down | -44 | -943 | 322 | |
| M5 | Right side up | 1012 | 107 | 374 | |
| M6 | Left side up | -1022 | 115 | 315 | |

**The 1.375 g level reading is explained by a large positive Z zero-g
offset, not a uniform gain error.** Z is +315 to +459 mg when nominally
horizontal. Its opposed-face half-sum is `(1376 - 776) / 2 = +300 mg`,
consistent with an offset about +300 to +350 mg. The Z half-difference is
1076 mg/g. X and Y half-differences are about 1017 and 1030 mg/g. These
hand-held data are diagnostic estimates, not approved gain corrections.

The Y opposed-face half-sum is `(1116 - 943) / 2 = +86.5 mg`, approximately
+87 mg, matching level Y=82 mg. Thus a roughly 3.5-degree raw level pitch
must not be called mounting pitch: Y offset may explain it. A 180-degree
yaw flip on the same level support is needed to separate offset from
mounting/support tilt.

The identity axis map is **confirmed**: nose up gives +Y, right side up
+X, top up +Z. The S4-U01 sign portion therefore passes through this stream
run. This does not complete its absent-device check or cold-power/MQTT-TEST
acceptance.

## Datasheet and code audit

Reference: ST LSM303DLHC, DocID018771 Rev 2, Table 3 (page 10), Tables 26/27
(page 27), and output registers (pages 30–31).
[ST datasheet](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf),
[Pololu mirror](https://www.pololu.com/file/0J564/LSM303DLHC.pdf).

| Item | Required setting | Driver/decoder |
| --- | --- | --- |
| CTRL_REG1_A | 100 Hz, low power off, XYZ enabled | `0x57` |
| CTRL_REG4_A | BDU=1, BLE=0, FS=00 (+/-2 g), HR=1 | `0x88` |
| Register byte order | Lower address is low byte | Little-endian assembly |
| HR signed sample | 12-bit left aligned | Unsigned word divided by 16, then subtract 4096 if its sign bit is set |
| Sensitivity | Table 3: 1 mg/LSB for FS=00 | One decoded count is one mg |

The driver reads the configured registers back and refuses a mismatch.
The stream confirmed saved setup readbacks `ctrl1=57` and `ctrl4=88`, with
zero config faults and no extra per-cycle register transactions.
The host tests check positive/negative counts, ignored low bits and limits;
there is no floating-point scale factor in the accel conversion.

**No decode/configuration bug was found in this audit.** The later six-face
measurements identify the Z offset as the cause of the high level magnitude;
no 1/1.375 multiplier or full-scale change is justified.

## Calibration consequence (not implemented)

Calibration must subtract a **per-axis bias**. Jaison's stationary-level
method (mean measured vector minus expected `(0,0,1000)` mg) works for
pitch-from-level: it removes the Z offset and Y offset together. It assumes
the robot is stationary on a level surface and defines that pose as level;
it does not separately measure mounting angle, support tilt or axis gains.

Jaison's 0.90–1.10 g magnitude gate must be applied **after bias removal**,
or dropped. Applied to these raw readings it would reject calibration
indefinitely. A gate after correction alone is not proof of stationarity:
separate motion/noise checks still need a later design. No calibration code,
bias constants, scales or thresholds change in this documentation update.

## Remaining measurements (Not run)

Use the continuous stream in
[IMU bring-up](IMU_BRINGUP.md#continuous-operator-stream), wheels lifted.
Repeat level measurements before/after a 180-degree yaw flip on the same
support to distinguish Y offset from mounting/support tilt. Better-supported
opposed faces can refine the diagnostic bias/gain estimates, but do not
install calibration constants from these hand-held means. Absent-device,
cold-power and MQTT-TEST hardware checks remain separate pending gates.
