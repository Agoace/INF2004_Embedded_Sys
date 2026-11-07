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

The driver reads the configured registers back and refuses a mismatch. The
observed zero config faults implies that its setup readbacks matched. The
new stream also prints `ctrl1`/`ctrl4`, the saved setup readbacks, so the
operator can confirm 57/88 in the next run without extra per-cycle transfers.
The host tests check positive/negative counts, ignored low bits and limits;
there is no floating-point scale factor in the accel conversion.

**No decode/configuration bug was found in this audit.** A 1.375 magnitude
factor is not a documented normal +/-2 g HR scale, and is not the factor of
two/four expected from a bit shift or full-scale mismatch. Do not label it a
confirmed part property either: these limited orientations cannot separate
axis bias, per-axis gain, mounting, supply/part issues or an unexpected device
variant. The side angles are plausible, but ratios can look correct even
with a common gain error. WHO_AM_I alone is not a full physical part check.

Do not multiply the samples by 1/1.375 or learn a bias from this single pose.
Jaison's unmodified 0.90–1.10 g stationary-calibration gate would reject these
samples indefinitely. That gate is not implemented here; it must not be
ported unchanged before the cause is understood.

## Next measurements (Not run)

Use the continuous stream in [IMU bring-up](IMU_BRINGUP.md#continuous-operator-stream).
Keep the wheels lifted. Mark and hold each static position for several seconds:
level, nose up/down, right side up/down, then each of +X, -X, +Y, -Y, +Z, -Z
pointing upward. Press GP20 once after a pose settles to insert `MARK n`.
Record saved register values, all three uncorrected axes, gravity and read
counters. Photograph the module markings/orientation and check the supply and
board pull-ups. Keep motors off; no driving test is needed.

For each axis, compare the stable +1 g and -1 g measurements. Their half-sum
estimates bias; their half-difference estimates counts per g. This distinguishes
an offset from a gain issue. These are diagnostics, not approved calibration
values. Review the complete data before changing scales, gates or thresholds.
