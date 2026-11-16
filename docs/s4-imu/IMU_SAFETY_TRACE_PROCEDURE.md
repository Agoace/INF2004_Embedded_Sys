# IMU safety trace capture and replay

How to turn a robot session into evidence for the IMU safety bounds. Use with
`IMU_SAFETY_HUMP_PLAN.md` and `IMU_SAFETY_HUMP_CHECKLIST.md`, which own the
acceptance gates. This record covers the tooling only.

Those two records are not in this branch yet: they are untracked files in the
`mtk3-staging` worktree. They must be committed with this branch before
handover, which is why they are named here rather than linked.

Status: the tooling is verified on synthetic fixtures and on the host. No
measured IMU trace exists in the tree, so no threshold is confirmed by this
procedure yet.

## Why a capture is needed

The detector's lift bounds are 0.4 g free fall and 0.5 g vertical change, and
the overturn bound is 75 degrees. The plan treats all three as candidates
until recorded motion separates a real lift from a turn, a brake or a hump.
A host test cannot supply that separation, and a successful build says
nothing about it.

## Path

```text
robot (TEST or IMU_STREAM)      laptop
  [S4] observer line    --->   imu_trace_to_csv.py    --->   CSV
                                                              |
                              replay_imu_safety  <-----------+
                                       |
                                       v
                          trigger onset, latch, g range
```

The observer needs no firmware change: every `[S4]` line already prints
`cal_mg`, which is the mapped, bias-corrected acceleration the detector
consumes. The converter only selects columns, so it cannot re-derive a
mapping or a bias.

## Capture

Use a TEST image. `IMU_STREAM=1` gives 50 Hz instead of 10 Hz, which is
closer to the 100 Hz detector and better for a short lift.

```sh
cd build_make
make SMP=0 WIFI=none TEST=1 IMU_STREAM=1 -j8
```

This produces `build_make/mtk3pico_smp0_usb_cdc_test_imu_stream.uf2`.

**A TEST image captures motion but cannot validate the stop.** TEST images
do not run the Vehicle task at all, so no safety gate, latch or restart
exists in them. Use TEST only to record the acceleration trace. Check the
stop, the latch, the start gate and the reported reason on a normal robot
image:

```sh
make SMP=0 WIFI=none -j8          # normal robot image, no network
```

Both were checked by building them and searching the linked image: the
`[S4]` and `cal_mg` strings are in the TEST image, and the safety report
line is in the normal image.

Flash, then capture the USB console. Assert DTR **and** RTS, or the Pico
stays silent:

```powershell
$p = New-Object System.IO.Ports.SerialPort('COM6', 115200, 'None', 8, 'One')
$p.DtrEnable = $true; $p.RtsEnable = $true; $p.Open()
# Example: 200 samples at 50 Hz is 4 s.
for ($i = 0; $i -lt 200; $i++) { $p.ReadLine(); Start-Sleep -Milliseconds 20 }
$p.Close()
```

Redirect that to a file, or use any terminal that can save its output. Lift
the robot by hand from rest and, separately, while the wheels turn. Repeat
several times so one bad capture is not the only evidence.

**Safety:** lift both wheels before a TEST image runs. S2-U02 sends a real
40/40 mm/s command at power-up.

## Convert and replay

```sh
python3 scripts/imu_trace_to_csv.py capture.txt trace.csv
build/host/test_replay_imu_safety trace.csv
```

The analyser prints one line per confirmed trigger and a summary:

```text
trigger t_ms=... reason=lift g_mg=... tilt_ddeg=... vertical_mg=...
summary rows=... events=... first_reason=... latched=... stops=... min_g_mg=... max_g_mg=... max_tilt_ddeg=... clear_ms=...
```

## Reading the stop reason on the robot

A latched reason prints one bounded USB line, once per latch:

```text
[robot] IMU safety <reason> g_mg <n> tilt_ddeg <n> vertical_mg <n> clear_ms <n> age_ms <n>
```

`<reason>` is `IMU lift detected`, `IMU overturn detected` or
`IMU unavailable`. The line is printed from the Vehicle idle loop, not from
the stop path, so the stop itself stays bounded and free of console writes.
An MQTT build additionally carries the same reason as stop reason 14, 15 or
16 in `robot/summary`; a non-MQTT build reports it only on this line.

## Limits to state with any result

- **`wheels_still` is not on the `[S4]` line**, so the converter emits 0.
  That field feeds only the all-clear window, never a trigger decision, so a
  zero here cannot invent or hide a lift or an overturn. It does mean a
  replay cannot report the restart window.
- **The observer is slower than the detector.** 10 Hz normally, 50 Hz with
  `IMU_STREAM`, against a 100 Hz detector. A replay shows whether the bounds
  would fire on the recorded motion. It does not reproduce detector timing,
  the two- and three-sample windows, or stop latency.
- **A capture only contains a transient the observer caught.** A missed lift
  in the log is not proof that the detector would miss it, and the reverse.
- **Synthetic fixtures prove the tool, not the robot.**
  `tests/fixtures/imu_safety_synthetic_lift.csv` and
  `imu_trace_log_synthetic.txt` are labelled synthetic. Do not cite them as
  lift evidence.

## What the evidence would change

If a real capture shows a lift that does not reach 0.4 g or 0.5 g, the
bounds are too high for that gesture. Do not lower them to make a gentle
lift pass: the plan prioritises few false stops, and a lower bound trades a
missed detection for a spurious stop. Record the separation against turn,
brake and hump traces first, then change the bounds and the host fixtures
together.
