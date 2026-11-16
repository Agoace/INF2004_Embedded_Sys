# Split (dual-core) placement parked — 2026-11-15

| Item | Record |
| --- | --- |
| Decision | Operator: split is PARKED. The demo uses the UP image. Step 1 and the RAM generator work stop. |
| Reason 1 | UP system run PASS. Idle-meter load, UP core 0: 570-662 permille mean (30-40 % headroom), run peak 957 (P1). Split core 0 / core 1: 621-699 / 569-626 permille. |
| Reason 2 | Split misses the vehicle deadline on the robot (P1: first overrun at step 20, 9084 us). |
| Bench step 0 | Both images PASS: 2280000 edges, COUNT_MATCH 1140000/1140000 per wheel, no overflow, no reset. |
| UP vs split | ISR max 49 vs 193 us; response max 103 vs 482 us; line sample max 132 vs 193 us; motion gap 9983..10017 vs 9909..10064 us; BC_CPU 30.2 vs 49.7 s. |
| Cause | Lock-free generator ISR: 5.60 vs 7.43 us per call (+33 %), so shared XIP cache and bus contention. Kernel paths add SMP costs (BKL, cross-core reschedule, IPI). |
| Generator cost | 212 / 282 bp measured; plus IRQ entry/exit (1-2 us x 3800/s) the split figure is about 3.2-3.6 %, over the 3 % limit. |
| Code state | `mtk3-core-live` 52748fe: P1/P2 timing, bench step 0. Ordinary images byte-identical to 4d583c6 (76 hashes). |

## Evidence

| What | Where |
| --- | --- |
| Step-0 images | 52748fe; UP UF2 sha256 34095b67…c2223, split 73a5bee7…3b64 |
| Step-0 logs | `~/Projects/mtk3-core-review/build/bench_step0_52748fe/step0_{UP.final,SPLIT}.log` |
| Robot P1 logs | `build/school_20261113/` in the parent coursework checkout; load: `{ctrl,p1}_smp0_gp20_usb.txt`, `p1_split_gp20_usb.txt` |
| Step-0 rules | [BENCH_KERNEL_STEP0.md](BENCH_KERNEL_STEP0.md) |

## If resumed, try first

1. Discriminator: put the hot core-0 ISR path in RAM (`.data.ramfunc.*`, as for
   `rt_tl`). If split drops toward UP, XIP sharing is the main cost.
2. Generator: move it to RAM and cut its clock reads; re-run split step 0.
3. Then step 1: real ADC conversions with model values, motor-driven encoder,
   virtual track with barcode, GP21 calibrate + 90 s run x 10 per image.
