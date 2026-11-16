# NVIC priority finding (F-NVIC)

Status: finding recorded 2026-11-13 on 451e562. Not fixed. The code fix
is a template change and needs its own approval.

## Finding

`EnableInt(intno, level)` does not set the interrupt priority on RP2040.

- `EnableInt_nvic()` in
  [`lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c`](../../lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c)
  writes `NVIC_IPR(intno)` with a byte store. This is template code,
  unchanged since `c3b0954`.
- On ARMv6-M (Cortex-M0+) the NVIC_IPR registers are word-access only. The
  Pico SDK says the same in `hardware_irq/irq.c` ("only 32 bit writes are
  supported"). The byte store has no effect.
- The kernel helper `knl_set_irq_priority()`
  (`kernel/sysdepend/cpu/rp2040/smp_rp2040.c`) uses a word
  read-modify-write and works.

This applies to UP and SMP images alike, and to every earlier robot image.

## Measured (robot, core 0, SWD read with no halt)

| Register | Value | Meaning |
| --- | --- | --- |
| IPR0 0xE000E400 | 0x00000000 | IRQ0 line alarm = 0 (requested 2) |
| IPR1 0xE000E404 | 0x00000000 | IRQ5 USB = 0 |
| IPR3 0xE000E40C | 0xC0000000 | IRQ13 GPIO = 0 (requested 1); IRQ15 SIO IPI = 3 |
| SHPR3 0xE000ED20 | 0xC0C00000 | SysTick = 3, PendSV = 3 |

Raw data: MAIN `build/school_20261113/irq_*.json` (not in Git).

## Effect

- The line alarm, USB and GPIO interrupts have the same priority, so none
  of them preempts another. When more than one is pending, the lowest IRQ
  number runs first: IRQ0 (line alarm), then IRQ5 (USB), then IRQ13
  (GPIO).
- All three still preempt SysTick, PendSV and the SIO IPI (priority 3).
- The requested order in the source (USB 0, GPIO 1, line alarm 2) was never
  in effect. All hardware results so far were measured with all external
  interrupts at priority 0.

## Fix (on hold)

Use a word read-modify-write in `EnableInt_nvic()`, under the same
interrupt mask, as `knl_set_irq_priority()` does. This is a template change:
it needs approval, a row in
[TEMPLATE_CHANGES.md](../architecture/TEMPLATE_CHANGES.md), a host test of
the mask and shift, a hardware readback of IPR and a timing re-run. With the
fix, USB and GPIO preempt the line alarm, so the line ISR worst case can
grow.
