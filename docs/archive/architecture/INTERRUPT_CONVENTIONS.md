# Interrupt and polling conventions

Use the method that matches the event being measured.

## Event-driven work

Prefer interrupts for event-driven work. The RP2040 has only two cores, and
GPIO edge detection can run independently of normal core work. An encoder edge
is an event, so the motor targets use GPIO interrupts to update the count and
to make the stop decision.

Keep an interrupt handler short. It may do a register write, take a timer
sample, and set a flag. It must not call `printf`, sleep, allocate memory, or
use floating point. Shared state is `volatile`. Read a multi-field shared
value with `save_and_disable_interrupts()` and
`restore_interrupts()`.

## Time-driven work

Use a timer for genuinely fixed-interval sampling. The important distinction
is event-driven work versus time-driven work, not interrupts versus polling.

The motor drift target keeps button input polled at 20 ms. This debounces
mechanical contact bounce without a separate button interrupt. It also keeps
the operator interface simple.

## Polling for no event

Poll when the required condition is the absence of an event. A stall is the
worked example: no encoder interrupt can fire when a jammed motor is not
turning. The main loop watches the count and time, then stops the motor after
the stall limit. The emergency button is checked in the same loop.

The motor drift target has no output printing in the timed run. Results are
printed after the run and replayed from RAM, so USB output does not change the
measurement.

## Higher event rates

At roughly tens of kilohertz, use PIO rather than either a normal GPIO
interrupt or a polling loop. These encoders produce about 1 kHz of edges at
10% duty, which is comfortably below that range.
