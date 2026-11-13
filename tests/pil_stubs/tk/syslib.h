#ifndef PIL_FAKE_SYSLIB_H
#define PIL_FAKE_SYSLIB_H
/* Single-threaded test kernel double, not evidence of IRQ exclusion. */
#define DI(saved) do { (saved) = 0u; } while (0)
#define EI(saved) do { (void)(saved); } while (0)
#endif
