#ifndef PIL_FAKE_SYSLIB_H
#define PIL_FAKE_SYSLIB_H
/* Register model only, not evidence of hardware IRQ exclusion/latency. */
#if defined(PIL_GATE1_RUNTIME_TEST) && PIL_GATE1_RUNTIME_TEST
unsigned int pil_test_irq_disable(void);
void pil_test_irq_restore(unsigned int saved);
#define DI(saved) do { (saved) = pil_test_irq_disable(); } while (0)
#define EI(saved) do { pil_test_irq_restore(saved); } while (0)
#else
#define DI(saved) do { (saved) = 0u; } while (0)
#define EI(saved) do { (void)(saved); } while (0)
#endif
#endif
