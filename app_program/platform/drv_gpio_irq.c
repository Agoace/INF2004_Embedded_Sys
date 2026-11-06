#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "drv_gpio_irq.h"
#include "drv_gpio_irq_hw.h"
#define CLIENTS 8u
typedef struct { uint32_t pins; drv_gpio_irq_fn fn; void *ctx; } client_t;
static client_t clients[CLIENTS];
static unsigned int used;
static int ready;
static void gpio_dispatch(UINT intno) {
    for (unsigned int group = 0u; group < 4u; ++group) {
        const uint32_t pending = drv_gpio_irq_hw_pending(group);
        for (unsigned int bit = 0u; bit < 8u; ++bit) {
            const unsigned int pin = group * 8u + bit;
            const uint32_t events = (pending >> (bit * 4u)) & 12u;
            if (pin >= 30u || events == 0u) continue;
            drv_gpio_irq_hw_ack(pin, events);
            for (unsigned int i = 0u; i < used; ++i)
                if ((clients[i].pins & (1u << pin)) != 0u)
                    clients[i].fn(pin, events, clients[i].ctx);
        }
    }
    ClearInt(intno);
}
int drv_gpio_irq_init(void) {
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) return (int)E_CTX;
#endif
    if (ready) return (int)E_OK;
    const T_DINT d = {.intatr = TA_HLNG, .inthdr = (FP)gpio_dispatch};
    DisableInt(DRV_GPIO_IRQ_NUMBER);
    const ER er = tk_def_int(DRV_GPIO_IRQ_NUMBER, &d);
    if (er < E_OK) return (int)er;
    used = 0u;
    ClearInt(DRV_GPIO_IRQ_NUMBER);
    /* USB IRQ5 remains reset priority 0; GPIO 1; SysTick/PendSV 3. */
    EnableInt(DRV_GPIO_IRQ_NUMBER, DRV_GPIO_IRQ_PRIORITY);
    ready = 1;
    return (int)E_OK;
}
int drv_gpio_irq_add(uint32_t pins, drv_gpio_irq_fn fn, void *ctx) {
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) return (int)E_CTX;
#endif
    if (!ready || !fn || pins == 0u || (pins & 0xC0000000u) != 0u) return (int)E_PAR;
    UINT irq;
    DI(irq);
    if (used == CLIENTS) { EI(irq); return (int)E_LIMIT; }
    for (unsigned int i = 0u; i < used; ++i) {
        if ((clients[i].pins & pins) != 0u) { EI(irq); return (int)E_OBJ; }
    }
    clients[used++] = (client_t){pins, fn, ctx};
    drv_gpio_irq_hw_enable(pins);
    EI(irq);
    return (int)E_OK;
}
