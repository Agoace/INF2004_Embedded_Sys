/* Host dispatcher test: no peripheral or timing claim. */
#include <assert.h>
#include <tk/tkernel.h>
#include "../app_program/platform/drv_gpio_irq.h"
static FP handler;
static int core = 1, irq_off, enabled_level = -1;
static uint32_t pending[4], acknowledged[30], enabled_pins;
static unsigned int calls;
INT tk_get_prc(void) { return core; }
ER tk_def_int(UINT irq, const T_DINT *d) {
    assert(irq == 13u && d->intatr == TA_HLNG); handler = d->inthdr; return E_OK;
}
void DisableInt(UINT irq) { assert(irq == 13u); }
void ClearInt(UINT irq) { assert(irq == 13u); }
void EnableInt(UINT irq, INT priority) { assert(irq == 13u); enabled_level = priority; }
void mock_di(UINT *irq) { assert(!irq_off); *irq = 0u; irq_off = 1; }
void mock_ei(UINT irq) { assert(irq == 0u && irq_off); irq_off = 0; }
uint32_t drv_gpio_irq_hw_pending(unsigned int group) { return pending[group]; }
void drv_gpio_irq_hw_ack(unsigned int pin, uint32_t events) { acknowledged[pin] |= events; }
void drv_gpio_irq_hw_enable(uint32_t pins) { assert(irq_off); enabled_pins |= pins; }
static void callback(unsigned int pin, uint32_t events, void *ctx) {
    assert(ctx == &calls && (acknowledged[pin] & events) == events); ++calls;
}
int main(void) {
    assert(drv_gpio_irq_init() == E_OK && enabled_level == 1);
    assert(drv_gpio_irq_add((1u << 16u) | (1u << 17u), callback, &calls) == E_OK);
    assert(enabled_pins == ((1u << 16u) | (1u << 17u)));
    assert(drv_gpio_irq_add(1u << 16u, callback, &calls) == E_OBJ);
    assert(drv_gpio_irq_add(0u, callback, &calls) == E_PAR);
    assert(drv_gpio_irq_add(1u << 30u, callback, &calls) == E_PAR);
    pending[2] = 4u | (8u << 4u);
    ((void (*)(UINT))handler)(13u);
    assert(calls == 2u && acknowledged[16] == 4u && acknowledged[17] == 8u);
    for (unsigned int pin = 0u; pin < 7u; ++pin)
        assert(drv_gpio_irq_add(1u << pin, callback, &calls) == E_OK);
    assert(drv_gpio_irq_add(1u << 7u, callback, &calls) == E_LIMIT);
    core = 2;
    assert(drv_gpio_irq_init() == E_CTX);
    assert(drv_gpio_irq_add(1u << 8u, callback, &calls) == E_CTX);
    return 0;
}
