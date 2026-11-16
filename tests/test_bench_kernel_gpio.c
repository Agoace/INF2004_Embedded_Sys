/* Provider seam: no direct native encoder-count writes. */
#include <assert.h>
#include <stdint.h>
#include "../app_program/bench_kernel/bench_gpio.h"
#include "../app_program/platform/drv_gpio_irq_hw.h"

int main(void)
{
    bk_gpio_reset();
    assert(!bk_gpio_edge(0u, 1u)); /* Unsubscribed pins cannot progress. */
    drv_gpio_irq_hw_enable((1u << 16) | (1u << 17) |
                           (1u << 4) | (1u << 5));
    assert(bk_gpio_edge(0u, 1u));
    assert(bk_gpio_ab(0u) == 1u);
    assert(bk_gpio_pending(2u) == (8u << 4)); /* GP17 rises. */
    assert(!bk_gpio_edge(0u, 3u)); /* No silent pending coalescence. */
    assert(bk_gpio_ab(0u) == 1u);
    assert(bk_gpio_edge(1u, 2u)); /* Other wheel can progress. */
    assert(bk_gpio_pending(0u) == (8u << 16)); /* GP4 rises. */
    drv_gpio_irq_hw_ack(17u, 4u); /* Wrong polarity cannot consume rise. */
    assert(bk_gpio_acked() == 0u);
    drv_gpio_irq_hw_ack(17u, 8u);
    assert(bk_gpio_acked() == 1u);
    drv_gpio_irq_hw_ack(17u, 8u); /* Duplicate ACK is not a delivered edge. */
    assert(bk_gpio_acked() == 1u);
    assert(drv_gpio_irq_hw_pending(2u) == 0u);
    assert(bk_gpio_edge(0u, 3u));
    assert(bk_gpio_pending(2u) == 8u); /* GP16 rises. */
    bk_gpio_ack(16u, 8u);
    assert(bk_gpio_edge(0u, 2u));
    assert(bk_gpio_pending(2u) == (4u << 4)); /* GP17 falls. */
    assert(!bk_gpio_edge(2u, 0u));
    assert(!bk_gpio_edge(0u, 4u));
    assert(!bk_gpio_edge(0u, 1u)); /* Two-bit jump is never an edge. */
    assert(bk_gpio_pending(4u) == 0u);
    bk_gpio_ack(31u, 12u); /* Invalid pin must not shift beyond word. */
    bk_gpio_reset();
    assert(bk_gpio_ab(0u) == 0u && bk_gpio_pending(2u) == 0u);
    assert(bk_gpio_acked() == 0u);
    return 0;
}
