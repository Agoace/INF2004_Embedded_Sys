/* Virtual GPIO status, consumed by the unchanged native GPIO dispatcher. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_gpio.h"
static uint32_t pending[4];
static uint32_t enabled;
static uint32_t acked;
static uint8_t ab[2];
static const unsigned int pin_a[2] = {16u, 4u};
static const unsigned int pin_b[2] = {17u, 5u};

void bk_gpio_reset(void)
{
    enabled = 0u;
    acked = 0u;
    ab[0] = 0u;
    ab[1] = 0u;
    for (unsigned int i = 0u; i < 4u; ++i) { pending[i] = 0u; }
}
void bk_gpio_enable(uint32_t pins)
{
    enabled |= pins & 0x3fffffffu;
}
uint8_t bk_gpio_ab(unsigned int wheel)
{
    return wheel < 2u ? ab[wheel] : 0u;
}
uint32_t bk_gpio_pending(unsigned int group)
{
    return group < 4u ? pending[group] : 0u;
}
uint32_t bk_gpio_acked(void)
{
    return acked;
}
void bk_gpio_ack(unsigned int pin, uint32_t events)
{
    if (pin < 30u) {
        const uint32_t bits = (events & 12u) << ((pin % 8u) * 4u);
        const uint32_t present = pending[pin / 8u] & bits;
        /* Only one event is published per pin until its actual ACK. */
        if (present != 0u) { ++acked; }
        pending[pin / 8u] &= ~bits;
    }
}
bool bk_gpio_edge(unsigned int wheel, uint8_t next_ab)
{
    if (wheel >= 2u || next_ab > 3u) { return false; }
    const uint8_t change = ab[wheel] ^ next_ab;
    if (change != 1u && change != 2u) { return false; }
    const unsigned int a = pin_a[wheel], b = pin_b[wheel];
    const unsigned int group = a / 8u;
    const uint32_t mask = (12u << ((a % 8u) * 4u)) |
                          (12u << ((b % 8u) * 4u));
    /* AB must not change before the ISR reads the previous published edge. */
    if ((pending[group] & mask) != 0u) { return false; }
    const unsigned int pin = change == 2u ? a : b;
    if ((enabled & (1u << pin)) == 0u) { return false; }
    const uint32_t event = (next_ab & change) != 0u ? 8u : 4u;
    ab[wheel] = next_ab;
    pending[group] |= event << ((pin % 8u) * 4u);
    return true;
}
#endif
