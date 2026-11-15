/* Step0 virtual encoder GPIO, consumed by the unchanged native dispatcher. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_gpio.h"
typedef struct {
    uint32_t pending; /* Unshifted A/B nibbles, one event until ACK. */
    uint8_t ab;
    uint8_t enabled;
} wheel_gpio_t;
static wheel_gpio_t wheels[2];
static uint32_t acked;

void bk_gpio_reset(void)
{
    wheels[0] = (wheel_gpio_t){0};
    wheels[1] = (wheel_gpio_t){0};
    acked = 0u;
}
void bk_gpio_enable(uint32_t pins)
{
    wheels[0].enabled |= (uint8_t)((((pins >> 16u) & 1u) << 1u) |
                                   ((pins >> 17u) & 1u));
    wheels[1].enabled |= (uint8_t)((((pins >> 4u) & 1u) << 1u) |
                                   ((pins >> 5u) & 1u));
}
uint8_t bk_gpio_ab(unsigned int wheel)
{
    return wheel < 2u ? wheels[wheel].ab : 0u;
}
uint32_t bk_gpio_pending(unsigned int group)
{
    if (group == 2u) { return wheels[0].pending; }
    if (group == 0u) { return wheels[1].pending << 16u; }
    return 0u;
}
uint32_t bk_gpio_acked(void) { return acked; }
void bk_gpio_ack(unsigned int pin, uint32_t events)
{
    unsigned int wheel;
    if (pin == 16u || pin == 17u) { wheel = 0u; }
    else if (pin == 4u || pin == 5u) { wheel = 1u; }
    else { return; }
    const unsigned int shift = (pin & 1u) != 0u ? 4u : 0u;
    const uint32_t bits = (events & 12u) << shift;
    if ((wheels[wheel].pending & bits) != 0u) { ++acked; }
    wheels[wheel].pending &= ~bits;
}
bool bk_gpio_edge(unsigned int wheel, uint8_t next_ab)
{
    if (wheel >= 2u || next_ab > 3u) { return false; }
    wheel_gpio_t *const w = &wheels[wheel];
    const uint8_t change = w->ab ^ next_ab;
    if (change != 1u && change != 2u) { return false; }
    if ((w->enabled & change) == 0u || w->pending != 0u) { return false; }
    w->ab = next_ab;
    const uint32_t event = (next_ab & change) != 0u ? 8u : 4u;
    w->pending = event << (change == 1u ? 4u : 0u);
    return true;
}
#endif
