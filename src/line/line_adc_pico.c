/**
 * @file line_adc_pico.c
 * @brief Pico implementation of the line-sensor ADC seam.
 *
 * Physical order comes from the YX operator note (yx_line_tracing @1ea095c,
 * 2026-10-27): on this robot GP28 is the LEFT sensor and GP26 is the RIGHT one,
 * the opposite of the naming in `ir_module_pins.h`. The mapping is defined here
 * on purpose. `ir_module_pins.h` is byte-identical across the worktrees and is
 * also used by the standalone IR diagnostics, so editing it would silently
 * change those targets too.
 */
#include "line_adc.h"

#include "hardware/adc.h"

/** Physical sides, seen from behind the robot. */
#define LINE_ADC_LEFT_GPIO   28u
#define LINE_ADC_CENTRE_GPIO 27u
#define LINE_ADC_RIGHT_GPIO  26u

/** ADC channel 0 is GP26. */
#define LINE_ADC_ADC_BASE    26u

/** Averaged samples per sensor per reading. */
#define LINE_ADC_AVERAGE     8u

static const uint8_t g_line_adc_gpio[LINE_ADC_SENSORS] =
{
    LINE_ADC_LEFT_GPIO, LINE_ADC_CENTRE_GPIO, LINE_ADC_RIGHT_GPIO,
};

bool
line_adc_init (void)
{
    adc_init();

    for (uint32_t s = 0u; s < LINE_ADC_SENSORS; s++)
    {
        adc_gpio_init(g_line_adc_gpio[s]);
    }

    return true;
}

void
line_adc_read (uint16_t raw[LINE_ADC_SENSORS])
{
    if (NULL == raw)
    {
        return;
    }

    for (uint32_t s = 0u; s < LINE_ADC_SENSORS; s++)
    {
        adc_select_input(g_line_adc_gpio[s] - LINE_ADC_ADC_BASE);

        uint32_t sum = 0u;

        for (uint32_t i = 0u; i < LINE_ADC_AVERAGE; i++)
        {
            sum += adc_read();
        }

        raw[s] = (uint16_t) (sum / LINE_ADC_AVERAGE);
    }
}
