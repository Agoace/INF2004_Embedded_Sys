/* SDK-only: no clocks/IRQ/time runtime and no kernel ADC device. */
#include "drv_line_hw.h"
#include "hardware/adc.h"
#include "hardware/resets.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/timer.h"
int drv_line_hw_init(void) {
    /* Kernel startup supplies PLL_USB=48 MHz. Never use SDK clock state. */
    if ((pll_usb_hw->cs & PLL_CS_LOCK_BITS) == 0u) return -1;
    const uint32_t cfg = CLOCKS_CLK_ADC_CTRL_ENABLE_BITS;
    if ((clocks_hw->clk[clk_adc].ctrl & (CLOCKS_CLK_ADC_CTRL_ENABLE_BITS |
         CLOCKS_CLK_ADC_CTRL_AUXSRC_BITS | CLOCKS_CLK_ADC_CTRL_KILL_BITS)) != cfg ||
         clocks_hw->clk[clk_adc].div != 256u) {
        clocks_hw->clk[clk_adc].ctrl = 0u;
        clocks_hw->clk[clk_adc].div = 256u;
        clocks_hw->clk[clk_adc].ctrl = cfg; /* AUXSRC 0 = PLL_USB */
    }
    reset_unreset_block_num_wait_blocking(RESET_ADC);
    adc_init();
    adc_gpio_init(27u); /* left ADC1 */
    adc_gpio_init(26u); /* centre ADC0; never ADC3 / GP29 */
    adc_gpio_init(28u); /* barcode ADC2: no pulls, digital input disabled */
    timer_hw->inte |= 1u;
    return 0;
}
int drv_line_hw_read(uint16_t raw[3]) {
    static const unsigned int channel[3] = {1u,0u,2u};
    for (unsigned int s = 0u; s < 3u; ++s) {
        adc_select_input(channel[s]);
        uint32_t sum = 0u;
        for (unsigned int i = 0u; i < 8u; ++i) {
            /* Bound a failed ADC conversion instead of hanging an interrupt. */
            adc_hw->cs |= ADC_CS_START_ONCE_BITS;
            unsigned int retry = 10000u;
            while ((adc_hw->cs & ADC_CS_READY_BITS) == 0u && --retry != 0u) { }
            if (retry == 0u) return -1;
            sum += adc_hw->result;
        }
        raw[s] = (uint16_t)(sum / 8u);
    }
    return 0;
}
void drv_line_hw_alarm(uint32_t at_us) {
    timer_hw->alarm[0] = at_us;
    if ((int32_t)(at_us - timer_hw->timerawl) <= 0) timer_hw->intf |= 1u;
}
void drv_line_hw_ack(void) { timer_hw->intf &= ~1u; timer_hw->intr = 1u; }
