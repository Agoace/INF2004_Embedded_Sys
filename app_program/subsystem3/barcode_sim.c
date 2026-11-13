/* Stage 0: no synthetic keypress; real console readiness is not forged. */
#include "barcode_hw.h"
#include "../sim/pil_stage0.h"
#if APP_BARCODE_CAPTURE
extern unsigned long tm_usb_state(void);

bool barcode_hw_usb_ready(void)
{
    return tm_usb_state() == 3UL;
}

void barcode_hw_button_init(void)
{
}

bool barcode_hw_button_down(void)
{
    return false;
}

bool barcode_hw_drive_down(void)
{
    return false;
}
#endif
