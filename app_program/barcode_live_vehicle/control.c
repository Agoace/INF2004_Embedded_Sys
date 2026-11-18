/* Single LINE-owner cache. Never used by the source publication recheck. */
#include "control.h"
#include <string.h>

static barcode_live_producer_t *owner;
static barcode_live_control_t last_good;
static uint32_t last_tag;
static bool have_good;
uint32_t blc_control_read_misses;

#if defined(BLC_KERNEL_RAM) && BLC_KERNEL_RAM
__attribute__((section(".data.ramfunc.barcode_live_producer_observe")))
#endif
void barcode_live_producer_observe(barcode_live_producer_t *p,
                                  const barcode_live_input_t *in)
{
    if (owner != p || p->samples == 0u) {
        owner = p;
        have_good = false;
        blc_control_read_misses = 0u;
    }
    if (in->control_ok) {
        last_good = in->control;
        last_tag = in->tag;
        have_good = true;
        blc_producer_uncached(p, in);
        return;
    }
    ++blc_control_read_misses;
    if (!have_good || !p->seen || !p->qualified || p->occupied ||
        in->now_ms - p->success_ms > BC_LIVE_CONTROL_MAX_AGE_MS ||
        in->now_ms - p->progress_ms > BC_LIVE_CONTROL_MAX_AGE_MS) {
        blc_producer_uncached(p, in);
        return;
    }
    barcode_live_input_t cached = *in;
    cached.control = last_good;
    cached.tag = last_tag;
    cached.control_ok = true;
    const uint32_t success = p->success_ms;
    /* The same tag cannot advance progress_ms. Restore success_ms as well:
     * a failed read is never evidence of heartbeat progress or observation.
     * Mask ACK even though occupied frames took the failed-read path above. */
    cached.control.ack_sequence = 0u;
    blc_producer_uncached(p, &cached);
    p->success_ms = success;
}
