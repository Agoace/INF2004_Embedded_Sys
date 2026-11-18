/* Live-only Vehicle-owner seam; no shared state or hardware calls. */
#include "policy.h"

static blv_t policy;

void blv_init(ja_t *s, int16_t speed, uint32_t epoch)
{
    policy = (blv_t){0};
    ja_init(s, speed, epoch);
}

void blv_pending_route(ja_t *s, uint8_t module, const int32_t counts[2],
                       uint32_t now)
{
    blv_policy_route(&policy, s, module, counts, now);
}
