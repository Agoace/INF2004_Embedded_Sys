/* Boot-only writer; aligned word loads/stores need no locked RMW. */
#include "app_startup.h"
#include <stdint.h>

static uint32_t complete;

void app_startup_complete_publish(void)
{
    __atomic_store_n(&complete, 1u, __ATOMIC_RELEASE);
}

bool app_startup_complete(void)
{
    return __atomic_load_n(&complete, __ATOMIC_ACQUIRE) != 0u;
}
