/* boot_id.c - initialize a fixed boot observation label before app startup. */
#if TM_MQTT
#include "../app.h"
#include "boot_id.h"
#include "boot_id_hw.h"

static uint32_t identity;

/* Repeated initialization preserves this boot's identity. */
int boot_id_init(void)
{
    uint32_t generated;

    if (identity != 0u) {
        return (int)E_OK;
    }
    if (!boot_id_hw_generate(&generated)) {
        return (int)E_SYS;
    }
    identity = generated;
    return (int)E_OK;
}

/* Publication begins only after initialization in the priority-1 task. */
uint32_t boot_id_get(void)
{
    return identity;
}
#endif
