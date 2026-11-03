#include "barcode_interface.h"

/*
 * Link-time provider for branches that consume the interface before the
 * barcode implementation is merged. The implementation branch replaces this
 * file; it must preserve the public declarations in barcode_interface.h.
 */
void barcode_init(const barcode_config_t *config, bool initial_level,
                  uint32_t now_us) {
    (void)config;
    (void)initial_level;
    (void)now_us;
}

void barcode_sample(bool level, uint32_t now_us) {
    (void)level;
    (void)now_us;
}

bool barcode_take_result(barcode_result_t *result) {
    (void)result;
    return false;
}
