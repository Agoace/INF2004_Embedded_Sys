#include <assert.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem3/barcode_live_source.h"

int main(void)
{
    barcode_live_counts_t counts = {.epoch = 17u, .frames = 19u,
                                   .attempts = 23u, .failures = 29u};
    const barcode_live_counts_t sentinel = counts;
    assert(barcode_live_counts_try_read(NULL) == E_PAR);
    assert(barcode_live_counts_try_read(&counts) == E_NOSPT);
    assert(memcmp(&counts, &sentinel, sizeof(counts)) == 0);
    assert(barcode_live_source_init() == E_OK);
    barcode_live_source_sample(1000u, 160u, NULL, 0u, 0u);
    assert(barcode_live_counts_try_read(&counts) == E_NOSPT);
    assert(memcmp(&counts, &sentinel, sizeof(counts)) == 0);
    return 0;
}
