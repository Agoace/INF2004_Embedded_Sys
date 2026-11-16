/* Exercise the real release/acquire handoff, not a scheduler timing model. */
#include <assert.h>
#include <threads.h>
#include "../app_program/platform/app_startup.h"

static int provider_ids[6];

static int observer(void *context)
{
    (void)context;
    while (!app_startup_complete()) {
        thrd_yield();
    }
    for (unsigned int i = 0u; i < 6u; ++i) {
        /* One failed provider remains unavailable after the handoff. */
        assert(provider_ids[i] == (i == 2u ? 0 : (int)i + 1));
    }
    return 0;
}

int main(void)
{
    thrd_t reader;
    int result;

    assert(!app_startup_complete());
    assert(thrd_create(&reader, observer, NULL) == thrd_success);
    for (unsigned int i = 0u; i < 6u; ++i) {
        provider_ids[i] = i == 2u ? 0 : (int)i + 1;
        assert(!app_startup_complete());
    }
    app_startup_complete_publish();
    assert(app_startup_complete());
    assert(thrd_join(reader, &result) == thrd_success);
    assert(result == 0);
    return 0;
}
