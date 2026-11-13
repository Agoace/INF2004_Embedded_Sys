#ifndef PIL_FAKE_RESETS_H
#define PIL_FAKE_RESETS_H
#include <stdint.h>
typedef struct { uint32_t reset, reset_done; } resets_hw_t;
extern resets_hw_t fake_resets;
#define resets_hw (&fake_resets)
#endif
