#include <assert.h>
#include <string.h>
#include "../app_program/subsystem3/barcode_attempt.h"
int main(void)
{
    rm_barcode_t value = {.sequence=99u}, out = value;
    assert(!barcode_attempt_peek(&out));
    assert(out.sequence == 99u);
    assert(!barcode_attempt_ack(99u));
    for (uint32_t i=1u;i<=BC_ATTEMPT_CAPACITY;++i) {
        value.sequence=i; value.epoch=7u;
        assert(barcode_attempt_enqueue(&value));
    }
    value.sequence=99u;
    assert(!barcode_attempt_enqueue(&value));
    assert(barcode_attempt_dropped()==1u);
    for (uint32_t i=1u;i<=BC_ATTEMPT_CAPACITY;++i) {
        assert(barcode_attempt_peek(&out) && out.sequence==i);
        assert(!barcode_attempt_ack(i+100u));
        assert(barcode_attempt_peek(&value));
        assert(memcmp(&out,&value,sizeof(out))==0);
        assert(barcode_attempt_ack(i));
    }
    assert(!barcode_attempt_peek(&out));
    for (uint32_t i=20u;i<100u;++i) {
        value.sequence=i;
        assert(barcode_attempt_enqueue(&value));
        assert(barcode_attempt_peek(&out) && out.sequence==i);
        assert(barcode_attempt_ack(i));
    }
    assert(barcode_attempt_dropped()==1u);
    return 0;
}
