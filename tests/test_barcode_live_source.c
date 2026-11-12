#include <assert.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem3/barcode_live_source.h"

extern void barcode_live_test_publish(const barcode_live_record_t *value);
static unsigned int interrupt;
static barcode_live_record_t replacement;
static barcode_live_control_t context;

void barcode_live_test_read_hook(const uint32_t *sequence, unsigned int index)
{
    (void)sequence;
    if (index != 0u) {
        return;
    }
    const unsigned int action = interrupt;
    interrupt = 0u;
    if (action == 1u) {
        barcode_live_test_publish(&replacement);
    } else if (action == 2u) {
        assert(barcode_live_control_publish(&context) == E_OK);
    }
}

int main(void)
{
    barcode_live_record_t record = {.epoch = 77u};
    barcode_live_control_t control = {.epoch = 88u};
    uint32_t tag = 99u;
    assert(barcode_live_source_try_read(NULL) == E_PAR);
    assert(barcode_live_source_try_read(&record) == E_OBJ);
    assert(record.epoch == 77u);
    assert(barcode_live_control_try_read(NULL, &tag) == E_PAR);
    assert(barcode_live_control_try_read(&control, &tag) == E_OBJ);
    assert(control.epoch == 88u && tag == 99u);
    assert(barcode_live_control_publish(NULL) == E_PAR);
    assert(barcode_live_control_publish(&control) == E_OBJ);
    assert(barcode_live_source_init() == E_OK);
    assert(barcode_live_source_try_read(&record) == E_OK);
    assert(record.valid == 0u && record.source == 2u && record.version == 1u);
    assert(record.sequence == 0u && record.flags == BC_LIVE_SHADOW);
    assert(barcode_live_control_try_read(&control, &tag) == E_TMOUT);
    assert(control.epoch == 88u && tag == 99u);

    context = (barcode_live_control_t){
        .epoch = 7u, .eligibility_revision = 3u, .revision_ms = 500u,
        .ack_sequence = 12u, .mode = BC_LIVE_SHADOW_MODE};
    assert(barcode_live_control_publish(&context) == E_OK);
    assert(barcode_live_control_try_read(&control, &tag) == E_OK);
    assert(memcmp(&context, &control, sizeof(control)) == 0);
    const uint32_t first = tag;
    assert(first > 0u && (first & 1u) == 0u);
    assert(barcode_live_source_init() == E_OK); /* No reset by reader startup. */
    assert(barcode_live_control_try_read(&control, &tag) == E_OK && tag == first);
    assert(barcode_live_control_publish(&context) == E_OK);
    assert(barcode_live_control_try_read(&control, &tag) == E_OK && tag != first);
    const uint32_t previous = tag;
    context.mode = BC_LIVE_DISABLED;
    context.ack_sequence = 13u; /* BUSY + ACK in one coherent value. */
    interrupt = 2u;
    assert(barcode_live_control_try_read(&control, &tag) == E_TMOUT);
    assert(tag == previous && control.ack_sequence == 12u);
    assert(barcode_live_control_try_read(&control, &tag) == E_OK);
    assert(control.mode == BC_LIVE_DISABLED && control.ack_sequence == 13u);
    assert(tag != previous);
    assert(barcode_live_control_try_read(&control, NULL) == E_OK);
    context.mode = 3u;
    assert(barcode_live_control_publish(&context) == E_PAR);
    assert(barcode_live_control_try_read(&control, NULL) == E_OK);
    assert(control.mode == BC_LIVE_DISABLED);

    replacement = (barcode_live_record_t){
        .epoch = 7u, .sequence = 13u, .read_ms = 900u,
        .read_counts = {-2, 31}, .command = 1u, .valid = 0u,
        .source = 2u, .frame_end_counts = {-20, 13},
        .eligibility_revision = 3u, .letter = 'A', .direction = 2u,
        .flags = BC_LIVE_SHADOW | BC_LIVE_REQUIRED, .version = 1u};
    interrupt = 1u;
    assert(barcode_live_source_try_read(&record) == E_TMOUT);
    assert(record.sequence == 0u && record.valid == 0u);
    assert(barcode_live_source_try_read(&record) == E_OK);
    assert(memcmp(&record, &replacement, sizeof(record)) == 0);
    assert(barcode_live_source_init() == E_OK);
    assert(barcode_live_source_try_read(&record) == E_OK);
    assert(record.sequence == 13u);
    return 0;
}
