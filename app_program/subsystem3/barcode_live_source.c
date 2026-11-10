/* Canonical boards only; no live decoder feeding or actuation in this step. */
#include "subsystem3.h"
#include "barcode_live_source.h"
#if BC_LIVE_SOURCE_HOST_TEST
extern void barcode_live_test_read_hook(const uint32_t *sequence,
                                        unsigned int index);
#define APP_BOARD_READ_HOOK(s, i) barcode_live_test_read_hook((s), (i))
#endif
#include "../platform/app_board.h"

static bool ready;
static APP_BOARD(barcode_live_record_t) record_board;
static APP_BOARD(barcode_live_control_t) control_board;
_Static_assert(sizeof(record_board) == 44u, "Live record board budget");
_Static_assert(sizeof(control_board) == 24u, "Live control board budget");

int barcode_live_source_init(void)
{
    if (!ready) {
        const barcode_live_record_t empty = {
            .source = BC_LIVE_SOURCE, .version = BC_LIVE_VERSION,
            .flags = BC_LIVE_SHADOW};
        app_board_publish(&record_board.sequence, record_board.words,
                          &empty, sizeof(empty));
        /* Control remains unpublished until its SOLE vehicle writer runs. */
        ready = true;
    }
    return E_OK;
}

int barcode_live_source_try_read(barcode_live_record_t *out)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    barcode_live_record_t value;
    if (!app_board_try_read(&record_board.sequence, record_board.words,
                           &value, sizeof(value))) {
        return E_TMOUT;
    }
    *out = value;
    return E_OK;
}

int barcode_live_control_publish(const barcode_live_control_t *control)
{
    if (!control) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    if (control->mode > BC_LIVE_PERMITTED) {
        return E_PAR;
    }
    app_board_publish(&control_board.sequence, control_board.words,
                      control, sizeof(*control));
    return E_OK;
}

int barcode_live_control_try_read(barcode_live_control_t *out,
                                  uint32_t *publication_tag)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    const uint32_t before =
        __atomic_load_n(&control_board.sequence, __ATOMIC_SEQ_CST);
    barcode_live_control_t value;
    if (!app_board_try_read(&control_board.sequence, control_board.words,
                           &value, sizeof(value)) ||
        __atomic_load_n(&control_board.sequence, __ATOMIC_SEQ_CST) != before) {
        return E_TMOUT;
    }
    *out = value;
    if (publication_tag) {
        *publication_tag = before;
    }
    return E_OK;
}

#if BC_LIVE_SOURCE_HOST_TEST
/* Test-only interrupted-write injection. No record setter is exported in
 * production: S3 remains the sole record producer, never the vehicle. */
void barcode_live_test_publish(const barcode_live_record_t *value)
{
    app_board_publish(&record_board.sequence, record_board.words,
                      value, sizeof(*value));
}
#endif
