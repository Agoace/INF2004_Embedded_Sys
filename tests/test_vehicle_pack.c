/* Vehicle observation storage: exact values and bounded coherent reads. */
#include <assert.h>
#include <limits.h>
#include <string.h>
#include "../app_program/vehicle/vehicle_pack.h"

static int interrupt_at = -1;
static uint32_t *shared_words;
static vehicle_packed_t replacement;
static void read_hook(const uint32_t *sequence, unsigned int index);
#define APP_BOARD_READ_HOOK(sequence, index) read_hook(sequence, index)
#include "../app_program/platform/app_board.h"

static void read_hook(const uint32_t *sequence, unsigned int index)
{
    if (interrupt_at >= 0 && index == (unsigned int)interrupt_at) {
        interrupt_at = -1;
        app_board_publish((uint32_t *)sequence, shared_words, &replacement,
                          sizeof(replacement));
    }
}

static vehicle_snapshot_t sample(void)
{
    const vehicle_snapshot_t v = {
        .timestamp_us = UINT32_MAX, .sequence = 1u,
        .wake_lost = UINT32_MAX, .overruns = 0u,
        .response_max_us = UINT32_MAX, .handoffs = 0u,
        .command_id = UINT32_MAX,
        .left_mm_s = INT16_MIN, .right_mm_s = INT16_MAX,
        .servo_deg = -36, .state = 2u, .module = 4u, .phase = 3u,
        .command_opcode = 4u, .command_status = 5u,
        .active = true, .calibrated = true
    };
    return v;
}

static void equal(const vehicle_snapshot_t *a, const vehicle_snapshot_t *b)
{
#define SAME(field) assert(a->field == b->field)
    SAME(timestamp_us); SAME(sequence); SAME(wake_lost); SAME(overruns);
    SAME(response_max_us); SAME(handoffs); SAME(command_id);
    SAME(left_mm_s); SAME(right_mm_s); SAME(servo_deg);
    SAME(state); SAME(module); SAME(phase); SAME(command_opcode);
    SAME(command_status); SAME(active); SAME(calibrated);
#undef SAME
}

static void round_trip(const vehicle_snapshot_t *v)
{
    vehicle_packed_t p;
    memset(&p, 0xa5, sizeof(p));
    assert(vehicle_pack(&p, v));
    assert((p.words[8] & UINT32_C(0xffc00000)) == 0u);
    vehicle_snapshot_t result;
    memset(&result, 0xa5, sizeof(result));
    assert(vehicle_unpack(&result, &p));
    equal(v, &result);
}

static void ranges(void)
{
    assert(sizeof(vehicle_packed_t) == 36u);
    assert(sizeof(vehicle_snapshot_t) == 40u);
    vehicle_snapshot_t v = sample();
    round_trip(&v);
    for (int speed = INT16_MIN; speed <= INT16_MAX; ++speed) {
        v.left_mm_s = (int16_t)speed;
        v.right_mm_s = (int16_t)(-1 - speed);
        round_trip(&v);
    }
    for (int angle = -36; angle <= 36; ++angle) {
        v.servo_deg = (int8_t)angle;
        round_trip(&v);
    }
    /* Exhaust every combination of the bounded enum and Boolean fields. */
    for (unsigned int state = 0; state <= 2u; ++state)
    for (unsigned int module = 0; module <= 4u; ++module)
    for (unsigned int phase = 0; phase <= 3u; ++phase)
    for (unsigned int opcode = 0; opcode <= 4u; ++opcode)
    for (unsigned int status = 0; status <= 5u; ++status)
    for (unsigned int flags = 0; flags < 4u; ++flags) {
        v.state = (uint8_t)state; v.module = (uint8_t)module;
        v.phase = (uint8_t)phase; v.command_opcode = (uint8_t)opcode;
        v.command_status = (uint8_t)status;
        v.active = (flags & 1u) != 0u;
        v.calibrated = (flags & 2u) != 0u;
        round_trip(&v);
    }
    const uint32_t endpoints[] = {0u, 1u, INT32_MAX, UINT32_C(0x80000000), UINT32_MAX};
    for (unsigned int i = 0u; i < sizeof(endpoints) / sizeof(endpoints[0]); ++i) {
        v.timestamp_us = endpoints[i]; v.sequence = endpoints[i];
        v.wake_lost = endpoints[i]; v.overruns = endpoints[i];
        v.response_max_us = endpoints[i]; v.handoffs = endpoints[i];
        v.command_id = endpoints[i];
        round_trip(&v);
    }
    v = (vehicle_snapshot_t){0};
    round_trip(&v);
}

static void invalid(void)
{
    const vehicle_snapshot_t good = sample();
    vehicle_packed_t original, output;
    assert(vehicle_pack(&original, &good));
    output = original;
    assert(!vehicle_pack(NULL, &good));
    assert(!vehicle_pack(&output, NULL));
    const int angles[] = {INT8_MIN, -37, 37, INT8_MAX};
    for (unsigned int i = 0u; i < sizeof(angles) / sizeof(angles[0]); ++i) {
        vehicle_snapshot_t bad = good;
        bad.servo_deg = (int8_t)angles[i];
        assert(!vehicle_pack(&output, &bad));
        assert(memcmp(&output, &original, sizeof(output)) == 0);
    }
    for (unsigned int code = 0u; code <= UINT8_MAX; ++code) {
#define REJECT(field, maximum) do { \
        if (code > (maximum)) { \
            vehicle_snapshot_t bad = good; bad.field = (uint8_t)code; \
            assert(!vehicle_pack(&output, &bad)); \
            assert(memcmp(&output, &original, sizeof(output)) == 0); \
        } \
    } while (0)
        REJECT(state, 2u); REJECT(module, 4u); REJECT(phase, 3u);
        REJECT(command_opcode, 4u); REJECT(command_status, 5u);
#undef REJECT
    }
    vehicle_snapshot_t decoded = good;
    assert(!vehicle_unpack(NULL, &original));
    assert(!vehicle_unpack(&decoded, NULL));
    equal(&decoded, &good);
    for (unsigned int bit = 22u; bit < 32u; ++bit) {
        vehicle_packed_t bad = original;
        bad.words[8] |= UINT32_C(1) << bit;
        assert(!vehicle_unpack(&decoded, &bad));
        equal(&decoded, &good);
    }
    /* Every unused field code, not only the first invalid code. */
    const unsigned int shifts[] = {0u, 7u, 9u, 14u, 17u};
    const unsigned int masks[] = {127u, 3u, 7u, 7u, 7u};
    const unsigned int maxima[] = {72u, 2u, 4u, 4u, 5u};
    for (unsigned int field = 0u; field < 5u; ++field) {
        for (unsigned int code = maxima[field] + 1u; code <= masks[field]; ++code) {
            vehicle_packed_t bad = original;
            bad.words[8] = (bad.words[8] & ~(masks[field] << shifts[field])) |
                           (code << shifts[field]);
            assert(!vehicle_unpack(&decoded, &bad));
            equal(&decoded, &good);
        }
    }
}

static void boards(void)
{
    APP_BOARD(vehicle_packed_t) board = {0};
    assert(sizeof(board) == 40u);
    shared_words = board.words;
    vehicle_snapshot_t before = sample();
    vehicle_snapshot_t after = {0};
    after.sequence = 2u; after.servo_deg = 36;
    assert(vehicle_pack(&replacement, &after));
    vehicle_packed_t initial, copy;
    assert(vehicle_pack(&initial, &before));
    vehicle_snapshot_t decoded = before;
    /* Unready, odd, wrap and one complete publication after each load. */
    assert(!app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy)));
    for (unsigned int i = 0u; i < 9u; ++i) {
        app_board_publish(&board.sequence, board.words, &initial, sizeof(initial));
        interrupt_at = (int)i;
        assert(!app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy)));
        assert(interrupt_at == -1);
        equal(&decoded, &before);
        assert(app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy)));
        assert(vehicle_unpack(&decoded, &copy));
        equal(&decoded, &after);
        decoded = before;
    }
    board.sequence = 3u;
    assert(!app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy)));
    board.sequence = UINT32_MAX - 1u;
    app_board_publish(&board.sequence, board.words, &replacement, sizeof(replacement));
    assert(board.sequence == 0u);
    assert(!app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy)));
    equal(&decoded, &before);
    app_board_publish(&board.sequence, board.words, &replacement, sizeof(replacement));
    assert(board.sequence == 2u);
    assert(app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy)));
    assert(vehicle_unpack(&decoded, &copy));
    equal(&decoded, &after);
}

int main(int argc, char **argv)
{
    assert(argc <= 2);
    if (argc == 1 || strcmp(argv[1], "ranges") == 0) ranges();
    else if (strcmp(argv[1], "invalid") == 0) invalid();
    else { assert(strcmp(argv[1], "boards") == 0); boards(); }
    return 0;
}
