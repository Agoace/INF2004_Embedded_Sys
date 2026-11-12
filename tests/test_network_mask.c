/* M2: deterministic second-writer updates around the production mask RMW.
 * Include the real implementation to instrument the OLD store boundary and
 * the NEW atomic-operation boundary, without production test hooks.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <tk/tkernel.h>
#include <tk/smp.h>

void mask_test_store(void *address, UW value, size_t size, int order);
UW mask_test_set(UW *address, UW bits);
UW mask_test_clear(UW *address, UW bits);
#define __atomic_store_n(p, v, o) mask_test_store((p), (v), sizeof(*(p)), (o))
#define atomic_bitset mask_test_set
#define atomic_bitclr mask_test_clear
#include "../app_program/subsystem1/network_queue.c"
#undef __atomic_store_n
#undef atomic_bitset
#undef atomic_bitclr

static bool armed;
static unsigned int injections;
static unsigned int other_board;
static int other_result;

static void inject(void *address)
{
    if (address == (void *)&unavailable_mask && armed) {
        armed = false;
        ++injections;
        robot_network_board_result(other_board, other_result);
    }
}

void mask_test_store(void *address, UW value, size_t size, int order)
{
    inject(address);
    if (size == sizeof(uint8_t)) {
        __atomic_store_n((uint8_t *)address, (uint8_t)value, order);
    } else {
        assert(size == sizeof(UW));
        __atomic_store_n((UW *)address, value, order);
    }
}

UW mask_test_set(UW *address, UW bits)
{
    inject(address);
    return atomic_bitset(address, bits);
}

UW mask_test_clear(UW *address, UW bits)
{
    inject(address);
    return atomic_bitclr(address, bits);
}

/* No queue operation is allowed in a board-result update. */
ID tk_cre_mbf(const T_CMBF *value) { (void)value; assert(0); return E_SYS; }
ER tk_del_mbf(ID id) { (void)id; assert(0); return E_SYS; }
ER tk_snd_mbf(ID id, const void *value, INT size, TMO timeout)
{
    (void)id; (void)value; (void)size; (void)timeout;
    assert(0); return E_SYS;
}
INT tk_rcv_mbf(ID id, void *value, TMO timeout)
{
    (void)id; (void)value; (void)timeout;
    assert(0); return E_SYS;
}

int main(int argc, char **argv)
{
    assert(argc <= 2);
    const unsigned int scenario = argc == 2 ?
        (unsigned int)strtoul(argv[1], NULL, 10) : 0u;
    assert(scenario < 6u);
    const UW motion_bit = 1u << ROBOT_BOARD_MOTION;
    const UW vehicle_bit = 1u << ROBOT_BOARD_VEHICLE;
    const bool outer_clear = scenario == 1u || scenario == 3u || scenario == 5u;
    const bool inner_clear = scenario == 2u || scenario == 3u || scenario == 4u;
    const bool same_bit = scenario >= 4u;
    unavailable_mask = (uint8_t)((outer_clear ? motion_bit : 0u) |
        (inner_clear && !same_bit ? vehicle_bit : 0u));
    other_board = same_bit ? ROBOT_BOARD_MOTION : ROBOT_BOARD_VEHICLE;
    other_result = inner_clear ? E_OK : E_OBJ;
    /* Both profiles preserve the caller's exact interrupt state. SMP0 models
     * preemption before the RMW; SMP1 models the other processor there. */
    for (UINT irq = 0u; irq < 2u; ++irq) {
        test_smp_irq_mask = irq;
        armed = true;
        robot_network_board_result(ROBOT_BOARD_MOTION,
                                   outer_clear ? E_OK : E_OBJ);
        assert(!armed && injections == (unsigned int)irq + 1u);
        rm_heartbeat_t metrics = {0};
        robot_queue_metrics(&metrics);
        const UW expected = (outer_clear ? 0u : motion_bit) |
            (!same_bit && !inner_clear ? vehicle_bit : 0u);
        assert(metrics.board_unavailable_mask == expected);
        assert(test_smp_irq_mask == irq);
        assert(test_smp_saves == test_smp_restores);
    }
    return 0;
}
