/* B1: actual kernel lock functions and production protected sections.
 * Model only the Cortex-M IRQ and atomic primitives, not lock ownership.
 * This tests deferred same-core preemption, not real-time kernel scheduling.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_program/subsystem1/mqtt_wire.h"
#include "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"

#define EXPORT
#define E_OK 0
#define E_PAR (-17)
typedef int ER;
typedef unsigned int UINT;
typedef uint32_t UW;
typedef UW T_SPLOCK;

static T_SPLOCK metrics_lock;
static T_SPLOCK status_lock;
static rm_heartbeat_t metrics;
static bool subscribed;
static bool commands_enabled;
static T_CYW43_UTK_STATUS radio_status;
static T_CYW43_UTK_STATUS radio_status_pub;
static UINT irq_mask;
static T_SPLOCK *pending_lock;
static bool in_preemptor;
static bool remote_busy;
static unsigned int retries;
static unsigned int acquisitions;
static uint32_t preemptions;

static void preemptor(T_SPLOCK *lock);

static UINT disint(void)
{
    const UINT saved = irq_mask;
    irq_mask = 1u;
    return saved;
}

static bool knl_arch_atomic_cmpxchg(volatile UW *address, UW old, UW value)
{
    assert(irq_mask == 1u);
    if (*address != old) {
        return false;
    }
    *address = value;
    ++acquisitions;
    /* A higher-priority task becomes ready during the section, but PRIMASK
     * prevents its dispatch until the holder restores interrupts. */
    if (!in_preemptor) {
        pending_lock = (T_SPLOCK *)address;
    }
    return true;
}

static void knl_arch_cpu_relax(void)
{
    /* Simulated remote holder releases after one failed claim. The local
     * caller must remain non-preemptible while waiting for the other core. */
    assert(remote_busy && irq_mask == 1u);
    remote_busy = false;
    metrics_lock = 0u;
    ++retries;
}

static void knl_arch_atomic_store(volatile UW *address, UW value)
{
    assert(irq_mask == 1u && *address == 1u && value == 0u);
    *address = value;
}

static void set_primask(UINT saved)
{
    irq_mask = saved;
    if (pending_lock != NULL && irq_mask == 0u) {
        T_SPLOCK *lock = pending_lock;
        pending_lock = NULL;
        assert(*lock == 0u);
        in_preemptor = true;
        preemptor(lock);
        in_preemptor = false;
    }
}

#include "lock_sections.inc"

static void preemptor(T_SPLOCK *lock)
{
    assert(*lock == 0u && irq_mask == 0u);
    if (lock == &metrics_lock) {
        inc(&preemptions);
    } else {
        T_CYW43_UTK_STATUS copy;
        assert(lock == &status_lock);
        cyw43_utk_get_status(&copy);
        assert(copy.poll_count == radio_status_pub.poll_count);
        ++preemptions;
    }
}

static void run_sections(UINT initial_mask)
{
    irq_mask = initial_mask;
    const uint32_t before = preemptions;
    uint32_t counter = 0u;
    inc(&counter);
    assert(counter == 1u);
    metrics_state_set(2u);
    metrics_subscribed_set(false);
    metrics_subscribed_and_state(true, 3u);
    metrics_commands_enabled_set(true);
    metrics_sonar_missed_add(7u);
    metrics_sonar_sequence_set(123u);
    metrics.wifi_observed_priority = 0u;
    assert(metrics_observed_priority_unset());
    metrics_observed_priority_set_if_unset(20u);
    metrics_observed_priority_set_if_unset(30u);
    assert(!metrics_observed_priority_unset());
    assert(metrics.wifi_observed_priority == 20u);
    metrics_hook_max_update(300u);
    metrics_hook_max_update(100u);
    assert(metrics.mqtt_hook_max_us == 300u);
    snapshot_read();
    assert(metrics.mqtt_state == 3u && subscribed && commands_enabled);
    radio_status.poll_count = 999u;
    radio_status.link_status = 3;
    status_publish();
    T_CYW43_UTK_STATUS copy;
    cyw43_utk_get_status(&copy);
    assert(copy.poll_count == 999u && copy.link_status == 3);
    const unsigned int prior_calls = acquisitions;
    cyw43_utk_get_status(NULL);
    assert(acquisitions == prior_calls);
    assert(irq_mask == initial_mask);
    assert(metrics_lock == 0u && status_lock == 0u);
    /* 16 calls cover all 13 acquisition sites and conditional branches. */
    assert(preemptions == before + (initial_mask == 0u ? 16u : 0u));
    /* An initially masked caller leaves scheduling to its enclosing section. */
    pending_lock = NULL;
}

int main(void)
{
    run_sections(0u);
    run_sections(1u);
    irq_mask = 0u;
    remote_busy = true;
    metrics_lock = 1u;
    uint32_t counter = 0u;
    inc(&counter);
    assert(counter == 1u && retries == 1u && !remote_busy);
    assert(irq_mask == 0u && metrics_lock == 0u);
    printf("B1 PASS: real SMP locks, 13 sites, IRQ restore 0/1, "
           "same-core deferral, remote contention; heartbeat %zu B, "
           "status %zu B\n",
           sizeof(rm_heartbeat_t), sizeof(T_CYW43_UTK_STATUS));
    return 0;
}
