/* M3: real Vehicle and real Telemetry; only scheduling/devices are doubles.
 * Do not replace admission with a mock: the old implementation must fail.
 */
#define main original_vehicle_main
#define telemetry_running unused_telemetry_running
#define telemetry_stats unused_telemetry_stats
#define telemetry_send unused_telemetry_send
#define app_start_task base_start_task
#define tk_slp_tsk base_sleep
#define drv_motor_hw_button_down base_button
#define motion_set_speed base_set_speed
#include "test_vehicle_task.c"
#undef motion_set_speed
#undef drv_motor_hw_button_down
#undef tk_slp_tsk
#undef app_start_task
#undef telemetry_send
#undef telemetry_stats
#undef telemetry_running
#undef main
#include <tk/smp.h>
#include "../app_program/app.h"
#include "../app_program/vehicle/vehicle_board.h"
#include "../app_program/subsystem1/robot_network.h"

/* Header declarations inside the reused fixture were renamed too. */
bool telemetry_running(void);
void telemetry_stats(telemetry_stats_t *out);
int telemetry_send(const telemetry_message_t *message);
static FP telemetry_entry;
static jmp_buf telemetry_done;
static telemetry_message_t messages[64];
static unsigned int message_head, message_tail, begins, motor_calls;
static bool initial_command, collision_command, fresh_command, released;
static uint32_t collision_ms, calibration_before, run_before;
static unsigned int motors_before;
static uint8_t opcode;
static bool verified_block, verified_fresh;

ID app_start_task(FP function, PRI priority, UW core, const char *name)
{
    if (priority == 30) {
        assert(core == app_prc_core1());
        telemetry_entry = function;
        return 8;
    }
    return base_start_task(function, priority, core, name);
}

ID tk_cre_mbf(const T_CMBF *config)
{
    assert(config->maxmsz == (INT)sizeof(telemetry_message_t));
    assert(config->bufsz == 4096);
    return 1;
}

ER tk_snd_mbf(ID id, const void *message, INT size, TMO timeout)
{
    assert(id == 1 && size == (INT)sizeof(telemetry_message_t));
    assert(timeout == TMO_POL && message_head - message_tail < 64u);
    const telemetry_message_t *value = message;
    messages[message_head++ % 64u] = *value;
    if (value->kind == TELEMETRY_BEGIN) ++begins;
    return E_OK;
}

INT tk_rcv_mbf(ID id, void *message, TMO timeout)
{
    assert(id == 1 && timeout == TMO_FEVR);
    if (message_head == message_tail) longjmp(telemetry_done, 1);
    *(telemetry_message_t *)message = messages[message_tail++ % 64u];
    return (INT)sizeof(telemetry_message_t);
}

static void pump_telemetry(void)
{
    if (setjmp(telemetry_done) == 0)
        ((void (*)(INT, void *))telemetry_entry)(0, NULL);
}

ER tk_dly_tsk(RELTIM delay) { assert(delay == 20u); return E_OK; }
const char *drv_watchdog_reset_name(void) { return "host"; }
void app_task_report(void) { }
void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
int robot_event_push(const rm_event_t *event) { assert(event); return E_OK; }
void robot_command_lost(void) { assert(0); }
int robot_network_dump(uint32_t request_id)
{
    (void)request_id;
    assert(0);
    return E_OBJ;
}

int robot_command_pop(robot_command_t *command)
{
    uint8_t next;
    if (!initial_command && now >= 20u) {
        initial_command = true;
        next = RM_CMD_CALIBRATE;
    } else if (collision_ms != 0u && !collision_command) {
        collision_command = true;
        next = opcode;
    } else if (released && !fresh_command) {
        fresh_command = true;
        next = opcode;
    } else {
        return E_TMOUT;
    }
    *command = (robot_command_t){.opcode = next,
        .request_id = fresh_command ? 3u : (collision_command ? 2u : 1u)};
    return (INT)sizeof(*command);
}

bool drv_motor_hw_button_down(unsigned int pin)
{
    /* Hold across one complete button cadence; its phase can be 10 ms
     * off the host's even-20-ms injection boundary after calibration. */
    return pin == 20u && collision_ms != 0u && now >= collision_ms &&
        now < collision_ms + 20u;
}

int motion_set_speed(int16_t left, int16_t right)
{
    ++motor_calls;
    return base_set_speed(left, right);
}

ER tk_slp_tsk(TMO timeout)
{
    const ER result = base_sleep(timeout);
    vehicle_snapshot_t value;
    assert(vehicle_snapshot_try_read(&value) == E_OK);
    if (collision_ms == 0u) {
        pump_telemetry();
        if (initial_command && value.calibrated && !value.active &&
            value.phase == 0u && now % 20u == 0u) {
            collision_ms = now;
            calibration_before = vehicle_calibration_epoch();
            run_before = vehicle_run_epoch();
            motors_before = motor_calls;
            assert(begins == 1u);
        }
    } else if (!released) {
        if (now >= collision_ms + 20u) {
            telemetry_stats_t state;
            telemetry_stats(&state);
            assert(collision_command);
            assert(state.busy);
            assert(begins == 1u && motor_calls == motors_before);
            assert(vehicle_calibration_epoch() == calibration_before);
            assert(vehicle_run_epoch() == run_before && !value.active);
            verified_block = true;
        }
        if (now >= collision_ms + 40u) {
            pump_telemetry();
            released = true;
        }
    } else {
        pump_telemetry();
        if (fresh_command && begins == 2u &&
            (opcode == RM_CMD_START ? vehicle_run_epoch() != run_before :
             vehicle_calibration_epoch() != calibration_before)) {
            verified_fresh = true;
            longjmp(done, 1);
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    opcode = argc > 1 && !strcmp(argv[1], "calibrate") ?
        RM_CMD_CALIBRATE : RM_CMD_START;
    lp_init(&line);
    lp_set_pair(&line, true, 0);
    watchdog_ready = true;
    services_ready = true;
    assert(telemetry_init() == E_OK && telemetry_start() == E_OK);
    assert(vehicle_task_start() == E_OK && starts == 1u);
    if (setjmp(done) == 0) ((void (*)(INT, void *))entry)(0, NULL);
    assert(verified_block && verified_fresh && released && fresh_command);
    return 0;
}
