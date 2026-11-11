/*
 * network_queue.c - bounded message buffers between the vehicle task and
 * the Wi-Fi task, with loss counters.
 *
 * Three message buffers exist: events (vehicle to network), commands
 * (network to vehicle) and a dedicated ESTOP slot. All sends and receives
 * poll (TMO_POL), so no caller ever waits. A full buffer drops the message
 * and counts it.
 */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tk/smp.h>   /* T_SPLOCK, SpinLock: cross-core queue counters */
#include "robot_network.h"

/* Message buffer sizing: depth in messages; the kernel adds a header. */
#define EVENT_QUEUE_DEPTH   16
#define COMMAND_QUEUE_DEPTH 8
#define MBF_HEADER_BYTES    4

static ID event_queue;
static ID command_queue;
static ID estop_queue;
static uint32_t event_lost;
static uint32_t command_lost;
static uint32_t busy[ROBOT_BOARD_COUNT];
static uint32_t unavailable;
static uint8_t unavailable_mask;

/* Phase 4: the Wi-Fi task on core 1 updates these counters while Health and the
 * board readers copy them on core 0, so a core-local mask protects neither.
 * Task context only, so plain SpinLock is the right variant under the lock rule.
 * A zero-filled static already holds the unlocked state. */
static T_SPLOCK queue_lock;

/* Increment a loss counter. Interrupts are masked for the update. */
static void lost(uint32_t *counter)
{
    (void)SpinLock(&queue_lock);
    ++*counter;
    (void)SpinUnlock(&queue_lock);
}

/*
 * Create the three message buffers once.
 * Returns E_OK (also if already created) or the first kernel error.
 * On error all buffers made so far are deleted.
 */
int robot_network_init(void)
{
    if (event_queue > 0 && command_queue > 0 && estop_queue > 0) {
        return E_OK;
    }
    T_CMBF q = {.mbfatr = TA_TFIFO,
                .bufsz =
                    EVENT_QUEUE_DEPTH *
                    (INT)(sizeof(rm_event_t) + (unsigned int)MBF_HEADER_BYTES),
                .maxmsz = (INT)sizeof(rm_event_t)};
    event_queue = tk_cre_mbf(&q);
    if (event_queue < 0) {
        return (int)event_queue;
    }
    q.bufsz = COMMAND_QUEUE_DEPTH *
              (INT)(sizeof(robot_command_t) + (unsigned int)MBF_HEADER_BYTES);
    q.maxmsz = (INT)sizeof(robot_command_t);
    command_queue = tk_cre_mbf(&q);
    if (command_queue < 0) {
        (void)tk_del_mbf(event_queue);
        event_queue = 0;
        return (int)command_queue;
    }
    q.bufsz = (INT)sizeof(robot_command_t) + MBF_HEADER_BYTES;
    estop_queue = tk_cre_mbf(&q);
    if (estop_queue < 0) {
        (void)tk_del_mbf(event_queue);
        (void)tk_del_mbf(command_queue);
        event_queue = 0;
        command_queue = 0;
        return (int)estop_queue;
    }
    return E_OK;
}

/*
 * Queue one event from the vehicle task. Never blocks.
 * Returns a negative error and counts the loss when the buffer is full.
 */
int robot_event_push(const rm_event_t *event)
{
    if (!event) {
        return E_PAR;
    }
    const ER er = event_queue > 0 ? tk_snd_mbf(event_queue, event,
                                               (INT)sizeof(*event), TMO_POL)
                                  : E_OBJ;
    if (er < E_OK) {
        lost(&event_lost);
    }
    return (int)er;
}

/*
 * Take the oldest event. Never blocks.
 * Returns the message size (> 0), E_TMOUT if empty, or an error.
 */
int robot_event_pop(rm_event_t *out)
{
    if (!out) {
        return E_PAR;
    }
    return event_queue > 0 ? (int)tk_rcv_mbf(event_queue, out, TMO_POL)
                           : (int)E_OBJ;
}

/*
 * Queue a remote command from the Wi-Fi task. Never blocks.
 * ESTOP goes to its own slot. A full ESTOP slot already holds an ESTOP,
 * so it counts as accepted but is counted as lost.
 */
int robot_command_push(const robot_command_t *command)
{
    if (!command || command->opcode < RM_CMD_ESTOP ||
        command->opcode > RM_CMD_START) {
        return E_PAR;
    }
    const ID queue =
        command->opcode == RM_CMD_ESTOP ? estop_queue : command_queue;
    const ER er =
        queue > 0 ? tk_snd_mbf(queue, command, (INT)sizeof(*command), TMO_POL)
                  : E_OBJ;
    if (er < E_OK) {
        lost(&command_lost);
    }
    /* A full dedicated ESTOP slot already contains an ESTOP. Coalesce, count. */
    return command->opcode == RM_CMD_ESTOP && er == E_TMOUT ? E_OK : (int)er;
}

/*
 * Take the next command. ESTOP is always read first.
 * Returns the message size (> 0), E_TMOUT if both are empty, or an error.
 */
int robot_command_pop(robot_command_t *out)
{
    if (!out) {
        return E_PAR;
    }
    if (estop_queue <= 0 || command_queue <= 0) {
        return E_OBJ;
    }
    const INT stop = tk_rcv_mbf(estop_queue, out, TMO_POL);
    return stop > 0 ? (int)stop : (int)tk_rcv_mbf(command_queue, out, TMO_POL);
}

/*
 * Count a board read that found the board busy.
 * Boards outside ROBOT_BOARD_COUNT are ignored.
 */
void robot_network_busy(unsigned int board)
{
    if (board < (unsigned int)ROBOT_BOARD_COUNT) {
        lost(&busy[board]);
    }
}

/* Classify a main-board read: absence is not writer contention. */
void robot_network_board_result(unsigned int board, int result)
{
    if (board >= (unsigned int)ROBOT_BOARD_COUNT) {
        return;
    }
    const uint8_t mask = (uint8_t)(1u << board);

    (void)SpinLock(&queue_lock);
    if (result >= E_OK) {
        unavailable_mask = (uint8_t)(unavailable_mask & (uint8_t)~mask);
    } else if (result == E_OBJ || result == E_NOEXS) {
        ++unavailable;
        unavailable_mask = (uint8_t)(unavailable_mask | mask);
    } else if (result == E_TMOUT) {
        ++busy[board];
    }
    (void)SpinUnlock(&queue_lock);
}

/* Count a command dropped before it reached the queue. */
void robot_command_lost(void)
{
    lost(&command_lost);
}

/* Copy the queue loss and busy counters into a heartbeat. */
void robot_queue_metrics(rm_heartbeat_t *out)
{
    (void)SpinLock(&queue_lock);
    out->event_lost = event_lost;
    out->command_lost = command_lost;
    out->line_busy = busy[0];
    out->sonar_busy = busy[1];
    out->motion_busy = busy[2];
    out->vehicle_busy = busy[3];
    out->health_busy = busy[4];
    out->board_unavailable = unavailable;
    out->board_unavailable_mask = unavailable_mask;
    (void)SpinUnlock(&queue_lock);
}

/* Default metrics provider. The MQTT module overrides it. */
void __attribute__((weak)) robot_network_metrics(rm_heartbeat_t *out)
{
    robot_queue_metrics(out);
}
