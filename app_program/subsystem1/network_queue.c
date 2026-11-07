#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "robot_network.h"
static ID event_queue,command_queue,estop_queue;
static uint32_t event_lost,command_lost,busy[5];
static void lost(uint32_t *counter) { UINT irq;DI(irq);++*counter;EI(irq); }
int robot_network_init(void) {
    if (event_queue>0 && command_queue>0 && estop_queue>0) return E_OK;
    T_CMBF q={.mbfatr=TA_TFIFO,.bufsz=16*(INT)(sizeof(rm_event_t)+4u),.maxmsz=(INT)sizeof(rm_event_t)};
    event_queue=tk_cre_mbf(&q);
    if (event_queue<0) return (int)event_queue;
    q.bufsz=8*(INT)(sizeof(robot_command_t)+4u);q.maxmsz=(INT)sizeof(robot_command_t);
    command_queue=tk_cre_mbf(&q);
    if (command_queue<0) { (void)tk_del_mbf(event_queue);event_queue=0;return (int)command_queue; }
    q.bufsz=(INT)sizeof(robot_command_t)+4;
    estop_queue=tk_cre_mbf(&q);
    if (estop_queue<0) { (void)tk_del_mbf(event_queue);(void)tk_del_mbf(command_queue);event_queue=0;command_queue=0;return (int)estop_queue; }
    return E_OK;
}
int robot_event_push(const rm_event_t *event) {
    if (!event) return E_PAR;
    const ER er=event_queue>0 ? tk_snd_mbf(event_queue,event,(INT)sizeof(*event),TMO_POL) : E_OBJ;
    if (er<E_OK) lost(&event_lost);
    return (int)er;
}
int robot_event_pop(rm_event_t *out) {
    if (!out) return E_PAR;
    return event_queue>0 ? (int)tk_rcv_mbf(event_queue,out,TMO_POL) : (int)E_OBJ;
}
int robot_command_push(const robot_command_t *command) {
    if (!command || command->opcode<RM_CMD_ESTOP || command->opcode>RM_CMD_START) return E_PAR;
    const ID queue=command->opcode==RM_CMD_ESTOP ? estop_queue : command_queue;
    const ER er=queue>0 ? tk_snd_mbf(queue,command,(INT)sizeof(*command),TMO_POL) : E_OBJ;
    if (er<E_OK) lost(&command_lost);
    /* A full dedicated ESTOP slot already contains an ESTOP. Coalesce, count. */
    return command->opcode==RM_CMD_ESTOP && er==E_TMOUT ? E_OK : (int)er;
}
int robot_command_pop(robot_command_t *out) {
    if (!out) return E_PAR;
    if (estop_queue<=0 || command_queue<=0) return E_OBJ;
    const INT stop=tk_rcv_mbf(estop_queue,out,TMO_POL);
    return stop>0 ? (int)stop : (int)tk_rcv_mbf(command_queue,out,TMO_POL);
}
void robot_network_busy(unsigned int board) { if (board<5u) lost(&busy[board]); }
void robot_command_lost(void) { lost(&command_lost); }
void robot_queue_metrics(rm_heartbeat_t *out) {
    UINT irq;DI(irq);
    out->event_lost=event_lost;out->command_lost=command_lost;
    out->line_busy=busy[0];out->sonar_busy=busy[1];out->motion_busy=busy[2];
    out->vehicle_busy=busy[3];out->health_busy=busy[4];
    EI(irq);
}
void __attribute__((weak)) robot_network_metrics(rm_heartbeat_t *out) { robot_queue_metrics(out); }
