#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem1/robot_network.h"
/* Integration-private functions are audited without changing public headers. */
int robot_event_pop(rm_event_t *out);
void robot_command_lost(void);
void robot_queue_metrics(rm_heartbeat_t *out);
typedef struct {
    unsigned char data[16][sizeof(rm_event_t)+sizeof(robot_command_t)];
    unsigned int count,head,capacity;INT size;bool live;
} queue_t;
static queue_t queues[3];
static unsigned int created,deleted,irq_off,di_count,ei_count,sends,receives;
static unsigned int fail_create;
void mock_di(UINT *irq) { assert(!irq_off);irq_off=1u;*irq=42u;++di_count; }
void mock_ei(UINT irq) { assert(irq==42u && irq_off);irq_off=0u;++ei_count; }
ID tk_cre_mbf(const T_CMBF *value) {
    const unsigned int slot=created%3u;assert(!irq_off && value && value->mbfatr==TA_TFIFO);
    const INT size=slot==0u ? (INT)sizeof(rm_event_t) : (INT)sizeof(robot_command_t);
    const unsigned int capacity=slot==0u ? 16u : (slot==1u ? 8u : 1u);
    assert(value->maxmsz==size && value->bufsz==(INT)capacity*(size+4));
    ++created;
    if (created==fail_create) return E_SYS;
    queues[slot]=(queue_t){.capacity=capacity,.size=size,.live=true};return (ID)(slot+1u);
}
ER tk_del_mbf(ID id) {
    assert(!irq_off && id>0 && id<=3 && queues[(unsigned int)id-1u].live);
    queues[(unsigned int)id-1u].live=false;++deleted;return E_OK;
}
ER tk_snd_mbf(ID id,const void *value,INT size,TMO timeout) {
    assert(!irq_off && id>0 && id<=3 && timeout==TMO_POL);++sends;
    queue_t *q=&queues[(unsigned int)id-1u];assert(q->live && size==q->size);
    if (q->count==q->capacity) return E_TMOUT;
    memcpy(q->data[(q->head+q->count)%q->capacity],value,(size_t)size);++q->count;return E_OK;
}
INT tk_rcv_mbf(ID id,void *value,TMO timeout) {
    assert(!irq_off && id>0 && id<=3 && timeout==TMO_POL);++receives;
    queue_t *q=&queues[(unsigned int)id-1u];assert(q->live);
    if (q->count==0u) return E_TMOUT;
    memcpy(value,q->data[q->head],(size_t)q->size);q->head=(q->head+1u)%q->capacity;--q->count;return q->size;
}
int main(int argc,char **argv) {
    rm_event_t event={.kind=1u,.sequence=123u},event_out;
    robot_command_t command={.opcode=RM_CMD_START,.request_id=99u},command_out;
    rm_heartbeat_t metrics={.publish_ok=123u};
    assert(robot_event_push(NULL)==E_PAR && robot_event_pop(NULL)==E_PAR);
    assert(robot_command_push(NULL)==E_PAR && robot_command_pop(NULL)==E_PAR);
    assert(robot_event_push(&event)==E_OBJ && robot_event_pop(&event_out)==E_OBJ);
    assert(robot_command_push(&command)==E_OBJ && robot_command_pop(&command_out)==E_OBJ);
    assert(sends==0u && receives==0u);
    robot_network_metrics(&metrics);
    assert(metrics.event_lost==1u && metrics.command_lost==1u);
    if (argc>1) {
        fail_create=(unsigned int)strtoul(argv[1],NULL,10);
        assert(fail_create>=1u && fail_create<=3u);
        assert(robot_network_init()==E_SYS && deleted==fail_create-1u);
        for (unsigned int i=0u;i<3u;++i) assert(!queues[i].live);
        assert(robot_event_push(&event)==E_OBJ && robot_command_push(&command)==E_OBJ);
        /* Successful retry must recreate all resources after each failure. */
        created=0u;fail_create=0u;
    }
    assert(robot_network_init()==E_OK && created==3u);
    assert(robot_network_init()==E_OK && created==3u);
    robot_network_metrics(&metrics);
    const uint32_t event_loss_base=metrics.event_lost,command_loss_base=metrics.command_lost;
    for (unsigned int i=0u;i<16u;++i) { event.sequence=i;assert(robot_event_push(&event)==E_OK); }
    assert(robot_event_push(&event)==E_TMOUT);
    for (unsigned int i=0u;i<8u;++i) { command.request_id=i;assert(robot_command_push(&command)==E_OK); }
    assert(robot_command_push(&command)==E_TMOUT);
    robot_command_t stop={.opcode=RM_CMD_ESTOP,.request_id=100u};
    assert(robot_command_push(&stop)==E_OK);stop.request_id=200u;
    assert(robot_command_push(&stop)==E_OK); /* Full ESTOP slot coalesces, counts loss. */
    assert(robot_command_pop(&command_out)==(INT)sizeof(command_out));
    assert(command_out.opcode==RM_CMD_ESTOP && command_out.request_id==100u); /* Priority over START. */
    for (unsigned int i=0u;i<8u;++i) {
        assert(robot_command_pop(&command_out)==(INT)sizeof(command_out));
        assert(command_out.opcode==RM_CMD_START && command_out.request_id==i);
    }
    assert(robot_command_pop(&command_out)==E_TMOUT);
    for (uint8_t opcode=RM_CMD_DUMP;opcode<=RM_CMD_START;++opcode) {
        command.opcode=opcode;command.request_id=400u+opcode;
        assert(robot_command_push(&command)==E_OK);
        assert(robot_command_pop(&command_out)==(INT)sizeof(command_out));
        assert(command_out.opcode==opcode && command_out.request_id==400u+opcode);
    }
    for (unsigned int i=0u;i<16u;++i) {
        assert(robot_event_pop(&event_out)==(INT)sizeof(event_out) && event_out.sequence==i);
    }
    assert(robot_event_pop(&event_out)==E_TMOUT);
    const unsigned int before=sends;
    command.opcode=0u;assert(robot_command_push(&command)==E_PAR);
    command.opcode=5u;assert(robot_command_push(&command)==E_PAR && sends==before);
    robot_command_lost();
    for (unsigned int i=0u;i<5u;++i) for (unsigned int j=0u;j<=i;++j) robot_network_busy(i);
    robot_network_busy(5u);robot_network_busy(UINT32_MAX);
    robot_queue_metrics(&metrics);
    assert(metrics.event_lost==event_loss_base+1u && metrics.command_lost==command_loss_base+3u);
    assert(metrics.line_busy==1u && metrics.sonar_busy==2u && metrics.motion_busy==3u && metrics.vehicle_busy==4u && metrics.health_busy==5u);
    assert(metrics.publish_ok==123u);
    /* TEST has no vehicle role: absence must not inflate contention. */
    for (unsigned int i=0u;i<233u;++i) robot_network_board_result(ROBOT_BOARD_VEHICLE,E_OBJ);
    robot_queue_metrics(&metrics);
    assert(metrics.vehicle_busy==4u && metrics.board_unavailable==233u);
    assert(metrics.board_unavailable_mask==(1u<<ROBOT_BOARD_VEHICLE));
    robot_network_board_result(ROBOT_BOARD_VEHICLE,E_TMOUT);
    robot_queue_metrics(&metrics);
    assert(metrics.vehicle_busy==5u && metrics.board_unavailable==233u);
    assert(metrics.board_unavailable_mask==(1u<<ROBOT_BOARD_VEHICLE));
    robot_network_board_result(ROBOT_BOARD_VEHICLE,E_OK);
    robot_network_board_result(ROBOT_BOARD_COUNT,E_OBJ);
    robot_queue_metrics(&metrics);
    assert(metrics.board_unavailable_mask==0u && metrics.board_unavailable==233u);
    assert(irq_off==0u && di_count==ei_count);
    return 0;
}
