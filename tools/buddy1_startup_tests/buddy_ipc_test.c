/* Production Buddy 1 command/telemetry code; only kernel IPC is simulated. */
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "firmware_buddy.c"

static struct {unsigned count, size; unsigned char data[8][32];} queues[4];
static unsigned next_queue;
static jmp_buf task_idle;
ID tk_cre_mbf(const T_CMBF *definition) {
    assert(++next_queue<4 && definition->maxmsz<=32);
    queues[next_queue].size=definition->maxmsz; return next_queue;
}
ER tk_snd_mbf(ID id, const void *message, INT size, INT timeout) {
    assert(id>0 && id<4 && timeout==TMO_POL);
    assert((unsigned)size==queues[id].size);
    if(queues[id].count==8) return E_TMOUT;
    memcpy(queues[id].data[queues[id].count++],message,size); return E_OK;
}
INT tk_rcv_mbf(ID id, void *message, INT timeout) {
    assert(id>0 && id<4);
    if(!queues[id].count) {
        if(timeout==TMO_FEVR) longjmp(task_idle,1);
        assert(timeout==TMO_POL); return E_TMOUT;
    }
    memcpy(message,queues[id].data[0],queues[id].size);
    queues[id].count--;
    memmove(queues[id].data[0],queues[id].data[1],queues[id].count*32);
    return queues[id].size;
}
ID tk_cre_tsk(const T_CTSK *definition) { (void)definition; return 1; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
void test_buddy_init(void) {
    assert(buddy1_command_init()==E_OK);
    assert(buddy1_telemetry_init()==E_OK);
}
void test_queue_telemetry(void) {
    telemetry_message_t message={.source=1,.type=1,.sequence=42,.value_1=7,.value_2=-9};
    assert(buddy1_telemetry_submit(&message)==E_OK);
    if(setjmp(task_idle)==0) telemetry_task(0,NULL);
    assert(queues[mqtt_outbox_id].count==1);
}
void test_consume_commands(void) {
    assert(queues[command_buffer_id].count==2);
    for(unsigned i=0;i<2;i++) {
        command_message_t message;
        memcpy(&message,queues[command_buffer_id].data[i],sizeof(message));
        assert(message.type==(i ? COMMAND_TYPE_STOP : COMMAND_TYPE_START));
        assert(message.sequence==i+1 && message.source==1 && message.value==0);
    }
    if(setjmp(task_idle)==0) command_task(0,NULL);
    assert(queues[command_buffer_id].count==0);
}
