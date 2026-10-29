#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "firmware_usb.c"

volatile UW tm_usb_irq_count, tm_usb_nvic_enabled;
volatile W tm_usb_def_int_ercd, tm_usb_service_tskid;
static int mounted, init_ok=1, sleeps, services, pending_events, starts, exits;
static FP task;
static jmp_buf stopped;
int tm_tusb_init(void) { return init_ok; }
void tm_tusb_service(void) { services++; if(pending_events) pending_events--; }
int tm_tusb_event_ready(void) { return pending_events!=0; }
int tm_tusb_mounted(void) { return mounted; }
int tm_tusb_connected(void) { return 0; }
unsigned tm_tusb_write_available(void) { return 0; }
void tm_tusb_write_char(char c) { (void)c; assert(!"unexpected write"); }
void tm_tusb_write_flush(void) {}
void tm_tusb_discard_input(void) {}
UW tm_tusb_diag_controller_en(void) { return 1; }
UW tm_tusb_diag_pullup_en(void) { return 1; }
UW tm_tusb_diag_bus_seen(void) { return 0; }
int tk_get_tid(void) { return 7; }
void tk_ext_tsk(void) { exits++; }
int tk_slp_tsk(int timeout) {
    assert(timeout==10);
    if(++sleeps==3) longjmp(stopped,1);
    return 0;
}
int tk_cre_tsk(T_CTSK *definition) {
    assert(definition->itskpri==3);
    task=definition->task; return 7;
}
int tk_sta_tsk(int id, int code) { assert(id==7 && code==0); starts++; return 0; }
int tk_del_tsk(int id) { (void)id; return 0; }

int main(int argc, char **argv) {
    assert(argc==2);
    mounted=!strcmp(argv[1],"mounted_blocked");
    init_ok=strcmp(argv[1],"init_failure")!=0;
    pending_events=!strcmp(argv[1],"event_burst") ? 100 : 0;
    assert(tm_usb_console_start()==0 && starts==1 && task!=NULL);
    assert(tm_usb_console_start()==0 && starts==1);
    UB output[10000]; memset(output,'x',sizeof(output));
    tm_usb_console_put(output,sizeof(output));
    assert(sleeps==0 && tm_usb_pending()==4095 && tm_usb_dropped==5905);
    if(setjmp(stopped)==0) task(0,NULL);
    if(init_ok) {
        assert(sleeps==3 && exits==0 && tm_usb_service_tskid==7);
        assert(tm_usb_pending()==4095 && tm_usb_dropped==5905);
        assert(tm_usb_state()==(mounted ? 2u : 1u));
        assert(services==(!strcmp(argv[1],"event_burst") ? 102 : 3));
    } else {
        assert(exits==1 && sleeps==0 && tm_usb_service_tskid==0);
    }
    printf("PASS %-16s console returns/sleeps; bounded ring drops %u bytes\n",
           argv[1],(unsigned)tm_usb_dropped);
    return 0;
}
