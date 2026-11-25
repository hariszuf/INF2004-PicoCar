#include "../../app_program/barrc_types.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "firmware_usb.c"

volatile UW tm_usb_irq_count, tm_usb_nvic_enabled;
volatile W tm_usb_def_int_ercd, tm_usb_service_tskid;
static int32_t s_mounted;
static int32_t s_init_ok = 1;
static int32_t s_sleeps;
static int32_t s_services;
static int32_t s_pending_events;
static int32_t s_starts;
static int32_t s_exits;
static FP task;
static jmp_buf s_stopped;
int32_t tm_tusb_init (void)
{
    return s_init_ok;
}

void tm_tusb_service (void)
{
    s_services++;

    if (s_pending_events)
    {
        s_pending_events--;
    }
}

int32_t tm_tusb_event_ready (void)
{
    return s_pending_events != 0;
}

int32_t tm_tusb_mounted (void)
{
    return s_mounted;
}

int32_t tm_tusb_connected (void)
{
    return 0;
}

uint32_t tm_tusb_write_available (void)
{
    return 0;
}

void tm_tusb_write_char (char c)
{
    (void)c;
    assert(!"unexpected write");
}

void tm_tusb_write_flush (void)
{
}

void tm_tusb_discard_input (void)
{
}

UW tm_tusb_diag_controller_en (void)
{
    return 1;
}

UW tm_tusb_diag_pullup_en (void)
{
    return 1;
}

UW tm_tusb_diag_bus_seen (void)
{
    return 0;
}

int32_t tk_get_tid (void)
{
    return 7;
}

void tk_ext_tsk (void)
{
    s_exits++;
}

int32_t tk_slp_tsk (int32_t timeout)
{
    assert(10 == timeout);

    if (++s_sleeps == 3)
    {
        longjmp(s_stopped, 1);
    }

    return 0;
}

int32_t tk_cre_tsk (T_CTSK *definition)
{
    assert(3 == definition->itskpri);
    task = definition->task;
    return 7;
}

int32_t tk_sta_tsk (int32_t id, int32_t code)
{
    assert((7 == id) && (0 == code));
    s_starts++;
    return 0;
}

int32_t tk_del_tsk (int32_t id)
{
    (void)id;
    return 0;
}

int main (int argc, char ** argv)
{
    assert(2 == argc);
    s_mounted = !strcmp(argv[1], "mounted_blocked");
    s_init_ok = strcmp(argv[1], "init_failure") != 0;
    s_pending_events = !strcmp(argv[1], "event_burst") ? 100 : 0;
    assert((0 == tm_usb_console_start()) && (1 == s_starts) && (task != NULL));
    assert((0 == tm_usb_console_start()) && (1 == s_starts));
    UB output[10000];
    memset(output, 'x', sizeof(output));
    tm_usb_console_put(output, sizeof(output));
    assert((0 == s_sleeps) && (4095 == tm_usb_pending())
           && (5905 == tm_usb_dropped));

    if (0 == setjmp(s_stopped))
    {
        task(0, NULL);
    }

    if (s_init_ok)
    {
        assert((3 == s_sleeps) && (0 == s_exits)
               && (7 == tm_usb_service_tskid));
        assert((4095 == tm_usb_pending()) && (5905 == tm_usb_dropped));
        assert(tm_usb_state() == (s_mounted ? 2u : 1u));
        assert(s_services == (!strcmp(argv[1], "event_burst") ? 102 : 3));
    }
    else
    {
        assert((1 == s_exits) && (0 == s_sleeps)
               && (0 == tm_usb_service_tskid));
    }

    printf("PASS %-16s console returns/sleeps; bounded ring drops %u bytes\n",
           argv[1], (uint32_t)tm_usb_dropped);
    return 0;
}

/*** end of file ***/
