/* Host-only kernel surface. timer.c and wait.c below are the real trial/live
 * sources; these types/ready operations replace only scheduler/CPU machinery. */
#ifndef BUDDY4_HOST_KERNEL_H
#define BUDDY4_HOST_KERNEL_H
#define _WAIT_
#define _TIMER_
#define _SYSDEPEND_SYSTIMER_
#define IMPORT extern
#define CONST const
#define Noinit(x) x
#define USE_SHUTDOWN 0
#define USE_DBGSPT 0
#define TIMER_PERIOD 10
#define ABSTIM_DIFF_MIN 0x7fffffffu
#define TA_TPRI 1
#define TMO_FEVR (-1)
#define TMO_POL 0
#define E_DLT (-51)
typedef unsigned int ATR, RELTIM;
typedef int64_t LSYSTIM;
typedef UW ABSTIM;
typedef struct queue { struct queue *next, *prev; } QUEUE;
static inline void QueInit(QUEUE *q) { q->next = q->prev = q; }
static inline int isQueEmpty(QUEUE *q) { return q->next == q; }
static inline void QueInsert(QUEUE *p, QUEUE *q) {
    p->prev = q->prev; p->next = q; q->prev->next = p; q->prev = p;
}
static inline void QueRemove(QUEUE *p) {
    if (p->next != p) { p->prev->next = p->next; p->next->prev = p->prev; }
}
typedef void (*CBACK)(void *);
typedef struct { QUEUE queue; ABSTIM time; CBACK callback; void *arg; } TMEB;
typedef struct task_control_block TCB;
typedef struct { UW tskwait; void (*chg_pri_hook)(TCB *, INT); void (*rel_wai_hook)(TCB *); } WSPEC;
struct task_control_block {
    QUEUE tskque;
    ID tskid, wid;
    INT state, priority;
    TMEB wtmeb;
    const WSPEC *wspec;
    ER *wercd;
};
typedef struct { QUEUE wait_queue; ID objid; void *exinf; ATR objatr; } GCB;
enum { TS_READY = 1, TS_WAIT = 2, TS_SUSPEND = 3, TS_WAITSUS = 4 };
extern TCB *knl_ctxtsk;
extern LSYSTIM knl_current_time;
extern QUEUE knl_timer_queue;
#define lltoul(x) ((UW)(x))
#define uitoll(x) ((LSYSTIM)(x))
#define ll_add(x,y) ((x)+(y))
static inline int knl_abstim_reached(ABSTIM a, ABSTIM b) { return a-b <= ABSTIM_DIFF_MIN; }
static inline void knl_make_ready(TCB *t) { t->state = TS_READY; }
static inline void knl_make_non_ready(TCB *t) { (void)t; }
static inline void knl_make_non_wait(TCB *t) {
    if (t->state == TS_WAIT) knl_make_ready(t); else t->state = TS_SUSPEND;
}
static inline void knl_wait_release(TCB *t) {
    QueRemove(&t->wtmeb.queue); QueRemove(&t->tskque); knl_make_non_wait(t);
}
static inline void knl_queue_insert_tpri(TCB *t, QUEUE *q) { QueInsert(&t->tskque, q); }
static inline void knl_start_hw_timer(void) {}
static inline void knl_clear_hw_timer_interrupt(void) {}
static inline void knl_end_of_hw_timer_interrupt(void) {}
#define BEGIN_CRITICAL_SECTION { UW saved_mask = mock_disint();
#define END_CRITICAL_SECTION \
    if (!saved_mask && knl_ctxtsk && knl_ctxtsk->state == TS_READY) dispatch_pending = 1; \
    mock_restore_interrupts(saved_mask); }
#endif
