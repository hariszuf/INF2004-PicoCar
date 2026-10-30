#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "line_test.h"
#include "platform.h"
#include "range_policy.h"
#include "buddy5_servo.h"
#include "../buddy5/ultrasonic.h"
#include "../buddy1/telemetry.h"
static b5_core_t core;
static line_scan_status_t shared;
static bool reset_pending, abort_pending;
static uint32_t report_sequence;
static void publish(uint32_t now,int angle,bool valid,uint32_t timestamp) {
 UW irq;
 line_scan_status_t s={core.state,core.fault,core.profile,core.front_clearance_mm,
                       angle,valid,timestamp,now};
 DI(irq); shared=s; EI(irq);
}
void line_scan_get_status(line_scan_status_t *s) { UW irq; DI(irq); *s=shared; EI(irq); }
void line_scan_reset(void) { UW irq; DI(irq); reset_pending=true; EI(irq); }
void line_scan_abort(void) { UW irq; DI(irq); abort_pending=true; EI(irq); }
static void emit(uint8_t type,int32_t a,int32_t b,uint32_t seq) {
 telemetry_message_t m={0};
 m.source=5; m.type=type; m.value_1=a; m.value_2=b; m.sequence=seq;
 m.flags=(core.profile.width_valid?1u:0u)|(core.profile.depth_valid?2u:0u)|16u|
         (core.front_clearance_mm==B5_CLEAR_FRONT_MM?32u:0u); /* provisional full test */
 (void)buddy1_telemetry_submit(&m);
}
static void scan_task(INT stacd,void *exinf) {
 uint32_t settled=0,last_ping=0,last_report=0,delivered=0,front_time=0;
 b5_state_t previous=(b5_state_t)99;
 b5_range_t readings[B5_SAMPLE_COUNT];
 unsigned count=0;
 int32_t close_candidate_mm=-1;
 unsigned close_batches=0;
 int angle=999;
 bool front_valid=false;
 (void)stacd; (void)exinf;
 b5_core_init(&core,buddy5_clock_ms());
 for(;;) {
  uint32_t now=buddy5_clock_ms();
  b5_feedback_t feedback={0};
  UW irq;
  bool reset,abort;
  (void)line_controller_feedback(&feedback);
  DI(irq); reset=reset_pending; reset_pending=false; abort=abort_pending; abort_pending=false; EI(irq);
  if(reset && feedback.valid && feedback.stopped && !feedback.fault &&
     (uint32_t)(now-feedback.timestamp_ms)<=B5_FEEDBACK_MAX_AGE_MS) {
   uint32_t seq=core.next_request_id,id=core.profile.obstacle_id;
   b5_core_init(&core,now); core.next_request_id=seq; core.profile.obstacle_id=id;
   count=0; angle=999; front_valid=false; close_candidate_mm=-1; close_batches=0;
  }
  if(abort && !reset) b5_core_fail(&core,B5_FAULT_CONTROLLER,now);
  b5_core_update(&core,now,&feedback,NULL);
  bool positioning_mode=core.state==B5_PROCESSING;
  bool guard_mode=core.state==B5_LINE_RECOVERY ||
                  (core.state==B5_AVOIDING && core.phase==2);
  int wanted=b5_core_angle(&core);
  if(wanted!=angle) {
   count=0; front_valid=false;
   if(!buddy5_servo_set_angle(wanted)) b5_core_fail(&core,B5_FAULT_SERVO,now);
   else { angle=wanted; settled=now; }
  }
  if((b5_core_wants_range(&core) || guard_mode) &&
     (uint32_t)(now-settled)>=B5_SERVO_SETTLE_MS &&
     (uint32_t)(now-last_ping)>=B5_PING_INTERVAL_MS) {
   ultrasonic_result_t raw=ultrasonic_read();
   now=last_ping=buddy5_clock_ms();
   b5_range_t reading=b5_range_from_raw(raw);
   bool no_echo=b5_range_no_echo(&reading);
   readings[count++]=reading;
   bool forward=angle==0;
   /* While normally driving, a single HC-SR04 echo must not stop the car:
    * require the existing three-sample filter.  Guarding a movement remains
    * immediate, because that happens after an obstacle has been confirmed. */
   bool urgent=forward && !no_echo && guard_mode && (!reading.valid ||
       reading.distance_mm<=B5_GUARD_FRONT_MM+B5_FRONT_OVERHANG_MM);
   if(count==B5_SAMPLE_COUNT || urgent || (!no_echo && guard_mode)) {
    /* Motion changes acquisition pose: guard each new echo directly. */
    b5_range_t filtered=(urgent || (!no_echo && guard_mode))?reading:b5_filter(readings);
    (void)line_controller_feedback(&feedback);
    if(forward) { front_valid=filtered.valid; front_time=now; }
    if(core.state==B5_SCANNING)
     tm_printf((UB*)"[SCAN] angle=%d echo=%d mm=%d\n",angle,filtered.valid && !b5_range_no_echo(&filtered),b5_range_no_echo(&filtered)?-1:(int)filtered.distance_mm);
    if(core.state==B5_MONITORING && !line_controller_active()) {
     /* Standby gathers front data without starting an obstacle scan. */
     core.front_clearance_mm=b5_front_clearance(&filtered);
    } else if(positioning_mode) {
     /* The initial turn is allowed with the obstacle ahead.  Keep updating
      * clearance so the following lateral translation starts only when clear. */
     core.front_clearance_mm=b5_front_clearance(&filtered);
    } else if(guard_mode) {
     /* One malformed HC-SR04 batch must not latch the whole run off. The
      * controller keeps the last clear observation for 600 ms; a confirmed
      * close return still stops immediately. */
     if(filtered.valid) {
      core.front_clearance_mm=b5_front_clearance(&filtered);
      if(core.front_clearance_mm<=B5_GUARD_FRONT_MM)
       b5_core_fail(&core,B5_FAULT_NO_ROUTE,now);
     }
    } else {
     b5_range_t *confirmed=&filtered;
     if(core.state==B5_MONITORING) {
      /* A malformed/noisy pulse is neither a clear path nor an obstacle.
       * Retry it on the next batch; do not latch the driving test off. */
      if(!filtered.valid && !b5_range_no_echo(&filtered)) {
       confirmed=NULL; close_candidate_mm=-1; close_batches=0;
      } else {
      int32_t clearance=b5_front_clearance(&filtered);
      if(filtered.valid && !b5_range_no_echo(&filtered) && clearance<=B5_STOP_FRONT_MM) {
       int32_t difference=clearance-close_candidate_mm;
       if(difference<0) difference=-difference;
       if(close_candidate_mm>=0 && difference<=60) ++close_batches;
       else { close_candidate_mm=clearance; close_batches=1; }
       /* Two matching 3-ping groups reject transient phantom echoes. */
       if(close_batches<2) confirmed=NULL;
      } else { close_candidate_mm=-1; close_batches=0; }
      }
     } else { close_candidate_mm=-1; close_batches=0; }
     b5_core_update(&core,now,&feedback,confirmed);
    }
    count=0;
   }
  }
  if(core.request.id && core.request.id!=delivered) {
   delivered=core.request.id;
   if(!line_controller_request(&core.request)) {
    if(core.state==B5_ERROR) delivered=0;
    else b5_core_fail(&core,B5_FAULT_CONTROLLER,now);
   }
  }
  if(core.state!=previous) {
   tm_printf((UB*)"[SCAN] state=%s fault=%d width=%d side=%d offset=%d\n",
      (UB*)b5_state_name(core.state),core.fault,
      core.profile.width_valid?core.profile.width_mm:-1,core.profile.side,core.profile.offset_mm);
   previous=core.state;
  }
  if((uint32_t)(now-last_report)>=1000u) {
   uint32_t seq=++report_sequence;
   emit(50,core.state,core.fault,seq);
   emit(51,core.profile.width_valid?core.profile.width_mm:-1,core.profile.depth_valid?core.profile.depth_high_mm:-1,seq);
   emit(52,core.profile.side,core.profile.offset_mm,seq);
   emit(54,core.front_clearance_mm==B5_CLEAR_FRONT_MM?-1:core.front_clearance_mm,angle,seq);
   emit(53,core.profile.depth_valid?core.profile.depth_low_mm:-1,core.profile.depth_valid?core.profile.depth_high_mm:-1,seq);
   emit(56,(core.profile.width_valid?1:0)|(core.profile.depth_valid?2:0)|16,0,seq);
   last_report=now;
  }
  publish(now,angle,front_valid,front_time);
  tk_dly_tsk(5);
 }
}
int32_t line_scan_start(void) {
 T_CTSK cfg={0};
 cfg.tskatr=TA_HLNG|TA_RNG3; cfg.task=scan_task; cfg.itskpri=11; cfg.stksz=4096;
 ID id=tk_cre_tsk(&cfg);
 if(id<0) return id;
 ER e=tk_sta_tsk(id,0);
 if(e<0) tk_del_tsk(id);
 return e;
}
#endif
