#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "platform.h"
#include "scan_link.h"
#include "vehicle_nav.h"
#include "auto_start.h"
#include "../../buddy1/buddy1.h"
#include "buddy5_servo.h"
#include "../../buddy5/ultrasonic.h"
#include "../../buddy2/motion_control.h"
#include "../../buddy3/line_follow.h"
#include "../../buddy3/line_sensors.h"
#include "../../buddy3/line_config.h"
#ifndef B5_HEADLESS
#define B5_HEADLESS 0
#endif
#ifndef B5_TIGHT_COURSE
#define B5_TIGHT_COURSE 0
#endif
#if TK_SUPPORT_SMP
#error "This integration profile requires SMP=0"
#endif
#if CNF_TIMER_PERIOD != 1
#error "Use picocar_integration.mk (1ms kernel tick required)"
#endif
#if TM_CONSOLE_UART
#error "GP0 belongs to the line sensor: this profile requires USB-only console"
#endif
#if TM_CONSOLE_USB_CDC
IMPORT UW tm_usb_state(void);
#endif
static b5_feedback_t feedback_shared;
static b5_request_t pending_request;
static bool request_pending;
static volatile bool run_enabled;
static volatile bool user_stop_pending;
static unsigned char commands[8];
static unsigned command_head,command_tail;
bool line_controller_active(void) { return run_enabled; }
bool b5_line_usb_connected(void) {
#if TM_CONSOLE_USB_CDC
 return tm_usb_state()>=3;
#else
 return false;
#endif
}
void b5_line_command_received(unsigned c) {
 UW irq;
 if(c!='R' && c!='S' && c!=' ') return;
 DI(irq);
 if(c=='S' || c==' ') {
  user_stop_pending=true; command_tail=command_head;
 } else if(b5_line_usb_connected()) {
  unsigned next=(command_head+1u)%8u;
  if(next==command_tail) { user_stop_pending=true; command_tail=command_head; }
  else { commands[command_head]=(unsigned char)c; command_head=next; }
 }
 EI(irq);
}
bool b5_line_take_stop(void) {
 UW irq; bool stop;
 DI(irq); stop=user_stop_pending; user_stop_pending=false; EI(irq);
 return stop;
}
int b5_line_take_command(void) {
 UW irq; int c=-1;
 DI(irq);
 if(command_head!=command_tail) {
  c=commands[command_tail]; command_tail=(command_tail+1u)%8u;
 }
 EI(irq); return c;
}
static void discard_commands(void) { UW irq; DI(irq); command_tail=command_head; EI(irq); }
bool line_controller_feedback(b5_feedback_t *f) {
 UW irq; DI(irq); *f=feedback_shared; EI(irq); return f->valid;
}
bool line_controller_request(const b5_request_t *r) {
 UW irq; bool accepted=false;
 DI(irq);
 if((run_enabled || r->kind==B5_REQ_STOP) && (!request_pending || r->kind==B5_REQ_STOP)) {
  pending_request=*r; request_pending=true; accepted=true;
 }
 EI(irq); return accepted;
}
static void halt(full_nav_t *nav,auto_start_t *start,const char *why) {
 run_enabled=false; auto_start_stop(start); nav->active=false; nav->moving=false; nav->forward_moving=false;
 line_follow_stop(); motion_emergency_stop(); line_scan_abort(); discard_commands();
 tm_printf((UB*)"[AUTO] STOPPED: %s. Power-cycle to restart; optional R retries.\n",(UB*)why);
}
static void controller_task(INT stacd,void *exinf) {
 motion_status_t motor; line_scan_status_t scan;
 full_nav_t nav; auto_start_t start;
 motion_update(); motion_get_status(&motor); full_nav_init(&nav,&motor);
 uint32_t now=buddy5_clock_ms(),last_motor=now,last_line=now,last_report=now;
 uint32_t run_since=0,lost_since=0,last_guard_clear=0;
 uint8_t mask=0; bool fault_latched=false;
 (void)stacd; (void)exinf;
 auto_start_init(&start,now);
 tm_printf((UB*)"[AUTO] Power-on automatic mode. 3s delay, then wait for line and forward observations.\n");
 tm_printf((UB*)"[AUTO] USB optional. Obstacle trigger=200mm from car front; S/space stops.\n");
 for(;;) {
  now=buddy5_clock_ms(); line_scan_get_status(&scan);
  int command=b5_line_take_stop()?'S':b5_line_take_command();
  if(command=='S') halt(&nav,&start,"user stop");
  if(command=='R' && !run_enabled && nav.settled>=3) {
   auto_start_init(&start,now-3000u); discard_commands();
  }
  if(run_enabled && ((uint32_t)(now-run_since)>300000u ||
     (uint32_t)(now-scan.heartbeat_ms)>400u)) halt(&nav,&start,"run/scan heartbeat timeout");
  if(run_enabled && scan.state==B5_ERROR) halt(&nav,&start,"obstacle subsystem fault (see SCAN)");
  if((uint32_t)(now-last_line)>=5u) {
   last_line=now; mask=line_sensors_read();
   if(run_enabled && scan.state==B5_MONITORING && !nav.active) {
    if(!(mask&(LINE_SENSOR_LEFT|LINE_SENSOR_RIGHT))) {
     if(!lost_since) lost_since=now;
     if((uint32_t)(now-lost_since)>700u) halt(&nav,&start,"line lost outside recovery");
    } else lost_since=0;
    if(run_enabled) line_follow_update();
   } else lost_since=0;
  }
  UW irq; bool got; b5_request_t request={0};
  DI(irq); got=request_pending; if(got) { request=pending_request; request_pending=false; } EI(irq);
  if(got && run_enabled) {
   if(!full_nav_accept(&nav,&request,now)) halt(&nav,&start,"request rejected");
   else tm_printf((UB*)"[NAV] request=%d id=%u target_x/y=%d/%d\n",request.kind,(unsigned)request.id,request.forward_mm,request.lateral_mm);
  }
  int32_t guard_limit=(nav.active && nav.request.kind==B5_REQ_BYPASS) ?
                      B5_BYPASS_GUARD_FRONT_MM : B5_GUARD_FRONT_MM;
  bool guard_fresh=scan.angle==0 && scan.front_valid &&
                   (uint32_t)(now-scan.front_timestamp_ms)<=400u;
  bool guard=false;
  if(guard_fresh) {
   /* A measured close return stops immediately. Only a missing/invalid batch
    * receives a short grace period. */
   guard=scan.front_clearance_mm>guard_limit;
   if(guard) last_guard_clear=now;
   else last_guard_clear=0;
  } else if(last_guard_clear &&
            (uint32_t)(now-last_guard_clear)<=
              ((B5_TIGHT_COURSE && nav.active && nav.request.kind==B5_REQ_BYPASS)?4000u:600u)) {
   guard=true;
  }
  /* The side-looking scan can leave an old clear reading behind. Wait for
   * the servo to face forward and acquire a new reading before ADVANCE. */
  if(nav.active && nav.request.kind==B5_REQ_ADVANCE && !nav.forward_moving &&
     !guard_fresh) guard=false;
  if((uint32_t)(now-last_motor)>=20u) {
   if((uint32_t)(now-last_motor)>60u && run_enabled) halt(&nav,&start,"motor scheduling delay");
   last_motor=now; motion_update(); motion_get_status(&motor);
   if(motor.fault==MOTION_FAULT_STALL) fault_latched=true;
   if(run_enabled && motor.fault!=MOTION_FAULT_NONE)
    halt(&nav,&start,"motor fault");
   if(run_enabled && nav.forward_moving && !guard)
    halt(&nav,&start,"forward range unavailable while moving");
   full_nav_update(&nav,now,&motor,mask,guard);
   if(run_enabled && nav.failed) halt(&nav,&start,"navigation failed/pose mismatch");
   if(nav.resumed) { tm_printf((UB*)"[AUTO] LINE REACQUIRED: line following resumed.\n"); nav.resumed=false; }
  }
  /* Start the proven Buddy 3 line follower after the power-on delay.
   * The scanner continues in parallel and will take control on an obstacle.
   * Do not make motor start depend on a first ultrasonic/IR sample: a missed
   * first sample otherwise leaves the car permanently stationary. */
  bool ready=motor.fault==MOTION_FAULT_NONE && !fault_latched;
#if B5_HEADLESS
  /* A fast power-bank boot can otherwise reach AUTO_START before the first
   * filtered centre ping, skip line_follow_start(), and never retry it. */
  ready=ready && scan.angle==0 && scan.front_valid &&
        (uint32_t)(now-scan.front_timestamp_ms)<=400u;
#endif
  auto_action_t action=auto_start_update(&start,now,nav.settled>=3,ready);
  if(action==AUTO_RESET) {
   line_follow_stop();
   if(motion_reset()!=MOTION_OK) halt(&nav,&start,"motor reset failed");
   else {
    motion_update(); motion_get_status(&motor); last_motor=now;
    full_nav_init(&nav,&motor); nav.settled=3; fault_latched=false; line_scan_reset();
    DI(irq); request_pending=false; EI(irq);
    tm_printf((UB*)"[AUTO] Waiting for line and fresh forward observations; no keyboard command needed.\n");
   }
  }
  if(action==AUTO_START) {
   run_enabled=true; run_since=now;
   /* If already 15..20cm from an obstacle, remain stationary and let the
    * first active front sample request STOP/SCAN. Never drive into it. */
   if(scan.front_clearance_mm>B5_STOP_FRONT_MM && !line_follow_start())
    halt(&nav,&start,"line follower could not start");
   else tm_printf((UB*)"[AUTO] RUNNING automatically; front=%dmm (-1=out of range).\n",scan.front_clearance_mm==B5_CLEAR_FRONT_MM?-1:scan.front_clearance_mm);
  }
  b5_feedback_t f;
  full_nav_feedback(&nav,last_motor,&f); f.fault=fault_latched;
  DI(irq); feedback_shared=f; EI(irq);
  if((uint32_t)(now-last_report)>=1000u) {
   line_status_t line; line_follow_get_status(&line);
   tm_printf((UB*)"[LINE] state=%d mask=%u target_mm_s=%d/%d actual_mm_s=%d/%d\n",
    line.state,(unsigned)mask,(int)(line.left_cmd_cm_s*10),(int)(line.right_cmd_cm_s*10),
    (int)(motor.left_speed_cm_s*10),(int)(motor.right_speed_cm_s*10));
   tm_printf((UB*)"[AUTO] phase=%d enabled=%d line_mask=%u scan=%s fault=%d nominal_servo=%d front=%d guard=%d fresh=%d x/y=%d/%d heading=%d width=%d depth=%d..%d\n",
    start.phase,run_enabled,(unsigned)mask,(UB*)b5_state_name(scan.state),scan.fault,scan.angle,scan.front_clearance_mm==B5_CLEAR_FRONT_MM?-1:scan.front_clearance_mm,
    guard,guard_fresh,
    (int)nav.x,(int)nav.y,(int)nav.heading,scan.profile.width_valid?scan.profile.width_mm:-1,
    scan.profile.depth_valid?scan.profile.depth_low_mm:-1,scan.profile.depth_valid?scan.profile.depth_high_mm:-1);
   last_report=now;
  }
  tk_dly_tsk(1);
 }
}
EXPORT INT usermain(void) {
 T_CTSK cfg={0};
 if(b5_platform_init()<E_OK || motion_init()!=MOTION_OK) return 1;
 motion_emergency_stop(); line_follow_init(); ultrasonic_init();
 if(!buddy5_servo_init() || buddy1_init()<E_OK) return 1;
 cfg.tskatr=TA_HLNG|TA_RNG3; cfg.task=controller_task; cfg.itskpri=4; cfg.stksz=4096;
 ID id=tk_cre_tsk(&cfg); if(id<0) return 1;
 if(tk_sta_tsk(id,0)<0) { tk_del_tsk(id); return 1; }
 if(buddy1_start()<E_OK || line_scan_start()<E_OK) { motion_emergency_stop(); return 1; }
 tk_slp_tsk(TMO_FEVR); return 0;
}
#endif
