#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "platform.h"
#include "barcode_task.h"
#include "scan_link.h"
#include "vehicle_nav.h"
#include "auto_start.h"
#include "../../buddy1/buddy1.h"
#include "../../buddy1/telemetry.h"
#include "../../buddy2/motion_encoder.h"
#include "buddy5_servo.h"
#include "../../buddy5/ultrasonic.h"
#include "../../buddy2/motion_control.h"
#include "../../buddy3/line_follow.h"
#include "../../buddy3/line_sensors.h"
#include "../../buddy3/line_config.h"
#include "../../buddy4/rtos/buddy4_task.h"
#if TM_WIFI_MQTT
#include "../../../lib/libnet/lwip/mqtt_dashboard_ready.h"
#endif
#ifndef B5_HEADLESS
#define B5_HEADLESS 0
#endif
#ifndef B5_DISABLED
#define B5_DISABLED 0 /* 1 = Buddy 1-4 only: ultrasonic/servo/scan off, front always clear */
#endif
#ifndef B3_BARCODE_ONLY
#define B3_BARCODE_ONLY 0 /* 1 = bench test: never auto-start, never move */
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
static bool buddy4_ready;

#if TM_WIFI_MQTT
static ER report_measurement(uint8_t source, uint8_t type, uint16_t flags,
                               int32_t first, int32_t second, uint32_t sequence)
{
 telemetry_message_t message={source,type,flags,first,second,sequence};
 return buddy1_telemetry_submit(&message);
}
#if B2_ENC_DIAG
/* Temporary: pack encoder pin state into the type-4 flags field.
 * bits0-3 GP4 funcsel (5=SIO ok), 4-7 GP6 funcsel, 8 GP4 level, 9 GP6 level,
 * 10 GP4 irq enabled, 11 GP6 irq enabled, 12 NVIC13 enabled, 13 NVIC13 pending,
 * 14 GP4 raw edge latched, 15 GP6 raw edge latched. */
extern volatile uint32_t b5p_dbg_enc_events, b5p_dbg_cb_calls;
static uint16_t enc_diag_flags(void)
{
 uint32_t f4=(*(volatile uint32_t *)0x40014024u)&0xfu;
 uint32_t f6=(*(volatile uint32_t *)0x40014034u)&0xfu;
 uint32_t in=*(volatile uint32_t *)0xd0000004u;
 uint32_t en=*(volatile uint32_t *)0x40014100u;
 uint32_t nvic_en=(*(volatile uint32_t *)0xE000E100u>>13)&1u;
 uint32_t nvic_pend=(*(volatile uint32_t *)0xE000E200u>>13)&1u;
 uint32_t raw=*(volatile uint32_t *)0x400140f0u;
 return (uint16_t)(f4|(f6<<4)|(((in>>4)&1u)<<8)|(((in>>6)&1u)<<9)|
  ((((en>>16)&12u)?1u:0u)<<10)|((((en>>24)&12u)?1u:0u)<<11)|
  (nvic_en<<12)|(nvic_pend<<13)|((((raw>>16)&12u)?1u:0u)<<14)|((((raw>>24)&12u)?1u:0u)<<15));
}
#define B2_ENC_FLAGS enc_diag_flags()
#else
#define B2_ENC_FLAGS 0
#endif
#endif
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
/* Why the car last stopped itself, reported as telemetry src=5 type=57 (value_1=code, value_2=count). */
volatile int32_t b5_halt_code; volatile int32_t b5_halt_count;
static const char *const halt_reasons[]={"user stop","obstacle scanner heartbeat timeout",
 "obstacle subsystem fault (see SCAN)","line lost outside recovery","request rejected","barcode turn rejected",
 "motor scheduling delay","motor fault","forward range unavailable while moving",
 "navigation failed/pose mismatch","motor reset failed","line follower could not start"};
static void halt(full_nav_t *nav,auto_start_t *start,const char *why) {
 { int32_t code=0; for(unsigned i=0;i<sizeof(halt_reasons)/sizeof(halt_reasons[0]);i++) {
    const char *a=why,*b=halt_reasons[i]; while(*a && *a==*b) { a++; b++; }
    if(*a==0 && *b==0) { code=(int32_t)i+1; break; } }
   b5_halt_code=code; b5_halt_count++; }
 run_enabled=false; auto_start_stop(start); nav->active=false; nav->moving=false; nav->forward_moving=false;
 line_follow_stop(); motion_emergency_stop(); line_scan_abort(); discard_commands();
 tm_printf((UB*)"[AUTO] STOPPED: %s. Power-cycle to restart; optional R retries.\n",(UB*)why);
}
static void controller_task(INT stacd,void *exinf) {
 motion_status_t motor; line_scan_status_t scan;
 full_nav_t nav; auto_start_t start;
 motion_update(); motion_get_status(&motor); full_nav_init(&nav,&motor);
 uint32_t now=buddy5_clock_ms(),last_motor=now,last_line=now,last_report=now;
#if TM_WIFI_MQTT
 uint32_t report_sequence=0;
 uint32_t last_line_report=now-1000u;
 uint8_t last_reported_mask=0;
 line_state_t last_reported_state=LINE_STATE_IDLE;
 uint32_t line_reacquired_count=0;
#endif
 uint32_t lost_since=0,last_guard_clear=0;
#if TM_CONSOLE_USB_CDC || TM_WIFI_MQTT
 uint32_t last_barcode_action_ms=0;
 bool barcode_seen=false;
#endif
 uint8_t mask=0; bool fault_latched=false;
 (void)stacd; (void)exinf;
 auto_start_init(&start,now);
 tm_printf((UB*)"[AUTO] Power-on automatic mode. 3s delay, then wait for line and forward observations.\n");
 tm_printf((UB*)"[AUTO] USB optional. Obstacle trigger=200mm from car front; S/space stops.\n");
 for(;;) {
  now=buddy5_clock_ms(); line_scan_get_status(&scan);
#if B5_DISABLED
  /* No obstacle sensing: present a permanently clear, fresh forward reading. */
  scan.state=B5_MONITORING; scan.fault=(b5_fault_t)0; scan.heartbeat_ms=now; scan.angle=0;
  scan.front_valid=true; scan.front_timestamp_ms=now; scan.front_clearance_mm=B5_CLEAR_FRONT_MM;
#endif
  int command=b5_line_take_stop()?'S':b5_line_take_command();
  if(command=='S') halt(&nav,&start,"user stop");
  if(command=='R' && !run_enabled && nav.settled>=3) {
   auto_start_init(&start,now-3000u); discard_commands();
  }
  if(run_enabled && (uint32_t)(now-scan.heartbeat_ms)>400u)
   halt(&nav,&start,"obstacle scanner heartbeat timeout");
  if(run_enabled && scan.state==B5_ERROR) halt(&nav,&start,"obstacle subsystem fault (see SCAN)");
  if((uint32_t)(now-last_line)>=5u) {
   last_line=now; mask=line_sensors_read();
   if(run_enabled && scan.state==B5_MONITORING && !nav.active) {
    if(!LINE_SENSORS_STRADDLE && !(mask&(LINE_SENSOR_LEFT|LINE_SENSOR_RIGHT))) {
     if(!lost_since) lost_since=now;
     if((uint32_t)(now-lost_since)>=700u)
      halt(&nav,&start,"line lost outside recovery");
    } else lost_since=0;
    if(run_enabled) {
     line_follow_update();
    }
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
#if B5_BYPASS_IGNORE_GUARD
  /* Moving back to the line after the box: the box itself can be inside the forward guard distance while the
   * car faces the line, which used to latch an emergency stop. The sonar reading is still used by the
   * CHECK_FRONT step (reverse/extend), only the self-halt is overridden. */
  if(nav.active && (nav.request.kind==B5_REQ_ADVANCE || nav.request.kind==B5_REQ_RECOVER_LINE ||
                    nav.request.kind==B5_REQ_BACKUP)) guard=true;
#endif
#if TM_CONSOLE_USB_CDC || TM_WIFI_MQTT
  bool barcode_clear=guard_fresh && scan.front_clearance_mm>B5_STOP_FRONT_MM;
  char barcode_letter;
  uint32_t barcode_read_ms;
  if(buddy3_barcode_latest(&barcode_letter,&barcode_read_ms) &&
     (!barcode_seen || barcode_read_ms!=last_barcode_action_ms)) {
   barcode_seen=true;
   last_barcode_action_ms=barcode_read_ms;
   /* A scan read while stopped, turning, or avoiding an obstacle is display-only. */
   /* With the "+" junction logic on, line_follow.c owns barcode-based turns. */
   if(!LINE_JUNCTION && !B3_BARCODE_ONLY && run_enabled && !got && !nav.active && scan.state==B5_MONITORING && barcode_clear &&
      (barcode_letter=='A' || barcode_letter=='B')) {
    b5_request_t turn={.kind=barcode_letter=='A'?B5_REQ_BARCODE_LEFT:B5_REQ_BARCODE_RIGHT};
    if(!full_nav_accept(&nav,&turn,now)) halt(&nav,&start,"barcode turn rejected");
   }
  }
  if(full_nav_barcode_turn_active(&nav)) guard=barcode_clear;
#endif
  if((uint32_t)(now-last_motor)>=20u) {
   if((uint32_t)(now-last_motor)>60u && run_enabled) halt(&nav,&start,"motor scheduling delay");
   last_motor=now; motion_update(); motion_get_status(&motor);
   if(buddy4_ready) {
    uint64_t sample_us;
    /* motion_update() has just sampled both encoders. Timestamp that fresh
     * pair in Buddy 4's monotonic clock domain, never a later cached getter. */
    if(buddy4_task_time_us(&sample_us)>=E_OK) {
     buddy4_task_speed_t speed={true,sample_us,motor.left_speed_cm_s,motor.right_speed_cm_s};
     (void)buddy4_task_submit_speed(&speed);
    }
    buddy4_task_snapshot_t imu;
    if(buddy4_task_get(&imu)>=E_OK) {
     motion_terrain_input_t terrain={0};
     terrain.valid=imu.valid && imu.tilt_valid;
     if(terrain.valid) {
      terrain.pitch_state=(int8_t)imu.terrain;
      terrain.roll_state=(int8_t)imu.side;
      terrain.pitch_deg=imu.pitch_deg;
      terrain.roll_deg=imu.roll_deg;
     }
     (void)motion_set_terrain(&terrain);
    }
   }
   if(motor.fault==MOTION_FAULT_STALL) fault_latched=true;
   if(run_enabled && motor.fault!=MOTION_FAULT_NONE)
    halt(&nav,&start,"motor fault");
   if(run_enabled && nav.forward_moving && !guard)
    halt(&nav,&start,"forward range unavailable while moving");
   full_nav_set_front(&nav,guard_fresh,scan.front_clearance_mm);
   full_nav_update(&nav,now,&motor,mask,guard);
   if(run_enabled && nav.failed) halt(&nav,&start,"navigation failed/pose mismatch");
#if TM_WIFI_MQTT
   if(nav.resumed) ++line_reacquired_count;
#endif
   if(nav.resumed) { tm_printf((UB*)"[AUTO] LINE REACQUIRED: line following resumed.\n"); nav.resumed=false; }
  }
  /* Start the proven Buddy 3 line follower after the power-on delay.
   * The scanner continues in parallel and will take control on an obstacle.
   * Do not make motor start depend on a first ultrasonic/IR sample: a missed
   * first sample otherwise leaves the car permanently stationary. */
  bool ready=motor.fault==MOTION_FAULT_NONE && !fault_latched;
#if TM_WIFI_MQTT
  ready=ready && mqtt_dashboard_ready_after_connect(10000u) && report_sequence!=0;
#endif
#if B5_HEADLESS
  /* A fast power-bank boot can otherwise reach AUTO_START before the first
   * filtered centre ping, skip line_follow_start(), and never retry it. */
  ready=ready && scan.angle==0 && scan.front_valid &&
        (uint32_t)(now-scan.front_timestamp_ms)<=400u;
#endif
#if B3_BARCODE_ONLY
  ready=false; /* barcode-only test: the car must never start moving */
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
   run_enabled=true;
   /* If already 15..20cm from an obstacle, remain stationary and let the
    * first active front sample request STOP/SCAN. Never drive into it. */
   if(scan.front_clearance_mm>B5_STOP_FRONT_MM && !line_follow_start())
    halt(&nav,&start,"line follower could not start");
   else {
    if(buddy4_ready) (void)buddy4_task_begin_run();
    tm_printf((UB*)"[AUTO] RUNNING automatically; front=%dmm (-1=out of range).\n",scan.front_clearance_mm==B5_CLEAR_FRONT_MM?-1:scan.front_clearance_mm);
   }
  }
  b5_feedback_t f;
  full_nav_feedback(&nav,last_motor,&f); f.fault=fault_latched;
  DI(irq); feedback_shared=f; EI(irq);
#if TM_WIFI_MQTT
  if(mqtt_dashboard_ready() && (uint32_t)(now-last_line_report)>=200u) {
   line_status_t current;
   line_follow_get_status(&current);
   if(mask!=last_reported_mask || current.state!=last_reported_state ||
      (uint32_t)(now-last_line_report)>=1000u) {
    if(report_measurement(TELEMETRY_SOURCE_BUDDY3,TELEMETRY_TYPE_LINE_SENSOR,0,
        mask,current.state,report_sequence)==E_OK) {
     last_line_report=now;
     last_reported_mask=mask;
     last_reported_state=current.state;
    }
   }
  }
#endif
  if((uint32_t)(now-last_report)>=1000u
#if TM_WIFI_MQTT
     && mqtt_dashboard_ready()
#endif
  ) {
#if TM_WIFI_MQTT
   ++report_sequence;
   bool tilt_valid=false;
   int32_t pitch=0,roll=0;
#endif
   if(buddy4_ready) {
    buddy4_task_snapshot_t imu;
    if(buddy4_task_get(&imu)>=E_OK) {
#if TM_WIFI_MQTT
     report_measurement(4,11,imu.valid?1:0,
         imu.sample_age_ms==UINT32_MAX?-1:(int32_t)imu.sample_age_ms,imu.io_errors,report_sequence);
     report_measurement(4,12,0,imu.clipped_samples,imu.unsuitable_samples,report_sequence);
     report_measurement(4,13,imu.valid && imu.tilt_valid?1:0,
         imu.terrain,imu.side,report_sequence);
     report_measurement(4,14,(imu.motion_valid?1:0)|(imu.yaw_valid?2:0),
         imu.motion_state,imu.yaw_valid?(int32_t)(imu.encoder_yaw_deg_s*100):0,report_sequence);
     report_measurement(4,15,imu.valid && imu.tilt_valid?1:0,
         imu.valid && imu.tilt_valid?(int32_t)(imu.height_cm*10):0,imu.measurement_quality,report_sequence);
     report_measurement(4,16,(imu.last_event.stable_valid?1:0)|(imu.last_event.median_valid?2:0),
         imu.last_event.stable_valid?(int32_t)(imu.last_event.stable_cm*10):0,
         imu.last_event.median_valid?(int32_t)(imu.last_event.median_cm*10):0,report_sequence);
     report_measurement(4,17,imu.last_event.end_us?1:0,imu.last_event.side,
         (int32_t)(imu.last_event.end_us/1000u),report_sequence);
     report_measurement(4,18,imu.impact_valid?1:0,imu.impact_count,
         (int32_t)(imu.impact_time_us/1000u),report_sequence);
     report_measurement(4,TELEMETRY_TYPE_HIGHEST_HUMP,imu.highest_hump_valid?1:0,
         imu.highest_hump_valid?(int32_t)(imu.highest_hump_cm*10):0,
         (int32_t)imu.run_id,report_sequence);
     tilt_valid=imu.valid && imu.tilt_valid;
     if(tilt_valid) {
      pitch=(int32_t)(imu.pitch_deg*100);
      roll=(int32_t)(imu.roll_deg*100);
     }
#endif
     tm_printf((UB*)"[B4] imu=%d tilt=%d age_ms=%u terrain=%d side=%d impact=%u motion=%d yaw=%d io_errors=%u\n",
      imu.valid,imu.tilt_valid,(unsigned)imu.sample_age_ms,imu.terrain,imu.side,
      (unsigned)imu.impact_count,imu.motion_valid?imu.motion_state:-1,imu.yaw_valid,
      (unsigned)imu.io_errors);
    }
   }
   line_status_t line; line_follow_get_status(&line);
#if TM_WIFI_MQTT
   report_measurement(TELEMETRY_SOURCE_BUDDY2,TELEMETRY_TYPE_MOTOR_SPEED,0,
       (int32_t)(motor.left_speed_cm_s*10),(int32_t)(motor.right_speed_cm_s*10),report_sequence);
   report_measurement(2,TELEMETRY_TYPE_ENCODER,B2_ENC_FLAGS,
#if B2_ENC_DIAG
       (int32_t)b5p_dbg_enc_events,(int32_t)b5p_dbg_cb_calls,report_sequence);  /* DIAG: left=edges seen by IRQ, right=callback calls */
#else
       motion_encoder_get_count(MOTION_SIDE_LEFT),motion_encoder_get_count(MOTION_SIDE_RIGHT),report_sequence);
#endif
   report_measurement(6,57,0,line_reacquired_count,0,report_sequence);
   report_measurement(TELEMETRY_SOURCE_BUDDY2,TELEMETRY_TYPE_ODOMETRY,0,
       (int32_t)(motor.distance_cm*10),motor.fault,report_sequence);
   char letter;
   uint32_t read_ms;
   if(buddy3_barcode_latest(&letter,&read_ms)) {
    uint32_t age_ms=(uint32_t)(b5p_time_us_64()/1000u)-read_ms;
    report_measurement(TELEMETRY_SOURCE_BUDDY3,TELEMETRY_TYPE_BARCODE,0,
        (int32_t)letter,age_ms>INT32_MAX ? INT32_MAX : (int32_t)age_ms,report_sequence);
   }
   report_measurement(TELEMETRY_SOURCE_BUDDY4,TELEMETRY_TYPE_IMU,
       tilt_valid?TELEMETRY_IMU_TILT_VALID:0,pitch,roll,report_sequence);
#endif
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
 motion_emergency_stop(); line_follow_init();
#if !B5_DISABLED
 ultrasonic_init();
 if(!buddy5_servo_init()) return 1;
#endif
 ER buddy1_init_result=buddy1_init();
 if(buddy1_init_result<E_OK)
  tm_printf((UB*)"[B1] Telemetry unavailable (%d); driving continues.\n",buddy1_init_result);
 ER buddy4_init_result=buddy4_task_init();
 buddy4_ready=buddy4_init_result>=E_OK;
 if(!buddy4_ready) tm_printf((UB*)"[B4] IMU task unavailable (%d); driving continues.\n",buddy4_init_result);
 /* Motor timing must outrank the 2 ms Wi-Fi poll task (priority 4). */
 cfg.tskatr=TA_HLNG|TA_RNG3; cfg.task=controller_task; cfg.itskpri=2; cfg.stksz=4096;
 ID id=tk_cre_tsk(&cfg); if(id<0) return 1;
 if(tk_sta_tsk(id,0)<0) { tk_del_tsk(id); return 1; }
#if !B5_DISABLED
 if(line_scan_start()<E_OK) { motion_emergency_stop(); return 1; }
#endif
 if(buddy1_init_result>=E_OK) {
  ER e=buddy1_start();
  if(e<E_OK) tm_printf((UB*)"[B1] Telemetry start failed (%d); driving continues.\n",e);
 }
 if(buddy4_ready) {
  ER e=buddy4_task_start();
  if(e<E_OK) { buddy4_ready=false; tm_printf((UB*)"[B4] IMU task did not start (%d); driving continues.\n",e); }
 }
#if (TM_CONSOLE_USB_CDC || TM_WIFI_MQTT) && !B3_NO_BARCODE
 ER barcode_error=buddy3_barcode_start();
 if(barcode_error<E_OK)
  tm_printf((UB*)"[B3 BARCODE] Task start failed (%d); driving continues.\n",barcode_error);
#endif
 tk_slp_tsk(TMO_FEVR); return 0;
}
#endif
