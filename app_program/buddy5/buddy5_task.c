/* Buddy 5 scan task: ultrasonic + servo hardware, and the glue between the (pure) state machine in buddy5_core.c
 * and the vehicle controller (scan_link.h). Everything that decides anything lives in buddy5_core.c / buddy5_plan.c;
 * this file only measures, moves the servo and passes messages. */
#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <stdint.h>
#include "scan_link.h"
#include "platform.h"
#include "compat/hardware/gpio.h"
#include "compat/hardware/pwm.h"
#include "buddy5_config.h"
#include "buddy5_core.h"

#define B5T_REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define B5_ECHO_MAX_US 24000u     /* ~4 m round trip: nothing closer than this = no echo */
#define B5_ECHO_MIN_US 120u       /* shorter than ~2 cm: electrical noise */
#define B5_SERVO_DIV 64.0f        /* 125 MHz / 64 = 1.953 MHz, wrap 39061 -> 50 Hz */
#define B5_SERVO_WRAP 39061u
#define B5_TASK_PERIOD_MS 5u

uint32_t buddy5_clock_ms(void) {
 uint32_t hi, lo, again;
 do { hi=B5T_REG(0x40054024u); lo=B5T_REG(0x40054028u); again=B5T_REG(0x40054024u); } while(hi!=again);
 return (uint32_t)((((uint64_t)hi<<32)|lo)/1000u);
}

/* ---- echo capture (called from the GPIO interrupt in platform.c) ---- */
static volatile uint32_t echo_rise_us, echo_width_us;
static volatile bool echo_high, echo_done;
static volatile int32_t raw_mm=-1, raw_status=-1;
static volatile uint32_t raw_ms;
static volatile uint32_t obstacle_id;
static volatile uint8_t rescans;
void b5_echo_irq(uint32_t events, uint32_t timestamp_us) {
 if(events & GPIO_IRQ_EDGE_RISE) { echo_rise_us=timestamp_us; echo_high=true; }
 if((events & GPIO_IRQ_EDGE_FALL) && echo_high) {
  echo_width_us=(uint32_t)(timestamp_us-echo_rise_us); echo_high=false; echo_done=true;
 }
}

static void servo_set_angle(int deg) {
 /* + = left. 1500 us centre, 1000 us full left, 2000 us full right, linear over +-90 degrees. */
 int32_t us=(int32_t)B5_SERVO_CENTRE_US-(int32_t)deg*(int32_t)(B5_SERVO_RIGHT_US-B5_SERVO_CENTRE_US)/90;
 if(us<(int32_t)B5_SERVO_LEFT_US) us=(int32_t)B5_SERVO_LEFT_US;
 if(us>(int32_t)B5_SERVO_RIGHT_US) us=(int32_t)B5_SERVO_RIGHT_US;
 b5p_pwm_set_gpio_level(B5_SERVO_PIN,(uint16_t)(((uint32_t)us*125u)/64u));
}

/* One ping. Returns a "bad ping" (valid=false) only if the hardware misbehaved; no echo is a valid answer. */
static b5_range_t ping_once(void) {
 b5_range_t r={false,0};
 echo_done=false; echo_high=false;
 b5p_gpio_put(B5_TRIG_PIN,false);
 uint32_t t0=(uint32_t)b5p_time_us_64();
 while((uint32_t)((uint32_t)b5p_time_us_64()-t0)<4u) { }
 b5p_gpio_put(B5_TRIG_PIN,true);
 t0=(uint32_t)b5p_time_us_64();
 while((uint32_t)((uint32_t)b5p_time_us_64()-t0)<12u) { }
 b5p_gpio_put(B5_TRIG_PIN,false);
 /* The echo pulse starts ~0.5 ms after the burst; wait for it to end (poll politely, the CPU is shared). */
 for(uint32_t waited=0; waited<30u; ++waited) {
  if(echo_done) break;
  tk_dly_tsk(1);
 }
 if(!echo_done) {
  r.valid=true; r.distance_mm=B5_NO_ECHO_MM;   /* nothing came back (or it never fell): open air / far wall */
  raw_mm=-1; raw_status=echo_high?3:2; raw_ms=buddy5_clock_ms();
  return r;
 }
 uint32_t w=echo_width_us;
 raw_ms=buddy5_clock_ms();
 if(w<B5_ECHO_MIN_US) { raw_mm=-1; raw_status=1; return r; }          /* glitch: bad ping */
 if(w>=B5_ECHO_MAX_US) { r.valid=true; r.distance_mm=B5_NO_ECHO_MM; raw_mm=-1; raw_status=4; return r; }
 r.valid=true; r.distance_mm=(w*343u)/2000u;                        /* mm, speed of sound 343 m/s */
 raw_mm=(int32_t)r.distance_mm; raw_status=0;
 return r;
}

/* ---- shared state ---- */
static b5_core_t core;
static b5_sampler_t sampler;
static volatile bool task_started, paused, reset_req;
static line_scan_status_t status_shared;
static ID scan_tid;

static void publish(uint32_t now, int servo_angle, const b5_range_t *last, uint32_t last_ms) {
 line_scan_status_t s={0};
 s.state=core.state; s.fault=core.fault; s.profile=core.profile;
 s.front_clearance_mm=core.front_clearance_mm; s.angle=servo_angle;
 s.front_valid=last->valid; s.front_timestamp_ms=last_ms; s.heartbeat_ms=now;
 s.obstacle_id=obstacle_id; s.rescans=rescans;
 if((uint32_t)(now-raw_ms)>1500u) { s.raw_range_mm=-1; s.raw_status=-1; } else { s.raw_range_mm=raw_mm; s.raw_status=raw_status; }
 UW irq; DI(irq); status_shared=s; EI(irq);
}

static void scan_task(INT stacd, void *exinf) {
 (void)stacd; (void)exinf;
 uint32_t now=buddy5_clock_ms();
 int servo=0; uint32_t settle_until=now+B5_SERVO_SETTLE_MS, next_ping=now, last_ms=now;
 b5_range_t last={false,0};
 servo_set_angle(0);
 tk_dly_tsk(B5_SERVO_SETTLE_MS);
 for(;;) {
  now=buddy5_clock_ms();
  if(reset_req) {
   reset_req=false; b5_core_reset(&core,now); b5_sampler_reset(&sampler);
   servo=0; servo_set_angle(0); settle_until=now+B5_SERVO_SETTLE_MS; last.valid=false;
  }
  if(paused) {
   /* Stopped by the operator: stay in MONITORING, centred, no requests; keep the heartbeat alive. */
   if(servo!=0) { servo=0; servo_set_angle(0); }
   publish(now,servo,&last,last_ms);
   tk_dly_tsk(B5_TASK_PERIOD_MS);
   continue;
  }
  int want=b5_core_angle(&core);
  if(want!=servo) {
   servo=want; servo_set_angle(servo); b5_sampler_reset(&sampler);
   settle_until=now+B5_SERVO_SETTLE_MS; last.valid=false;
  }
  b5_range_t out; bool have=false;
  if((int32_t)(now-next_ping)>=0) {
   next_ping=now+B5_PING_INTERVAL_MS;
   if((int32_t)(now-settle_until)>=0) {
    b5_range_t sample=ping_once();
    if(b5_sampler_push(&sampler,&sample,&out)) have=true;
   }
  }
  if(have) { last=out; last_ms=buddy5_clock_ms(); }
  b5_feedback_t fb; bool fb_ok=line_controller_feedback(&fb);
  b5_core_update(&core,buddy5_clock_ms(),fb_ok?&fb:(b5_feedback_t *)0,have?&out:(b5_range_t *)0);
  {
   static b5_state_t prev_state=B5_MONITORING;
   if(core.state==B5_WAIT_STOP && prev_state!=B5_WAIT_STOP) { ++obstacle_id; rescans=0; }
   uint8_t n=(uint8_t)((core.blocked_mask&1u)+((core.blocked_mask>>1)&1u));
   if(n>rescans) rescans=n;
   prev_state=core.state;
#if B5_SCAN_ONLY
   static uint32_t seen_scans=0, seen_nofaces=0, seen_dump=0, seen_nofaces_dump=0;
   obstacle_id=core.scan_count;
   if(core.scan_count!=seen_scans) {
    seen_scans=core.scan_count;
    tm_printf((UB*)"[B5-SCAN] #%u width=%d mm (left %d + right %d) seen L/R=%d/%d width_valid=%d\n",
     (unsigned)core.scan_count,(int)core.profile.width_mm,(int)core.profile.left_edge_mm,(int)core.profile.right_edge_mm,
     core.profile.left_seen?1:0,core.profile.right_seen?1:0,core.profile.width_valid?1:0);
   }
   if(core.scan_count!=seen_dump || core.noface_count!=seen_nofaces_dump) {
    seen_dump=core.scan_count; seen_nofaces_dump=core.noface_count;
    tm_printf((UB*)"[B5-SCAN] raw scan -35..+25 (E=echo mm, N=no echo, -=invalid):");
    for(int i=0;i<B5_SCAN_POINTS;i++) {
     if(core.scan.state[i]==B5_SCAN_ECHO) tm_printf((UB*)" E%u",(unsigned)core.scan.distance[i]);
     else if(core.scan.state[i]==B5_SCAN_NO_ECHO) tm_printf((UB*)" N");
     else tm_printf((UB*)" -");
    }
    tm_printf((UB*)"\n");
   }
   {
    static uint32_t last_dbg=0;
    uint32_t t=buddy5_clock_ms();
    if((uint32_t)(t-last_dbg)>=1000u) {
     last_dbg=t;
     tm_printf((UB*)"[B5-DBG] state=%s servo=%d raw=%d/st%d clearance=%d scans=%u nofaces=%u saved_width=%d\n",
      (UB*)b5_state_name(core.state),servo,(int)raw_mm,(int)raw_status,(int)core.front_clearance_mm,
      (unsigned)core.scan_count,(unsigned)core.noface_count,(int)core.profile.width_mm);
    }
   }
   if(core.noface_count!=seen_nofaces) {
    seen_nofaces=core.noface_count;
    tm_printf((UB*)"[B5-SCAN] no box face in scan (#%u), last saved width kept=%d mm\n",(unsigned)seen_nofaces,(int)core.profile.width_mm);
   }
#endif
  }
  b5_request_t rq;
  if(b5_core_peek_request(&core,&rq) && line_controller_request(&rq)) b5_core_request_sent(&core,buddy5_clock_ms());
  publish(buddy5_clock_ms(),servo,&last,last_ms);
  tk_dly_tsk(B5_TASK_PERIOD_MS);
 }
}

int32_t line_scan_start(void) {
 if(task_started) return E_OK;
 b5p_gpio_init(B5_TRIG_PIN); b5p_gpio_set_dir(B5_TRIG_PIN,true); b5p_gpio_put(B5_TRIG_PIN,false);
 b5p_gpio_init(B5_ECHO_PIN); b5p_gpio_set_dir(B5_ECHO_PIN,false);
 b5p_gpio_set_irq_enabled(B5_ECHO_PIN,GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
 b5p_gpio_set_function(B5_SERVO_PIN,GPIO_FUNC_PWM);
 pwm_config pc=pwm_get_default_config();
 pwm_config_set_clkdiv(&pc,B5_SERVO_DIV); pwm_config_set_wrap(&pc,(uint16_t)B5_SERVO_WRAP);
 pwm_init(pwm_gpio_to_slice_num(B5_SERVO_PIN),&pc,true);
 servo_set_angle(0);
 b5_core_init(&core,buddy5_clock_ms()); b5_sampler_reset(&sampler);
 uint32_t now=buddy5_clock_ms();
 status_shared=(line_scan_status_t){0};
 status_shared.state=B5_MONITORING; status_shared.front_clearance_mm=B5_CLEAR_FRONT_MM; status_shared.heartbeat_ms=now;
 T_CTSK cfg={0};
 cfg.tskatr=TA_HLNG|TA_RNG3; cfg.task=scan_task; cfg.itskpri=3; cfg.stksz=3072;
 ID id=tk_cre_tsk(&cfg);
 if(id<E_OK) return id;
 scan_tid=id;
 ER e=tk_sta_tsk(id,0);
 if(e<E_OK) return e;
 task_started=true;
 return E_OK;
}
void line_scan_reset(void) { paused=false; reset_req=true; }
void line_scan_abort(void) { paused=true; reset_req=true; }
void line_scan_get_status(line_scan_status_t *status) {
 UW irq; DI(irq); *status=status_shared; EI(irq);
 if(!task_started) { status->state=B5_MONITORING; status->heartbeat_ms=buddy5_clock_ms(); status->front_clearance_mm=B5_CLEAR_FRONT_MM; status->front_valid=true; status->front_timestamp_ms=status->heartbeat_ms; }
}
#endif
