#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy5.h"
#include "buddy5_servo.h"
#include "ultrasonic.h"
#include "../buddy1/telemetry.h"
#if TK_SUPPORT_SMP
#error "Buddy 5 snapshot locking is qualified for SMP=0 only."
#endif
static ID task_id;
static bool initialised, reset_pending;
static buddy5_controller_t controller;
static buddy5_status_t published;
static b5_core_t core;
static uint32_t telemetry_dropped, report_sequence;
static void publish(uint32_t now,int angle) {
    buddy5_status_t s={0};
    UW irq;

    s.state=core.state; s.fault=core.fault; s.profile=core.profile;
    s.request=core.request; s.front_clearance_mm=core.front_clearance_mm;
    s.timestamp_ms=now; s.telemetry_dropped=telemetry_dropped;
    s.servo_angle_deg=(int16_t)angle; s.bench_mode=B5_BENCH_MODE!=0;
    DI(irq); published=s; EI(irq);
}
static void send_message(uint8_t type,int32_t a,int32_t b,uint32_t seq) {
    telemetry_message_t m={0};

    m.source=TELEMETRY_SOURCE_BUDDY5; m.type=type;
    m.flags=(core.profile.width_valid?1u:0u) |
            (core.profile.depth_valid?2u:0u) | (B5_BENCH_MODE?4u:0u);
    m.value_1=a; m.value_2=b; m.sequence=seq;
    if(buddy1_telemetry_submit(&m)<E_OK) ++telemetry_dropped;
}
static void telemetry(uint32_t now,int angle) {
    uint32_t seq=++report_sequence;
    send_message(B5_TELEM_STATE,core.state,core.fault,seq);
    send_message(B5_TELEM_SIZE,core.profile.width_valid?core.profile.width_mm:-1,
                 core.profile.depth_valid?core.profile.depth_high_mm:-1,seq);
    send_message(B5_TELEM_PLAN,core.profile.side,core.profile.offset_mm,seq);
    send_message(B5_TELEM_DEPTH_BOUNDS,core.profile.depth_valid?core.profile.depth_low_mm:-1,
                 core.profile.depth_valid?core.profile.depth_high_mm:-1,seq);
    send_message(B5_TELEM_FRONT,core.front_clearance_mm,angle,seq);
    /* 55: correlate snapshot sequence with obstacle id and acquisition uptime. */
    send_message(55u,(int32_t)core.profile.obstacle_id,(int32_t)now,seq);
    send_message(56u,(core.profile.width_valid?1:0) | (core.profile.depth_valid?2:0) | (B5_BENCH_MODE?4:0), (int32_t)telemetry_dropped,seq);
}
static bool get_feedback(b5_feedback_t *f,uint32_t now) {
    *f=(b5_feedback_t){0};
#if B5_BENCH_MODE
    /* Explicit stationary test fixture assumption. NEVER a driving adapter. */
    f->valid=true; f->stopped=true; f->heading_aligned=true;
    f->timestamp_ms=now;
    if(core.request.kind==B5_REQ_STOP) f->completed_id=core.request.id;
    return true;
#else
    (void)now;
    return controller.read_feedback &&
           controller.read_feedback(controller.context,f);
#endif
}
static void task(INT stacd,void *exinf) {
    b5_range_t samples[B5_SAMPLE_COUNT];
    unsigned count=0;
    int angle=999;
    uint32_t settled_since=0,last_ping=0,last_report=0,delivered=0;
    b5_state_t previous=(b5_state_t)99;
    uint32_t sampled_request=0;
    (void)stacd; (void)exinf;
    tm_printf((UB*)"[Buddy5] %s; car=355x170mm front=28 rear=327\n",
              B5_BENCH_MODE ? "STATIONARY BENCH (motor power OFF)" : "CONTROLLER MODE");
    for(;;) {
        uint32_t now=buddy5_clock_ms();
        b5_feedback_t f;
        UW irq;
        bool reset;
        (void)get_feedback(&f,now);
        DI(irq); reset=reset_pending; reset_pending=false; EI(irq);
        if(reset) {
#if B5_BENCH_MODE
            uint32_t seq=core.next_request_id, id=core.profile.obstacle_id;
            b5_core_init(&core,now);
            core.next_request_id=seq; core.profile.obstacle_id=id;
#else
            (void)b5_core_reset(&core,now,&f);
#endif
            count=0; angle=999;
        }
        /* A bench plan is held for inspection until explicit reset/reboot.
         * It is never acknowledged as a completed movement. */
        bool hold=B5_BENCH_MODE && core.state==B5_PROCESSING;
        if(!hold) b5_core_update(&core,now,&f,NULL);
        int wanted=b5_core_angle(&core);
        if(wanted!=angle) {
            count=0;
            if(!buddy5_servo_set_angle(wanted))
                b5_core_fail(&core,B5_FAULT_SERVO,now);
            else { angle=wanted; settled_since=now; }
        }
        if(sampled_request!=core.request.id) {
            sampled_request=core.request.id; count=0;
        }
        if(!hold && b5_core_wants_range(&core) &&
           (uint32_t)(now-settled_since)>=B5_SERVO_SETTLE_MS &&
           (uint32_t)(now-last_ping)>=B5_PING_INTERVAL_MS) {
            ultrasonic_result_t raw=ultrasonic_read();
            last_ping=buddy5_clock_ms(); /* >70ms from end of prior transaction */
            samples[count++]=(b5_range_t){raw.status==US_OK,raw.distance_mm};
            /* Near/invalid forward samples stop promptly, without waiting for
             * a median batch while the line follower might still be moving. */
            bool urgent=(core.state==B5_MONITORING) &&
                (raw.status!=US_OK || raw.distance_mm<=B5_STOP_FRONT_MM+B5_FRONT_OVERHANG_MM);
            if(count==B5_SAMPLE_COUNT || urgent) {
                b5_range_t result=urgent ? samples[count-1] : b5_filter(samples);
                now=buddy5_clock_ms();
                (void)get_feedback(&f,now);
                if(core.state==B5_SCANNING)
                    tm_printf((UB*)"[Buddy5 scan] angle=%d valid=%d mm=%u\n",
                              angle,result.valid,(unsigned)result.distance_mm);
                b5_core_update(&core,now,&f,&result);
                count=0;
            }
        }
        if(core.request.id && core.request.id!=delivered) {
            delivered=core.request.id;
#if !B5_BENCH_MODE
            if(!controller.submit_request ||
               !controller.submit_request(controller.context,&core.request)) {
                if(core.state==B5_ERROR) delivered=0; /* retry refused STOP */
                else b5_core_fail(&core,B5_FAULT_CONTROLLER,now);
                /* Error STOP is sent on the next loop as a new id. */
            }
#endif
        }
        if(core.state!=previous) {
            tm_printf((UB*)"[Buddy5] state=%s fault=%d width=%d side=%d offset=%d\n",
              (UB*)b5_state_name(core.state),core.fault,
              core.profile.width_valid?core.profile.width_mm:-1,
              core.profile.side,core.profile.offset_mm);
            previous=core.state;
        }
        if((uint32_t)(now-last_report)>=B5_TELEMETRY_INTERVAL_MS) {
            telemetry(now,angle); last_report=now;
        }
        publish(now,angle);
        tk_dly_tsk(10);
    }
}
ER buddy5_bind_controller(const buddy5_controller_t *binding) {
    if(task_id>0) return E_OBJ;
    if(!binding || !binding->read_feedback || !binding->submit_request) return E_PAR;
    controller=*binding;
    return E_OK;
}
ER buddy5_init(void) {
    if(initialised) return E_OK;
    ultrasonic_init();
    if(!buddy5_servo_init()) return E_SYS;
    b5_core_init(&core,buddy5_clock_ms());
    publish(buddy5_clock_ms(),0);
    initialised=true;
    return E_OK;
}
ER buddy5_start(void) {
    T_CTSK cfg={0};
    ER result;
    if(!initialised || task_id>0) return E_OBJ;
#if !B5_BENCH_MODE
    if(!controller.read_feedback || !controller.submit_request) return E_OBJ;
#endif

    cfg.itskpri=B5_TASK_PRIORITY; cfg.stksz=4096;
    cfg.task=task; cfg.tskatr=TA_HLNG|TA_RNG3;
    task_id=tk_cre_tsk(&cfg);
    if(task_id<0) { result=task_id; task_id=0; return result; }
    result=tk_sta_tsk(task_id,0);
    if(result<0) { tk_del_tsk(task_id); task_id=0; }
    return result;
}
ER buddy5_get_status(buddy5_status_t *status) {
    UW irq;
    if(!status) return E_PAR;
    if(!initialised) return E_OBJ;
    DI(irq); *status=published; EI(irq);
    return E_OK;
}
ER buddy5_reset(void) {
    UW irq;
    if(!initialised) return E_OBJ;
    DI(irq); reset_pending=true; EI(irq);
    return E_OK;
}