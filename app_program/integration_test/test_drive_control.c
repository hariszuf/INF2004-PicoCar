#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "drive_control.h"
#include "line_follow.h"
#define main original_imu_quality_fixture_main
#include "../buddy4/tests/test_imu.c"
#undef main
static motion_status_t motor;
static line_status_t line;
static int line_updates, line_starts, drives;
static bool latch;
static float left, right;
void line_follow_stop(void) { line.state = LINE_STATE_IDLE; }
void motion_emergency_stop(void) { latch = true; }
void motion_get_status(motion_status_t *s) { *s = motor; }
motion_result_t motion_reset(void) {
    motor.state = MOTION_STATE_IDLE; motor.fault = MOTION_FAULT_NONE;
    latch = false; return MOTION_OK;
}
motion_result_t motion_drive(float l, float r) {
    assert(!latch); left=l; right=r; drives++; return MOTION_OK;
}
bool line_follow_start(void) { line_starts++; line.state=LINE_STATE_STRAIGHT; return true; }
void line_follow_update(void) { line_updates++; }
void line_follow_get_status(line_status_t *s) { *s=line; }
static void settle_stop(void) { motor.state=MOTION_STATE_ERROR; motor.fault=MOTION_FAULT_ESTOP; }
static void start(drive_control_t *c) {
    settle_stop(); drive_command(c,'R',true,true,0);
    assert(c->armed && !c->running && !latch);
    drive_command(c,'G',true,true,0); assert(c->running && !c->armed);
}
static void begin_turn(drive_control_t *c, int key, uint64_t now) {
    drive_command(c,key,true,true,now);
    assert(!c->running && !c->armed && latch);
    drive_command(c,'G',true,true,now); assert(!c->running);
    settle_stop();
    drive_command(c,'R',true,false,now); assert(!c->armed);
    drive_command(c,'R',true,true,now); assert(c->armed && !c->running);
    drive_command(c,'G',true,true,now); assert(c->running && !c->armed);
    assert(c->started_us==now);
}
static void turn_tests(void) {
    for(int direction=0;direction<2;direction++) {
        drive_control_t c; drive_init(&c);
        int key=direction?'D':'A';
        uint64_t now=UINT64_C(0x100000000)+12345;
        int previous_updates=line_updates, previous_starts=line_starts;
        begin_turn(&c,key,now);
        assert(c.mode==(direction?DRIVE_TURN_RIGHT:DRIVE_TURN_LEFT));
        assert(left==(direction?TURN_TEST_SPEED_CM_S:-TURN_TEST_SPEED_CM_S));
        assert(right==-left && left==c.selected_left_cm_s && right==c.selected_right_cm_s);
        for(int i=0;i<1000;i++) drive_line_update(&c);
        assert(line_updates==previous_updates && line_starts==previous_starts);
        const char modes[]={'M','L','A','D','R','G'};
        for(unsigned i=0;i<sizeof(modes);i++) drive_command(&c,modes[i],true,true,now+1);
        assert(c.running && c.mode==(direction?DRIVE_TURN_RIGHT:DRIVE_TURN_LEFT));
        assert(c.started_us==now); /* G while running cannot extend deadline. */
        drive_safety(&c,true,true,now+999999,now+999999);assert(c.running && !latch);
        drive_safety(&c,true,true,now+1000000,now+1000000);
        assert(!c.running && !c.armed && latch && !strcmp(c.reason,"TURN_TEST_TIMEOUT"));
        drive_command(&c,'G',true,true,now+1000001);assert(!c.running);
        drive_safety(&c,true,true,now+2000000,now+2000000);assert(!c.running);
        begin_turn(&c,key,now+3000000);
        drive_command(&c,'S',true,true,now+3000001);
        assert(!c.running && latch && !strcmp(c.reason,"SERIAL_STOP"));
        begin_turn(&c,key,now+4000000);
        drive_command(&c,' ',true,true,now+4000001);assert(!c.running && latch);
        begin_turn(&c,key,now+5000000);
        drive_safety(&c,false,true,now+5000001,now+5000001);
        assert(!c.running && !c.armed && latch && !strcmp(c.reason,"USB_DISCONNECTED"));
        begin_turn(&c,key,now+6000000);
        motor.state=MOTION_STATE_ERROR;motor.fault=MOTION_FAULT_STALL;
        drive_safety(&c,true,true,now+6000001,now+6000001);
        assert(!c.running && latch && c.stop_fault==MOTION_FAULT_STALL);
        begin_turn(&c,key,now+7000000);
        drive_safety(&c,true,true,now+7500000,now+7000000);
        assert(!c.running && latch && !strcmp(c.reason,"IMU_INVALID_TIMEOUT"));
        /* Selecting a different mode while stopped revokes an existing rearm. */
        settle_stop();drive_command(&c,'R',true,true,now+8000000);assert(c.armed);
        drive_command(&c,key,true,true,now+8000001);assert(!c.armed && latch);
        drive_command(&c,'G',true,true,now+8000002);assert(!c.running);
    }
    puts("Both timed turn modes: direction, rearm, timeout, safety and steering isolation passed");
}
static void imu_quality_reproduction(void) {
    buddy4_imu_t imu;fake_t sensor;setup(&imu,&sensor);
    imu.config=buddy4_default_config();assert(buddy4_configure(&imu));
    drive_control_t c;drive_init(&c);settle_stop();
    bool has_fresh=false;uint64_t last_fresh=0,t=0;
    raw_vector(&sensor,-32760,-32760,-32760);
    for(int i=0;i<792;i++) {
        t+=20000;buddy4_result_t result=buddy4_poll(&imu,t);
        assert(result==BUDDY4_FRESH&&imu.output.valid&&imu.output.acceleration_valid);
        assert(!imu.output.tilt_valid&&imu.output.quality==BUDDY4_QUALITY_NON_GRAVITY);
        assert(!imu.output.clipped_axes&&imu.output.error_count==0);
        assert(imu.ready&&imu.output.valid); /* The old gate incorrectly accepted this. */
        bool usable=drive_imu_poll(&c,&imu,result);
        assert(!usable&&!drive_imu_usable(&imu));
        if(usable) { has_fresh=true;last_fresh=imu.output.timestamp_us; }
        if(i==10)drive_command(&c,'R',true,usable,t);
        if(i==20)drive_command(&c,'G',true,usable,t);
        drive_safety(&c,true,has_fresh,t,last_fresh);
        assert(!c.running&&!c.armed); /* Bad tilt must never arm/start. */
    }
    /* Recovery rows use an actual healthy stationary raw vector from the
     * post-restart capture, repeated for the reported 546-sample interval. */
    raw_vector(&sensor,901,456,-14521);
    for(int i=0;i<546;i++) {
        t+=20000;buddy4_result_t result=buddy4_poll(&imu,t);
        assert(drive_imu_poll(&c,&imu,result)&&drive_imu_usable(&imu));
        assert(!c.running&&!c.armed); /* Health recovery is not rearm. */
    }
    drive_command(&c,'G',true,drive_imu_usable(&imu),t);assert(!c.running);
    drive_command(&c,'R',true,drive_imu_usable(&imu),t);assert(c.armed&&!c.running);
    drive_command(&c,'G',true,drive_imu_usable(&imu),t);assert(c.running);
    raw_vector(&sensor,-32760,-32760,-32760);t+=20000;
    assert(!drive_imu_poll(&c,&imu,buddy4_poll(&imu,t)));
    assert(!c.running&&!c.armed&&latch&&!strcmp(c.reason,"IMU_QUALITY"));
    /* Restored data still cannot restart without a new explicit R. */
    raw_vector(&sensor,901,456,-14521);t+=20000;
    assert(drive_imu_poll(&c,&imu,buddy4_poll(&imu,t)));settle_stop();
    drive_command(&c,'G',true,drive_imu_usable(&imu),t);assert(!c.running);
    drive_command(&c,'R',true,drive_imu_usable(&imu),t);assert(c.armed);
    /* Quality loss while merely armed also revokes permission immediately. */
    raw_vector(&sensor,-32760,-32760,-32760);t+=20000;
    assert(!drive_imu_poll(&c,&imu,buddy4_poll(&imu,t)));
    assert(!c.running&&!c.armed&&latch);
    /* Quality can be unusable in several ways, even with success/zero I2C errors.
     * A finite tilted GRAVITY_LIKE sample remains usable: this is not a flat-only rule. */
    raw_vector(&sensor,901,456,-14521);t+=20000;
    assert(drive_imu_poll(&c,&imu,buddy4_poll(&imu,t)));
    buddy4_imu_t healthy_imu=imu;
    for(int kind=0;kind<7;kind++) {
        imu=healthy_imu;drive_init(&c);start(&c);
        if(kind==0)imu.output.tilt_valid=false;
        if(kind==1)imu.output.acceleration_valid=false;
        if(kind==2)imu.output.clipped_axes=1;
        if(kind==3)imu.output.accel_g[0]=NAN;
        if(kind==4)imu.output.filtered_roll_deg=INFINITY;
        if(kind==5)imu.output.quality=BUDDY4_QUALITY_NON_GRAVITY;
        if(kind==6)imu.ready=false;
        assert(!drive_imu_poll(&c,&imu,BUDDY4_FRESH));
        assert(!c.running&&!c.armed&&latch&&!strcmp(c.reason,"IMU_QUALITY"));
    }
    imu=healthy_imu;imu.output.pitch_deg=imu.output.filtered_pitch_deg=10;
    imu.output.terrain=1;assert(drive_imu_usable(&imu));
    /* Missing acquisition still uses the original bounded watchdog, and a
     * stale/failed poll must not replenish it or permit R/G. */
    imu=healthy_imu;drive_init(&c);start(&c);
    uint64_t last_good=imu.output.timestamp_us;
    sensor.regs[0x27]=0;t=last_good+20000;
    assert(buddy4_poll(&imu,t)==BUDDY4_STALE);
    assert(!drive_imu_poll(&c,&imu,BUDDY4_STALE)&&!drive_imu_usable(&imu));
    drive_safety(&c,true,true,last_good+499999,last_good);assert(c.running);
    drive_safety(&c,true,true,last_good+500000,last_good);
    assert(!c.running&&!c.armed&&latch&&!strcmp(c.reason,"IMU_INVALID_TIMEOUT"));
    puts("SDK drive IMU quality: 792 fresh NON_GRAVITY rows cannot arm; quality loss stops/latches; recovery requires R/G");
}
int main(void) {
    drive_control_t c; drive_init(&c);
    assert(!c.running && !c.armed && latch && c.mode==DRIVE_MANUAL);
    drive_command(&c,'G',true,true,0); assert(!c.running);
    settle_stop(); drive_command(&c,'R',true,false,0); assert(!c.armed);
    start(&c); assert(drives==1 && left==3.0f && right==3.0f && line_starts==0);
    line.state=LINE_STATE_STOPPED;
    for(int i=0;i<1000;i++) drive_line_update(&c);
    assert(line_updates==0 && c.running && drives==1);
    drive_command(&c,'L',true,true,0); assert(c.mode==DRIVE_MANUAL);
    drive_command(&c,'S',true,true,0); assert(latch && !c.running && !c.armed);
    drive_command(&c,'G',true,true,0); assert(!c.running);
    start(&c); drive_command(&c,' ',true,true,0); assert(latch && !c.running);
    start(&c); drive_safety(&c,false,true,20000,20000);
    assert(latch && !c.running && !strcmp(c.reason,"USB_DISCONNECTED"));
    drive_safety(&c,true,true,20001,20001); drive_command(&c,'G',true,true,0); assert(!c.running);
    start(&c); drive_safety(&c,true,true,499999,0); assert(c.running);
    drive_safety(&c,true,true,500000,0); assert(latch && !c.running);
    drive_safety(&c,true,true,500001,500001); assert(!c.running);
    start(&c); motor.state=MOTION_STATE_ERROR; motor.fault=MOTION_FAULT_STALL;
    drive_safety(&c,true,true,10,10); assert(!c.running && latch && c.stop_fault==MOTION_FAULT_STALL);
    drive_command(&c,'L',true,true,0); assert(!c.armed && c.mode==DRIVE_LINE);
    start(&c); assert(line_starts==1);
    line.state=LINE_STATE_STOPPED; drive_line_update(&c);
    assert(line_updates==1 && !c.running && latch && !strcmp(c.reason,"LINE_LOST"));
    start(&c); drive_command(&c,'S',true,true,0); drive_command(&c,'M',true,true,0);
    start(&c); drive_line_update(&c); assert(line_updates==1 && c.running);
    drive_command(&c,'S',true,true,0); settle_stop(); drive_command(&c,'R',true,true,0);
    drive_safety(&c,false,true,10,10); assert(!c.armed && latch);
    turn_tests();
    imu_quality_reproduction();
    puts("Drive safety/mode tests passed"); return 0;
}
