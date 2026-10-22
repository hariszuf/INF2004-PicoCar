#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "drive_control.h"
#include "line_follow.h"
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
    puts("Drive safety/mode tests passed"); return 0;
}
