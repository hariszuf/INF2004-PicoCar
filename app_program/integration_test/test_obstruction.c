#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "buddy4_obstruction.h"

/* Independent synthetic fixture. Only 114.86, 114.96 and 115.06 s are
 * operator-reported observations. Intermediate 20 ms values below are invented
 * for continuous replay, NOT recovered hardware samples or contact timestamps. */
typedef struct { uint64_t us; float left, right; } trace_sample_t;
static const trace_sample_t foam[] = {
    {114860000,3.18f,3.09f}, {114880000,1.59f,1.545f},
    {114900000,0.795f,0.7725f}, {114920000,0.42f,0.41f},
    {114940000,0.23f,0.22f}, {114960000,0.12f,0.12f},
    {114980000,0.06f,0.06f}, {115000000,0.03f,0.03f},
    {115020000,0.015f,0.015f}, {115040000,0.008f,0.008f},
    {115060000,0.004f,0.004f}, {115080000,0.002f,0.002f}
};
static const uint64_t stop_us=115090000;
static void reproduce_input(void) {
    assert(foam[0].left>3 && foam[0].right>3);
    assert(foam[5].us==114960000 && foam[5].left==0.12f);
    assert(foam[10].us==115060000 && foam[10].right==0.004f);
    assert(foam[10].us<stop_us && stop_us-foam[5].us==130000);
    for(unsigned i=1;i<sizeof(foam)/sizeof(foam[0]);i++) {
        assert(foam[i].us-foam[i-1].us==20000);
        assert(foam[i].left<foam[i-1].left && foam[i].right<foam[i-1].right);
        assert(foam[i].us<stop_us);
    }
    puts("Independent synthetic replay: both wheels 3.18/3.09 -> 0.12/0.12 -> 0.004/0.004 cm/s before S; intermediate observations assumed");
}
static buddy4_obstruction_t d;
static buddy4_obstruction_input_t in;
static uint64_t t;
static void reset(void) {
    buddy4_obstruction_init(&d);t=0;
    in=(buddy4_obstruction_input_t){.valid=true,.eligible=true,.demand_id=1,
        .command_left_cm_s=3,.command_right_cm_s=3};
}
static buddy4_obstruction_output_t tick(float left,float right) {
    t+=20000;in.observation_us=t;in.left_cm_s=left;in.right_cm_s=right;
    buddy4_obstruction_update(&d,&in,t);return buddy4_obstruction_get(&d,t);
}
static void establish(void) {
    for(int i=0;i<26;i++)assert(!tick(3,3).possible_obstruction);
    assert(d.output.valid&&d.output.established);
}
static void blocked(void) {
    for(int i=0;i<5;i++)assert(!tick(0.12f,0.12f).possible_obstruction);
    assert(tick(0.004f,0.004f).possible_obstruction);
}
static void replay(void) {
    reset();
    for(t=114300000;t<=114840000;t+=20000) {
        in.observation_us=t;in.left_cm_s=3.18f;in.right_cm_s=3.09f;
        buddy4_obstruction_update(&d,&in,t);
    }
    assert(d.output.established);
    uint64_t detected=0;
    for(unsigned i=0;i<sizeof(foam)/sizeof(foam[0]);i++) {
        in.observation_us=foam[i].us;in.left_cm_s=foam[i].left;in.right_cm_s=foam[i].right;
        buddy4_obstruction_update(&d,&in,foam[i].us);
        if(d.output.possible_obstruction)detected=d.output.event_us;
    }
    assert(detected==115020000 && detected<stop_us && d.output.event_count==1);
    buddy4_obstruction_cancel(&d,stop_us);
    assert(!d.output.valid&&!d.output.possible_obstruction&&d.output.event_count==1);
    assert(d.output.event_us==detected);
    printf("Synthetic continuous foam replay: POSSIBLE_OBSTRUCTION at %.2f s before S at 115.09 s (not hardware validation)\n",detected/1e6);
    /* The actual sparse anchors alone cannot bridge 100 ms gaps as fresh input. */
    reset();establish();
    tick(3.18f,3.09f);
    t+=100000;in.observation_us=t;in.left_cm_s=in.right_cm_s=0.12f;
    buddy4_obstruction_update(&d,&in,t);
    t+=100000;in.observation_us=t;in.left_cm_s=in.right_cm_s=0.004f;
    buddy4_obstruction_update(&d,&in,t);
    assert(d.output.event_count==0&&!d.output.established);
}
static void driving_and_stops(void) {
    reset();
    for(int i=0;i<200;i++)tick(0,0); /* Cannot detect startup/non-moving demand. */
    assert(!d.output.established&&d.output.event_count==0);
    establish();
    for(int i=0;i<500;i++)assert(!tick(3+0.15f*sinf(i),3-0.1f*sinf(i)).possible_obstruction);
    /* Same known forward-speed history, then commanded stop before wheel decay. */
    buddy4_obstruction_cancel(&d,t+1);in.eligible=false;
    for(unsigned i=1;i<sizeof(foam)/sizeof(foam[0]);i++)
        assert(!tick(foam[i].left,foam[i].right).possible_obstruction);
    assert(!d.output.valid&&d.output.event_count==0);
    reset();establish();
    for(int i=0;i<40;i++)assert(!tick(1.5f,1.5f).possible_obstruction); /* Partial reduction. */
    assert(d.output.event_count==0);
    reset();establish();tick(0.12f,0.12f);tick(0.12f,0.12f);
    buddy4_obstruction_cancel(&d,t+1);in.eligible=false;
    for(int i=0;i<20;i++)tick(0,0);
    assert(d.output.event_count==0); /* Pending evidence cannot survive S. */
    /* A reduced/new forward target cannot inherit earlier establishment. */
    reset();establish();in.command_left_cm_s=in.command_right_cm_s=2;
    for(int i=0;i<20;i++)tick(0.1f,0.1f);
    assert(!d.output.established&&d.output.event_count==0);
    reset();in.command_left_cm_s=in.command_right_cm_s=1;
    for(int i=0;i<100;i++)tick(1,1);
    for(int i=0;i<100;i++)tick(0,0);
    assert(!d.output.valid&&d.output.event_count==0); /* Below qualified speed envelope. */
}
static void exclusions(void) {
    for(int kind=0;kind<6;kind++) {
        reset();establish();tick(0.12f,0.12f);
        if(kind==0) { for(int i=0;i<30;i++)tick(0,3); }
        if(kind==1) { for(int i=0;i<30;i++)tick(3,0); }
        if(kind==2) { in.command_left_cm_s=2;for(int i=0;i<30;i++)tick(0,0); }
        if(kind==3) { for(int i=0;i<30;i++)tick(-0.1f,-0.1f); }
        if(kind==4) { in.eligible=false;for(int i=0;i<30;i++)tick(0,0); } /* Ramp/fault. */
        if(kind==5) { ++in.demand_id;for(int i=0;i<30;i++)tick(0,0); } /* Mode/session. */
        assert(!d.output.established&&d.output.event_count==0);
    }
    reset();establish();
    for(int i=0;i<50;i++) { tick(0.12f,0.12f);tick(3,3); }
    assert(d.output.event_count==0); /* Isolated low samples are not sustained. */
}
static void faults_and_recovery(void) {
    for(int history=0;history<2;history++)for(int kind=0;kind<9;kind++) {
        reset();establish();if(history)blocked();
        uint64_t event=d.output.event_us;
        tick(0.1f,0.1f);
        in.observation_us=t+20000;in.left_cm_s=in.right_cm_s=0.1f;
        uint64_t now=t+20000;
        if(kind==0)in.valid=false;
        if(kind==1)in.left_cm_s=NAN;
        if(kind==2)in.right_cm_s=INFINITY;
        if(kind==3)in.observation_us=t;
        if(kind==4)in.observation_us=t-1;
        if(kind==5)in.observation_us=now+1;
        if(kind==6)in.observation_us=now-40001;
        if(kind==7)now=t-1;
        if(kind==8) { buddy4_obstruction_get(&d,t+40001);now=t+60000;in.valid=false; }
        buddy4_obstruction_update(&d,&in,now);
        assert(!d.output.valid&&!d.output.possible_obstruction&&!d.output.established);
        assert(d.output.event_count==(unsigned)history&&d.output.event_us==event);
        t+=2000000;in.valid=true;
        for(int i=0;i<20;i++)tick(0,0);
        assert(d.output.event_count==(unsigned)history); /* Recovery cannot prime while stuck. */
        establish();blocked();assert(d.output.event_count==(unsigned)history+1);
    }
}
static void cooldown_and_boundaries(void) {
    reset();
    for(int i=0;i<25;i++)tick(3,3);
    assert(!d.output.established);tick(3,3);assert(d.output.established);
    blocked();uint64_t event=t;
    for(int i=0;i<100;i++)assert(!tick(0,0).possible_obstruction);
    assert(d.output.event_count==1); /* Holding blockage cannot rearm. */
    establish();blocked();assert(d.output.event_count==2);
    reset();establish();blocked();event=t;
    for(int i=0;i<26;i++)tick(3,3);
    assert(!d.output.established); /* Full moving window, but cooldown not complete. */
    while(t<event+980000)tick(3,3);
    assert(!d.output.established);tick(3,3);assert(d.output.established);
    blocked();assert(d.output.event_count==2);
    assert(buddy4_obstruction_get(&d,t+40000).valid);
    assert(!buddy4_obstruction_get(&d,t+40001).valid);
    assert(d.output.event_count==2);
    t+=1200000;establish();d.output.event_count=UINT32_MAX;
    blocked();assert(d.output.event_count==UINT32_MAX&&d.output.event_us==t);
    reset();establish();tick(0.5f,0.5f);
    for(int i=0;i<4;i++)tick(0.5f,0.5f);
    in.observation_us=t+19999;buddy4_obstruction_update(&d,&in,t+19999);
    assert(!d.output.possible_obstruction);
    in.observation_us=t+20000;buddy4_obstruction_update(&d,&in,t+20000);
    assert(d.output.possible_obstruction); /* Inclusive speed and 100 ms edges. */
}
static void asymmetric_transition(void) {
    reset();establish();
    for(int i=0;i<6;i++) {
        buddy4_obstruction_output_t o=tick(2.10f,3.15f);
        assert(o.valid&&o.established&&!o.possible_obstruction&&o.slow_ms==0);
    }
    tick(0.58f,0.99f);assert(d.output.established&&!d.pending);
    blocked();assert(d.output.event_count==1); /* Both-low confirmation is still 100 ms. */
    /* Symmetry never establishes motion by itself, and one-wheel slip alone
     * never triggers. A persistent transition expires rather than being latched. */
    reset();
    for(int i=0;i<20;i++)tick(1,3);
    for(int i=0;i<20;i++)tick(0,0);
    assert(!d.output.established&&d.output.event_count==0);
    for(int slip=0;slip<2;slip++) {
        reset();establish();
        for(int i=0;i<11;i++)assert(!tick(slip?3:0,slip?6:3).possible_obstruction);
        assert(d.output.established); /* Exactly 200 ms since first asymmetry. */
        tick(slip?3:0,slip?6:3);assert(!d.output.established); /* 220 ms: expired. */
        for(int i=0;i<20;i++)tick(0,0);
        assert(d.output.event_count==0);
        establish();blocked();assert(d.output.event_count==1);
    }
    /* Becoming roughly balanced but neither moving nor stopped cannot extend
     * the transition indefinitely. Only recovery or a true both-low candidate resolves it. */
    reset();establish();tick(1,3);
    for(int i=0;i<11;i++)tick(1,1);
    assert(!d.output.established&&d.output.event_count==0);
    /* A low sample before asymmetry must not contribute to later confirmation. */
    reset();establish();tick(0.1f,0.1f);tick(1,3);
    for(int i=0;i<5;i++)assert(!tick(0.1f,0.1f).possible_obstruction);
    assert(tick(0.1f,0.1f).possible_obstruction);
    /* Turns, changed demand/mode, invalid input and gaps always cancel the hold. */
    for(int kind=0;kind<6;kind++) {
        reset();establish();tick(1,3);
        if(kind==0)in.eligible=false;
        if(kind==1)in.command_right_cm_s=2;
        if(kind==2)++in.demand_id;
        if(kind==3)in.valid=false;
        if(kind==4)in.left_cm_s=NAN;
        if(kind==5) {
            t+=40001;in.observation_us=t;in.left_cm_s=in.right_cm_s=0;
        } else { t+=20000;in.observation_us=t; }
        buddy4_obstruction_update(&d,&in,t);
        assert(!d.output.established&&!d.output.possible_obstruction&&d.output.event_count==0);
    }
}
static void pitch_pause_boundaries(void) {
    reset();establish();in.pause_motion=true;tick(0,0);
    uint64_t paused=t;
    tick(0,0);assert(d.paused&&!d.output.valid&&!d.output.established);
    in.pause_motion=false;tick(0,0);
    assert(t==paused+40000&&d.output.established&&d.output.slow_ms==0);
    for(int i=0;i<4;i++)assert(!tick(0,0).possible_obstruction);
    assert(tick(0,0).possible_obstruction);
    uint64_t event=d.output.event_us;
    in.pause_motion=true;tick(0,0); /* No established history after an event. */
    assert(!d.paused&&d.output.event_count==1&&d.output.event_us==event);
    in.pause_motion=false;
    for(int i=0;i<27;i++)tick(3,3);
    assert(!d.output.established); /* Pause cannot bypass event cooldown. */

    for(int kind=0;kind<4;kind++) {
        reset();establish();in.pause_motion=true;tick(0,0);
        assert(d.paused);
        if(kind==0) { /* A persistent excursion expires at exactly 40 ms. */
            tick(0,0);tick(0,0);
        } else if(kind==1) { /* Duplicate is not new paused evidence. */
            buddy4_obstruction_update(&d,&in,t);
        } else if(kind==2) { /* No fresh update: normal age limit still applies. */
            buddy4_obstruction_get(&d,t+40001);t+=40001;
        } else { /* New input carrying stale observation time cannot revive it. */
            buddy4_obstruction_update(&d,&in,t+40001);t+=40001;
        }
        assert(!d.paused&&!d.output.established&&!d.output.possible_obstruction);
        in.pause_motion=false;
        for(int i=0;i<15;i++)assert(!tick(0,0).possible_obstruction);
        assert(!d.output.event_count);
    }
}
int main(void) {
    pitch_pause_boundaries();
    reproduce_input();replay();driving_and_stops();exclusions();
    faults_and_recovery();cooldown_and_boundaries();asymmetric_transition();
    puts("Buddy 4 possible-obstruction core tests passed");return 0;
}
