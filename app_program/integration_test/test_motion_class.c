#include "buddy4_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static buddy4_motion_t c;
static uint64_t t;
static buddy4_motion_output_t tick(float l,float r) {
    t+=20000; buddy4_motion_update(&c,l,r,true,t,t);
    return buddy4_motion_get(&c,t);
}
static void reset(void) {
    assert(buddy4_motion_init(&c,buddy4_motion_default_config())); t=0;
    buddy4_motion_output_t o=buddy4_motion_get(&c,0);
    assert(!o.valid && o.state==BUDDY4_MOTION_UNKNOWN);
}
static void confirmed(float l,float r,buddy4_motion_state_t expected) {
    for(int i=0;i<6;i++) tick(l,r);
    buddy4_motion_output_t o=buddy4_motion_get(&c,t);
    assert(o.valid && o.state==expected);
}
static void cases(void) {
    struct {float l,r; buddy4_motion_state_t s;} cases[]={
      {0,0,BUDDY4_MOTION_STOPPED},{3,3,BUDDY4_MOTION_FORWARD},
      {-3,-3,BUDDY4_MOTION_REVERSE},{1,3,BUDDY4_MOTION_TURNING_LEFT},
      {3,1,BUDDY4_MOTION_TURNING_RIGHT},{-3,-1,BUDDY4_MOTION_TURNING_LEFT},
      {-1,-3,BUDDY4_MOTION_TURNING_RIGHT},{-3,3,BUDDY4_MOTION_TURNING_LEFT},
      {3,-3,BUDDY4_MOTION_TURNING_RIGHT},{0,3,BUDDY4_MOTION_TURNING_LEFT}};
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);i++) {
        reset(); confirmed(cases[i].l,cases[i].r,cases[i].s);
    }
}
static void jitter(void) {
    reset(); confirmed(0,0,BUDDY4_MOTION_STOPPED);
    for(int i=0;i<20;i++) assert(tick(i%2?0.7f:0.4f,0.6f).state==BUDDY4_MOTION_STOPPED);
    for(int i=0;i<20;i++) assert(tick(i%2?0.9f:0.7f,i%2?0.9f:0.7f).state==BUDDY4_MOTION_STOPPED);
    confirmed(1,1,BUDDY4_MOTION_FORWARD);
    confirmed(1,3,BUDDY4_MOTION_TURNING_LEFT);
    for(int i=0;i<20;i++) assert(tick(1,i%2?2.2f:2.6f).state==BUDDY4_MOTION_TURNING_LEFT);
    confirmed(3,3,BUDDY4_MOTION_FORWARD);
    /* At turn entry jitter cannot satisfy confirmation continuously. */
    for(int i=0;i<20;i++) assert(tick(3,i%2?4.4f:4.6f).state==BUDDY4_MOTION_FORWARD);
    reset();
    for(int i=0;i<5;i++) { buddy4_motion_output_t o=tick(3,3); assert(!o.valid); }
    buddy4_motion_update(&c,3,3,true,119999,119999);
    assert(!buddy4_motion_get(&c,119999).valid);
    buddy4_motion_update(&c,3,3,true,120000,120000);
    assert(buddy4_motion_get(&c,120000).state==BUDDY4_MOTION_FORWARD);
}
static void invalid(void) {
    const float bad[]={NAN,INFINITY,-INFINITY};
    for(int side=0;side<2;side++) for(int i=0;i<3;i++) {
        reset(); confirmed(3,3,BUDDY4_MOTION_FORWARD);
        buddy4_motion_output_t o=tick(side?3:bad[i],side?bad[i]:3);
        assert(!o.valid && o.state==BUDDY4_MOTION_UNKNOWN);
        for(int j=0;j<5;j++) assert(!tick(3,3).valid);
        assert(tick(3,3).valid);
    }
    reset(); confirmed(0,0,BUDDY4_MOTION_STOPPED);
    t+=20000; buddy4_motion_update(&c,0,0,false,t,t);
    assert(buddy4_motion_get(&c,t).state==BUDDY4_MOTION_UNKNOWN);
    reset(); confirmed(3,3,BUDDY4_MOTION_FORWARD);
    assert(buddy4_motion_get(&c,t+99999).valid);
    assert(!buddy4_motion_get(&c,t+100000).valid);
    t+=100000; confirmed(-3,-3,BUDDY4_MOTION_REVERSE);
    reset(); confirmed(3,3,BUDDY4_MOTION_FORWARD);
    buddy4_motion_update(&c,3,3,true,t,t); /* Duplicate sample. */
    assert(!buddy4_motion_get(&c,t).valid);
    confirmed(3,3,BUDDY4_MOTION_FORWARD);
    buddy4_motion_update(&c,3,3,true,t-1,t); /* Reversed source time. */
    assert(!buddy4_motion_get(&c,t).valid);
    confirmed(3,3,BUDDY4_MOTION_FORWARD);
    assert(!buddy4_motion_get(&c,t-1).valid); /* Reversed consumer clock. */
    confirmed(3,3,BUDDY4_MOTION_FORWARD);
    t+=20000; buddy4_motion_update(&c,3,3,true,t+1,t);
    assert(!buddy4_motion_get(&c,t).valid); /* Future sample. */
    reset(); buddy4_motion_update(&c,0,0,true,0,100000);
    assert(!buddy4_motion_get(&c,100000).valid); /* Already stale at arrival. */
    reset(); tick(3,3); tick(3,3); t+=100000;
    buddy4_motion_update(&c,3,3,true,t,t);
    assert(!buddy4_motion_get(&c,t).valid); /* Gap cannot complete pending label. */

}
int main(void) {
    cases(); jitter();
    /* UNKNOWN cancellation is tested separately (valid must remain false). */
    reset(); for(int i=0;i<20;i++) assert(!tick(-0.7f,0.7f).valid);
    invalid();
    buddy4_motion_config_t cfg=buddy4_motion_default_config();
    cfg.stop_enter_cm_s=NAN; assert(!buddy4_motion_init(&c,cfg));
    assert(!tick(0,0).valid);
    cfg=buddy4_motion_default_config();cfg.confirm_us=40000;
    assert(buddy4_motion_init(&c,cfg));t=0;
    tick(0,0);tick(0,0);assert(tick(0,0).state==BUDDY4_MOTION_STOPPED);
    assert(!strcmp(buddy4_motion_name(BUDDY4_MOTION_UNKNOWN),"UNKNOWN"));
    puts("Buddy 4 encoder motion classification tests passed");
    return 0;
}

