#include "buddy4_impact.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static buddy4_impact_t d;
static uint64_t t;
static buddy4_impact_output_t xyz(float x,float y,float z) {
    t+=20000; buddy4_impact_update(&d,x,y,z,true,t,t);
    return buddy4_impact_get(&d,t);
}
static buddy4_impact_output_t g(float z) { return xyz(0,0,z); }
static void reset(void) {
    assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=0;
    assert(!buddy4_impact_get(&d,0).valid);
    assert(g(1).valid && !d.output.possible_impact);
}
static void quiet(void) {
    reset();
    for(int i=0;i<200;i++) { assert(!g(1).possible_impact); }
    assert(d.output.event_count==0 && fabsf(d.output.baseline_g-1)<1e-6f);
    /* Slow gravity rotation through a ramp, without suppressing tilted signals. */
    for(int i=0;i<100;i++) {
        float a=i*0.01f; assert(!xyz(sinf(a),0,cosf(a)).possible_impact);
    }
    for(int i=0;i<200;i++) assert(!g(1+0.05f*sinf(i*0.4f)).possible_impact);
    assert(d.output.event_count==0);
    reset(); assert(!g(1.6f).possible_impact); assert(!g(1).possible_impact);
    for(int i=0;i<20;i++) { g(1); g(1.6f); } /* Alternating isolated glitches. */
    assert(d.output.event_count==0);
}
static void trigger(void) {
    reset(); assert(!g(1.6f).possible_impact);
    assert(g(1.6f).possible_impact && d.output.event_count==1 && d.output.event_us==t);
    assert(!g(1.6f).possible_impact); /* One input-sample pulse. */
    for(int i=0;i<40;i++) assert(!g(1.6f).possible_impact);
    assert(d.output.event_count==1); /* Sustained input never auto-rearms. */
    g(1); g(1); assert(!g(1.6f).possible_impact);
    assert(g(1.6f).possible_impact && d.output.event_count==2);
    /* Quiet release before cooldown ends cannot trigger. */
    g(1);g(1);g(1.6f);assert(!g(1.6f).possible_impact);
    assert(d.output.event_count==2);
    for(int i=0;i<30;i++) g(1);
    g(1.6f);assert(g(1.6f).possible_impact && d.output.event_count==3);
    reset();g(0.4f);assert(g(0.4f).possible_impact); /* Negative transient. */
    reset();
    /* Same magnitude signal on a strongly tilted vector is not suppressed. */
    xyz(0.6f,0,0.8f);xyz(0.96f,0,1.28f);
    assert(xyz(0.96f,0,1.28f).possible_impact);
}
static void faults(void) {
    for(int kind=0;kind<8;kind++) {
        reset(); g(1.6f); /* Pending evidence must not survive any fault. */
        uint64_t before=t;
        if(kind==0) { t+=20000; buddy4_impact_update(&d,0,0,1.6f,false,t,t); }
        if(kind==1) { t+=20000; buddy4_impact_update(&d,NAN,0,1,true,t,t); }
        if(kind==2) { t+=20000; buddy4_impact_update(&d,0,INFINITY,1,true,t,t); }
        if(kind==3) buddy4_impact_update(&d,0,0,1.6f,true,t,t); /* duplicate */
        if(kind==4) buddy4_impact_update(&d,0,0,1.6f,true,t-1,t);
        if(kind==5) { t+=40001; buddy4_impact_get(&d,t); }
        if(kind==6) buddy4_impact_update(&d,0,0,1.6f,true,t+1,t);
        if(kind==7) buddy4_impact_get(&d,t-1);
        assert(!d.output.valid && !d.output.possible_impact && d.pending_samples==0);
        assert(d.output.event_count==0);
        t=before+100000; assert(g(1).valid); /* Recovery primes baseline. */
        assert(!g(1.6f).possible_impact); assert(g(1.6f).possible_impact);
    }
    reset();g(1.6f);t+=40001;
    buddy4_impact_update(&d,0,0,1,true,t,t); /* Unobserved gap resets/reprimes. */
    assert(d.output.valid && !d.output.possible_impact && d.pending_samples==0);
    g(1.6f);assert(g(1.6f).possible_impact);
    uint64_t event=t;
    t+=20000;buddy4_impact_update(&d,0,0,1,false,t,t);
    assert(d.output.event_count==1 && d.output.event_us==event);
    g(1);g(1);g(1);g(1.6f);assert(!g(1.6f).possible_impact);
    assert(d.output.event_count==1); /* Fault cannot bypass cooldown. */
    reset();g(1.6f);
    buddy4_impact_update(&d,0,0,1.6f,true,t+19999,t+19999);
    assert(!d.output.possible_impact);
    buddy4_impact_update(&d,0,0,1.6f,true,t+20000,t+20000);
    assert(d.output.possible_impact); /* Exact confirmation timing. */
    buddy4_impact_config_t cfg=buddy4_impact_default_config();cfg.trigger_g=NAN;
    assert(!buddy4_impact_init(&d,cfg));assert(!g(1).valid);
    cfg=buddy4_impact_default_config();cfg.trigger_g=0.8f;
    assert(buddy4_impact_init(&d,cfg));g(1);g(1.6f);assert(!g(1.6f).possible_impact);
}
int main(void) {
    quiet();trigger();faults();
    puts("Preliminary possible-impact synthetic tests passed");return 0;
}
