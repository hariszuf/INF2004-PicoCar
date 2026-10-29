#include "buddy4_impact.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
/* Reuse the authoritative fake-I2C fixtures, including transfer injection. */
#define main original_imu_test_main
#include "../buddy4/tests/test_imu.c"
#undef main
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
static void boundaries_and_history(void) {
    /* DC bias and gentle drift must track a nonzero gravity baseline. */
    reset();
    for(int i=0;i<400;i++) assert(!g(1+i*0.0005f).possible_impact);
    assert(d.output.event_count==0 && d.output.baseline_g>1.15f);

    reset();g(1.6f);assert(g(1.6f).possible_impact);
    uint64_t event=t;
    for(int i=0;i<3;i++) {
        buddy4_impact_output_t o=buddy4_impact_get(&d,t);
        assert(o.valid&&o.possible_impact&&o.event_count==1&&o.event_us==event);
    }
    /* The pulse belongs to the last input sample; getters do not consume it. */
    assert(buddy4_impact_get(&d,event+40000).valid);
    assert(!buddy4_impact_get(&d,event+40001).valid);
    assert(!d.output.possible_impact&&d.output.event_count==1&&d.output.event_us==event);
    t=event+40001;assert(g(1).valid&&!d.output.possible_impact);
    for(int i=0;i<30;i++)g(1);
    /* Exercise the bounded counter without billions of synthetic events. */
    d.output.event_count=UINT32_MAX;
    g(1.6f);assert(g(1.6f).possible_impact);
    assert(d.output.event_count==UINT32_MAX&&d.output.event_us==t);

    /* Exact cooldown edge, with two calm observations already available. */
    reset();g(1.6f);g(1.6f);event=t;
    for(int i=0;i<24;i++)g(1);
    assert(t==event+480000&&!d.armed);
    g(1);assert(t==event+500000&&d.armed);
    g(1.6f);assert(g(1.6f).possible_impact&&d.output.event_count==2);
    /* Cooldown alone is insufficient; one calm sample cannot rearm. */
    for(int i=0;i<30;i++)g(1.6f);
    g(1);g(1.6f);assert(!g(1.6f).possible_impact&&d.output.event_count==2);
    g(1);g(1);g(1.6f);assert(g(1.6f).possible_impact&&d.output.event_count==3);

    /* Binary-exact criteria isolate >= trigger, <= release and gap boundaries;
     * these are local test settings, not changes to firmware defaults. */
    buddy4_impact_config_t cfg=buddy4_impact_default_config();
    cfg.trigger_g=0.25f;cfg.release_g=0.125f;
    assert(buddy4_impact_init(&d,cfg));t=0;
    g(1);assert(!g(1.25f).possible_impact);
    t+=40000;buddy4_impact_update(&d,0,0,1.25f,true,t,t);
    assert(d.output.possible_impact&&d.output.event_count==1);
    for(int i=0;i<30;i++)g(1.25f);
    g(1.125f);assert(d.calm_samples==1&&!d.armed);
    g(1);assert(d.armed);
    reset();g(1.6f);t+=40001;
    buddy4_impact_update(&d,0,0,1.6f,true,t,t);
    assert(d.output.valid&&!d.output.possible_impact&&d.output.event_count==0);
    assert(d.output.deviation_g==0); /* Gap recovery primes, never confirms. */

    /* Stale input supplied to update, and reversal of its clock, were not
     * covered by the older getter-only fault cases. Retain event history. */
    for(int kind=0;kind<2;kind++) {
        reset();g(1.6f);g(1.6f);event=t;
        if(kind==0)buddy4_impact_update(&d,0,0,1,true,t+1,t+40002);
        else buddy4_impact_update(&d,0,0,1,true,t-1,t-1);
        assert(!d.output.valid&&!d.output.possible_impact);
        assert(d.output.event_count==1&&d.output.event_us==event);
        t=event+60000;g(1);g(1);g(1);g(1.6f);
        assert(!g(1.6f).possible_impact&&d.output.event_count==1);
    }
}
static buddy4_result_t acquire(buddy4_imu_t *imu,uint64_t sample,uint64_t now) {
    buddy4_result_t result=buddy4_poll(imu,sample);
    /* Same freshness gate and corrected axes used by SDK and isolated MTK. */
    buddy4_impact_update(&d,imu->output.accel_g[0],imu->output.accel_g[1],
        imu->output.accel_g[2],result==BUDDY4_FRESH&&imu->output.valid,
        imu->output.timestamp_us,now);
    return result;
}
static void corrected_vector(fake_t *sensor,const buddy4_imu_t *imu,float mag) {
    vector(sensor,imu->config.accel_bias_g[0],imu->config.accel_bias_g[1],
        -mag/imu->config.accel_scale[2]+imu->config.accel_bias_g[2]);
}
static void acquisition_faults(void) {
    for(int history=0;history<2;history++)for(int kind=0;kind<12;kind++) {
        buddy4_imu_t imu;fake_t sensor;
        setup(&imu,&sensor);
        /* Exercise actual production correction, not identity-calibrated data. */
        imu.config=buddy4_default_config();assert(buddy4_configure(&imu));
        assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=20000;
        corrected_vector(&sensor,&imu,1);
        assert(acquire(&imu,t,t)==BUDDY4_FRESH);
        assert(fabsf(d.output.magnitude_g-1)<0.001f);
        corrected_vector(&sensor,&imu,1.6f);t+=20000;
        assert(acquire(&imu,t,t)==BUDDY4_FRESH&&!d.output.possible_impact);
        if(history) {
            t+=20000;assert(acquire(&imu,t,t)==BUDDY4_FRESH);
            assert(d.output.possible_impact&&d.output.event_count==1);
        }
        uint64_t event=d.output.event_us,last_sample=imu.output.timestamp_us;
        t+=20000;
        buddy4_result_t expected=BUDDY4_STALE;
        uint64_t sample=t;
        if(kind==0)sensor.regs[0x27]=0; /* Missing data-ready. */
        if(kind==1)sample=last_sample;
        if(kind==2)sample=last_sample-1;
        if(kind>=3&&kind<=10) {
            int transfer=(kind-3)/2+1;
            if(kind%2)sensor.fail_at=sensor.calls+transfer;
            else sensor.short_at=sensor.calls+transfer;
            expected=BUDDY4_IO_ERROR;
        }
        if(kind==11) {imu.ready=false;expected=BUDDY4_NOT_READY;}
        assert(acquire(&imu,sample,t)==expected);
        assert(!d.output.valid&&!d.output.possible_impact&&d.pending_samples==0);
        assert(d.output.event_count==(uint32_t)history&&d.output.event_us==event);
        assert(imu.output.timestamp_us==last_sample); /* Retained data is invalid. */
        sensor.fail_at=sensor.short_at=0;sensor.regs[0x27]=8;
        assert(buddy4_configure(&imu));
        corrected_vector(&sensor,&imu,1);
        for(int i=0;i<3;i++) {
            t+=20000;assert(acquire(&imu,t,t)==BUDDY4_FRESH);
            assert(d.output.valid&&!d.output.possible_impact);
        }
        corrected_vector(&sensor,&imu,1.6f);
        t+=20000;acquire(&imu,t,t);assert(!d.output.possible_impact);
        t+=20000;acquire(&imu,t,t);
        assert(d.output.possible_impact==!history);
        assert(d.output.event_count==1); /* Recovery cannot bypass old cooldown. */
        corrected_vector(&sensor,&imu,1);
        for(int i=0;i<30;i++){t+=20000;acquire(&imu,t,t);}
        corrected_vector(&sensor,&imu,1.6f);
        t+=20000;acquire(&imu,t,t);assert(!d.output.possible_impact);
        t+=20000;acquire(&imu,t,t);
        assert(d.output.possible_impact&&d.output.event_count==2);
    }
}
int main(void) {
    quiet();trigger();faults();boundaries_and_history();acquisition_faults();
    puts("Preliminary possible-impact synthetic tests passed");return 0;
}
