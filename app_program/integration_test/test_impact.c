#include "buddy4_impact.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    for(int i=99;i>=0;i--) {
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
    assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=0;
    /* Same magnitude signal on a strongly tilted vector is not suppressed. */
    xyz(0.6f,0,0.8f);xyz(0.96f,0,1.28f);
    assert(xyz(0.96f,0,1.28f).possible_impact);
}
static void horizontal_contact(void) {
    /* Orthogonal to gravity: old |norm(a)-1| is only 0.0966 g for 0.45 g
     * horizontal contact. Cover both polarities, axes, diagonal and mounting. */
    const float cases[][6]={
        {0,0,1, 0.45f,0,0}, {0,0,1, -0.45f,0,0},
        {0,0,1, 0,0.45f,0}, {0,0,1, 0,-0.45f,0},
        {0,0,1, 0.27f,0.36f,0}, {1,0,0, 0,0,0.45f},
        {0.6f,0,0.8f, 0.36f,0,-0.27f}
    };
    for(unsigned c=0;c<sizeof(cases)/sizeof(cases[0]);c++) {
        const float *b=cases[c], *h=b+3;
        assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=0;
        xyz(b[0],b[1],b[2]);
        buddy4_impact_output_t o=xyz(b[0]+h[0],b[1]+h[1],b[2]+h[2]);
        assert(o.valid&&!o.possible_impact);
        assert(fabsf(o.magnitude_g-o.baseline_g)<0.10f);
        o=xyz(b[0]+h[0],b[1]+h[1],b[2]+h[2]);
        assert(o.possible_impact&&o.event_count==1);
        assert(fabsf(o.deviation_g-0.45f)<1e-5f);
        for(int i=0;i<40;i++) {
            o=xyz(b[0]+h[0],b[1]+h[1],b[2]+h[2]);
            assert(!o.possible_impact&&o.event_count==1); /* Frozen reference. */
        }
        xyz(b[0],b[1],b[2]);xyz(b[0],b[1],b[2]);
        xyz(b[0]+h[0],b[1]+h[1],b[2]+h[2]);
        assert(xyz(b[0]+h[0],b[1]+h[1],b[2]+h[2]).possible_impact);
        assert(d.output.event_count==2);
    }
    /* A negative horizontal change from an established moving baseline is
     * observable too, even though its total magnitude returns toward 1 g. */
    assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=0;
    xyz(0.45f,0,1);assert(!g(1).possible_impact);
    assert(g(1).possible_impact&&d.output.event_count==1);
    /* Below vector threshold remains unconfirmed; do not lower 0.30 g. */
    reset();for(int i=0;i<50;i++)assert(!xyz(0.29f,0,1).possible_impact);
    assert(d.output.event_count==0);
}
static void ordinary_driving(void) {
    reset();
    /* Stationary XYZ noise, including lateral noise invisible to magnitude. */
    for(int i=0;i<500;i++)
        assert(!xyz(0.015f*sinf(i*0.7f),0.02f*cosf(i*0.3f),
                    1+0.015f*sinf(i*0.4f)).possible_impact);
    /* Representative gentle forward start, coast, braking and reverse start.
     * These synthetic envelopes are not measured limits of the real vehicle. */
    const float drive[]={0,0.12f,0,-0.12f,0,-0.12f,0,0.12f,0};
    for(unsigned j=0;j<sizeof(drive)/sizeof(drive[0]);j++)
        for(int i=0;i<30;i++)assert(!xyz(drive[j],0,1).possible_impact);
    for(int i=0;i<300;i++) /* Smooth lateral acceleration through turns. */
        assert(!xyz(0,0.12f*sinf(i*0.02f),1).possible_impact);
    /* Pitch/roll ramp crossings, two 0.6 s slopes, with mild propulsion
     * and vertical vibration. Both sides, with hold and gradual return. */
    for(int axis=0;axis<2;axis++)for(int sign=-1;sign<=1;sign+=2)
        for(int i=0;i<100;i++) {
            float fraction=i<30?i/30.0f:(i<50?1:(i<80?(80-i)/30.0f:0));
            float a=sign*0.20944f*fraction;
            float x=axis==0?sinf(a):0, y=axis==1?sinf(a):0;
            assert(!xyz(x+0.05f*sinf(i*0.06f),y,
                        cosf(a)+0.02f*sinf(i*0.4f)).possible_impact);
        }
    assert(d.output.event_count==0);
    /* Known ambiguity, explicitly retained as POSSIBLE_IMPACT: an abrupt
     * 30-degree reorientation cannot be separated from contact without gyro
     * or external context. Gradual reorientation above does not trigger. */
    reset();xyz(0.5f,0,0.8660254f);
    assert(xyz(0.5f,0,0.8660254f).possible_impact);
}
static void vector_confirmation(void) {
    reset();
    for(int i=0;i<40;i++) { /* Isolated and alternating horizontal spikes. */
        assert(!xyz(i%2?0.6f:-0.6f,0,1).possible_impact);
        assert(!g(1).possible_impact);
    }
    assert(d.output.event_count==0);
    /* Confirmation is elapsed time AND at least two fresh qualifying samples. */
    reset();xyz(0.45f,0,1);
    for(int i=1;i<4;i++) {
        buddy4_impact_update(&d,0.45f,0,1,true,t+i*5000,t+i*5000);
        assert(!d.output.possible_impact);
    }
    t+=20000;buddy4_impact_update(&d,0.45f,0,1,true,t,t);
    assert(d.output.possible_impact);
    /* Allowed 40 ms sample spacing also confirms. Longer gaps cannot bridge. */
    reset();xyz(0.45f,0,1);t+=40000;
    buddy4_impact_update(&d,0.45f,0,1,true,t,t);
    assert(d.output.possible_impact);
    reset();xyz(0.45f,0,1);t+=40001;
    buddy4_impact_update(&d,0.45f,0,1,true,t,t);
    assert(d.output.valid&&!d.output.possible_impact&&d.output.deviation_g==0);
    /* Finite component inputs can still overflow a vector subtraction. */
    assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=0;
    xyz(FLT_MAX*0.75f,0,0);xyz(-FLT_MAX*0.75f,0,0);
    assert(!d.output.valid&&!d.output.possible_impact&&d.output.event_count==0);
    assert(g(1).valid&&!d.output.possible_impact);
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
        imu->output.accel_g[2],result==BUDDY4_FRESH&&imu->output.valid&&imu->output.acceleration_valid,
        imu->output.timestamp_us,now);
    return result;
}
static void corrected_vector(fake_t *sensor,const buddy4_imu_t *imu,float mag) {
    vector(sensor,imu->config.accel_bias_g[0],imu->config.accel_bias_g[1],
        -mag/imu->config.accel_scale[2]+imu->config.accel_bias_g[2]);
}
static void corrected_disturbance(fake_t *sensor,const buddy4_imu_t *imu,int horizontal) {
    if (!horizontal) { corrected_vector(sensor,imu,1.6f); return; }
    vector(sensor,0.45f/imu->config.accel_scale[0]+imu->config.accel_bias_g[0],
        imu->config.accel_bias_g[1],
        -1/imu->config.accel_scale[2]+imu->config.accel_bias_g[2]);
}
static void clipped_acquisition(void) {
 for(int horizontal=0;horizontal<2;++horizontal)for(int history=0;history<2;++history) {
  buddy4_imu_t imu;fake_t sensor;setup(&imu,&sensor);
  imu.config=buddy4_default_config();assert(buddy4_configure(&imu));
  assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=0;
  corrected_vector(&sensor,&imu,1);t+=20000;acquire(&imu,t,t);
  corrected_disturbance(&sensor,&imu,horizontal);t+=20000;acquire(&imu,t,t);
  if(history) {t+=20000;acquire(&imu,t,t);assert(d.output.event_count==1);}
  uint64_t event=d.output.event_us;
  for(int i=0;i<20;++i) {
   raw_vector(&sensor,INT16_MIN,INT16_MIN,INT16_MIN);t+=20000;acquire(&imu,t,t);
   assert(imu.output.valid && !imu.output.acceleration_valid);
   assert(!d.output.valid && !d.output.possible_impact && !d.pending_samples);
   assert(d.output.event_count==(unsigned)history && d.output.event_us==event);
  }
  corrected_vector(&sensor,&imu,1);t+=20000;acquire(&imu,t,t);
  assert(d.output.valid && !d.output.possible_impact);
  corrected_disturbance(&sensor,&imu,horizontal);t+=20000;acquire(&imu,t,t);
  assert(!d.output.possible_impact); /* Cannot bridge clipping. */
 }
}
static void acquisition_faults(void) {
    for(int horizontal=0;horizontal<2;horizontal++)
    for(int history=0;history<2;history++)for(int kind=0;kind<12;kind++) {
        buddy4_imu_t imu;fake_t sensor;
        setup(&imu,&sensor);
        /* Exercise actual production correction, not identity-calibrated data. */
        imu.config=buddy4_default_config();assert(buddy4_configure(&imu));
        assert(buddy4_impact_init(&d,buddy4_impact_default_config()));t=20000;
        corrected_vector(&sensor,&imu,1);
        assert(acquire(&imu,t,t)==BUDDY4_FRESH);
        assert(fabsf(d.output.magnitude_g-1)<0.001f);
        corrected_disturbance(&sensor,&imu,horizontal);t+=20000;
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
        corrected_disturbance(&sensor,&imu,horizontal);
        t+=20000;acquire(&imu,t,t);assert(!d.output.possible_impact);
        t+=20000;acquire(&imu,t,t);
        assert(d.output.possible_impact==!history);
        assert(d.output.event_count==1); /* Recovery cannot bypass old cooldown. */
        corrected_vector(&sensor,&imu,1);
        for(int i=0;i<30;i++){t+=20000;acquire(&imu,t,t);}
        corrected_disturbance(&sensor,&imu,horizontal);
        t+=20000;acquire(&imu,t,t);assert(!d.output.possible_impact);
        t+=20000;acquire(&imu,t,t);
        assert(d.output.possible_impact&&d.output.event_count==2);
    }
}
static void reported_peak_confirmation_limits(void) {
    /* Synthetic counterexamples using the REPORTED 0.5645 g peak, separate
     * from the recorded-vector replay below. Amplitude alone cannot establish its
     * duration, sample freshness or the detector's prior arming state. */
    reset();
    buddy4_impact_output_t out=xyz(0.5645f,0,1);
    assert(out.valid&&fabsf(out.deviation_g-0.5645f)<1e-6f);
    assert(!out.possible_impact&&d.pending_samples==1);
    /* Even a gravity-like magnitude permits a single isolated dynamic sample. */
    assert(out.magnitude_g>=0.8f&&out.magnitude_g<=1.2f);
    out=g(1);
    assert(out.valid&&!out.possible_impact&&out.event_count==0&&d.pending_samples==0);

    /* Two nominal 20 ms releases can be slightly less than 20 ms apart.
     * Such a pair does not meet the present elapsed-time requirement. */
    reset();xyz(0.5645f,0,1);
    t+=19999;buddy4_impact_update(&d,0.5645f,0,1,true,t,t);
    assert(d.output.valid&&!d.output.possible_impact&&d.pending_samples==2);
    assert(!g(1).possible_impact&&d.output.event_count==0);

    /* The same amplitude DOES confirm when fresh evidence spans >=20 ms.
     * Retain this positive control rather than accepting one-sample spikes. */
    reset();xyz(0.5645f,0,1);
    t+=20001;buddy4_impact_update(&d,0.5645f,0,1,true,t,t);
    assert(d.output.valid&&d.output.possible_impact&&d.output.event_count==1);
    /* An already-confirmed disturbance cannot repeat without calm/cooldown. */
    for(int i=0;i<30;i++)assert(!xyz(0.5645f,0,1).possible_impact);
    assert(d.output.event_count==1);
    puts("Reported 0.5645 g synthetic cases: isolated/19999 us pair rejected; 20001 us pair confirmed (not latest-recording replay)");
}
static void recorded_short_impulses(const char *name,unsigned expected_rows,float expected_peak) {
    char path[1024];size_t prefix=0;
    for(size_t i=0;__FILE__[i];i++)if(__FILE__[i]=='/'||__FILE__[i]=='\\')prefix=i+1;
    snprintf(path,sizeof(path),"%.*s../buddy4/evidence/2026-11-02/%s",(int)prefix,__FILE__,name);
    FILE *file=fopen(path,"r");assert(file);
    reset();char line[8192];unsigned rows=0,recorded_above=0,replayed_above=0;
    float peak=0,recorded_peak=0;
    while(fgets(line,sizeof(line),file)) {
        char *data=strstr(line," -> ");data=data?data+4:line;
        if(*data<'0'||*data>'9')continue;
        char *field[87];unsigned n=0;
        for(char *v=strtok(data,",");v&&n<87;v=strtok(NULL,","))field[n++]=v;
        if(n!=86)continue;
        uint64_t sample=strtoull(field[10],NULL,10),now=strtoull(field[1],NULL,10);
        /* Reprime at the first recorded vector: pre-capture XYZ reference is
         * unavailable. Use corrected recorded XYZ without recalibration. */
        if(!rows)assert(buddy4_impact_init(&d,buddy4_impact_default_config()));
        buddy4_impact_update(&d,strtof(field[15],NULL),strtof(field[16],NULL),strtof(field[17],NULL),
            atoi(field[2])==BUDDY4_FRESH&&atoi(field[3])&&atoi(field[5]),sample,now);
        assert(!d.output.possible_impact&&d.output.event_count==0);
        if(d.output.deviation_g>peak)peak=d.output.deviation_g;
        float recorded=strtof(field[71],NULL);
        if(recorded>recorded_peak)recorded_peak=recorded;
        recorded_above+=recorded>=0.30f;
        replayed_above+=d.output.deviation_g>=0.30f;rows++;
    }
    fclose(file);assert(rows==expected_rows&&recorded_above==1);
    assert(replayed_above==1&&fabsf(recorded_peak-expected_peak)<1e-6f);
    /* Missing rows and pre-capture reference cannot reproduce the hidden XYZ
     * baseline exactly. Compare event/confirmation behavior, not exact amplitude. */
    printf("Recorded %s: %u vectors, one above-threshold sample, no impact; replay peak %.4f g (initial reference reprimes)\n",name,rows,peak);
}
int main(void) {
    recorded_short_impulses("foam_contact_1510.txt",773,0.3265f);
    recorded_short_impulses("foam_contact_1513.txt",669,0.5645f);
    recorded_short_impulses("normal_driving_1346.txt",996,0.3175f);
    reported_peak_confirmation_limits();
    quiet();trigger();horizontal_contact();ordinary_driving();vector_confirmation();
    faults();clipped_acquisition();boundaries_and_history();acquisition_faults();
    puts("Preliminary possible-impact synthetic tests passed");return 0;
}
