#include "buddy4_obstruction_sdk.h"
#include "scheduler.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static buddy4_obstruction_sdk_t a;
static drive_control_t control;
static motion_status_t motor;
static buddy4_imu_t imu;
static uint64_t t;
static uint32_t skipped;
static void reset(void) {
    buddy4_obstruction_sdk_init(&a);t=0;skipped=0;
    control=(drive_control_t){.running=true,.mode=DRIVE_MANUAL,.started_us=1,
        .selected_left_cm_s=3,.selected_right_cm_s=3};
    motor=(motion_status_t){.state=MOTION_STATE_EXECUTING,.terrain_speed_scale=1};
    imu=(buddy4_imu_t){.ready=true};
    imu.output.valid=imu.output.acceleration_valid=imu.output.tilt_valid=true;
}
static void note(float left,float right) {
    t+=20000;motor.left_speed_cm_s=left;motor.right_speed_cm_s=right;
    imu.output.timestamp_us=t+500;
    buddy4_obstruction_sdk_note_update(&a,&motor,&control,t,t+100,skipped);
}
static buddy4_obstruction_output_t poll(void) {
    return buddy4_obstruction_sdk_poll(&a,&control,&imu,true,t+1000);
}
static buddy4_obstruction_output_t step(float l,float r) { note(l,r);return poll(); }
static void establish(void) {
    for(int i=0;i<27;i++)assert(!step(3,3).possible_obstruction);
    assert(a.observer.output.valid&&a.observer.output.established);
}
static void pending(void) {
    for(int i=0;i<5;i++)assert(!step(0.12f,0.12f).possible_obstruction);
}
static void ordinary_and_blocked(void) {
    reset();establish();
    for(int i=0;i<100;i++)assert(!step(3.1f,2.9f).possible_obstruction);
    pending();assert(step(0.004f,0.004f).possible_obstruction);
    assert(a.observer.output.event_count==1);
    for(int i=0;i<20;i++)assert(poll().event_count==1); /* Repeated getters are not updates. */
    for(int i=0;i<100;i++)assert(!step(0,0).possible_obstruction);
    establish();pending();assert(step(0,0).possible_obstruction);
    assert(a.observer.output.event_count==2);
    reset();establish();
    for(int i=0;i<5;i++)step(3.18f,3.09f); /* Normal pre-stop comparison. */
    control.running=false;
    for(int i=0;i<20;i++)assert(!step(0,0).possible_obstruction);
    assert(a.observer.output.event_count==0);
}
static void command_cancellation(void) {
    for(int kind=0;kind<6;kind++) {
        reset();establish();pending();note(0,0); /* Would confirm this observation. */
        if(kind==0)control.running=false; /* S processed AFTER motor update. */
        if(kind==1)control.mode=DRIVE_TURN_LEFT;
        if(kind==2)control.mode=DRIVE_LINE;
        if(kind==3)++control.started_us; /* New drive/session. */
        if(kind==4)control.selected_left_cm_s=2;
        if(kind==5)control.stop_fault=MOTION_FAULT_STALL;
        assert(!poll().possible_obstruction&&a.observer.output.event_count==0);
        assert(!a.observer.output.established);
    }
    reset();control.running=false;note(3,3);
    control.running=true;++control.started_us;
    assert(!poll().valid); /* G cannot reuse a pre-command motor observation. */
}
static void ramps_turns_faults(void) {
    for(int kind=0;kind<12;kind++) {
        reset();establish();pending();
        if(kind==0)imu.output.terrain=1;
        if(kind==1)imu.output.side=-1;
        if(kind==2)imu.output.filtered_pitch_deg=3.01f; /* Before 6 deg terrain entry. */
        if(kind==3)imu.output.filtered_roll_deg=-2.01f; /* Before 4 deg side entry. */
        if(kind==4)imu.output.filtered_pitch_deg=4;
        if(kind==5)motor.terrain=MOTION_TERRAIN_DOWNHILL;
        if(kind==6)motor.terrain_speed_scale=0.8f;
        if(kind==7)motor.left_terrain_duty=0.05f;
        if(kind==8)motor.fault=MOTION_FAULT_STALL;
        if(kind==9)motor.state=MOTION_STATE_STOPPING;
        if(kind==10)control.mode=DRIVE_TURN_RIGHT;
        if(kind==11)control.selected_right_cm_s=2;
        for(int i=0;i<20;i++)assert(!step(0,0).possible_obstruction);
        assert(!a.observer.output.valid&&a.observer.output.event_count==0);
    }
    reset();establish();
    for(int i=0;i<50;i++)assert(!step(0,3).possible_obstruction);
    assert(a.observer.output.event_count==0&&!a.observer.output.established);
}
static void timing_and_invalid(void) {
    for(int kind=0;kind<18;kind++) {
        reset();establish();pending();
        uint64_t imu_before=imu.output.timestamp_us;
        if(kind==0)skipped++;
        if(kind==1)t+=40000; /* Missing update: cadence no longer qualified. */
        if(kind==2)t-=15000; /* Too-short interval, Buddy 2 would clamp dt. */
        if(kind==3)imu.ready=false;
        if(kind==4)imu.output.valid=false;
        if(kind==5)imu.output.acceleration_valid=false;
        if(kind==6)imu.output.tilt_valid=false;
        if(kind==7)imu.output.clipped_axes=1;
        if(kind==8)imu.output.roll_deg=NAN;
        note(kind==9?NAN:0,kind==10?INFINITY:0);
        if(kind==11)imu.output.timestamp_us=t-40001;
        if(kind==12)imu.output.timestamp_us=t+1001; /* Future relative to poll. */
        if(kind==13)imu.output.timestamp_us=t-31000; /* Fresh age, excess cross-source skew. */
        if(kind==14)imu.output.timestamp_us=imu_before-1; /* Reversed IMU time is a fault. */
        if(kind==15)buddy4_obstruction_sdk_note_update(&a,&motor,&control,t,t+100,skipped); /* Duplicate callback. */
        if(kind==16)buddy4_obstruction_sdk_note_update(&a,&motor,&control,t,t+6000,skipped); /* Long update. */
        if(kind==17)a.end_us=t+1001; /* Future update end. */
        assert(!poll().possible_obstruction&&a.observer.output.event_count==0);
        assert(!a.observer.output.valid&&!a.observer.output.established);
    }
    reset();establish();pending();
    assert(!buddy4_obstruction_sdk_poll(&a,&control,&imu,true,t+41000).valid);
    t+=60000;
    for(int i=0;i<30;i++)step(0,0);
    assert(a.observer.output.event_count==0&&!a.observer.output.established);
    establish();pending();assert(step(0,0).possible_obstruction);
    /* Context failure retains a completed event; recovery cannot bypass cooldown. */
    control.running=false;assert(!poll().possible_obstruction);
    assert(a.observer.output.event_count==1);
    control.running=true;++control.started_us;
    for(int i=0;i<27;i++)step(3,3);
    assert(!a.observer.output.established);
    for(int i=0;i<30;i++)assert(!step(0,0).possible_obstruction);
    establish();pending();assert(step(0,0).possible_obstruction);
    assert(a.observer.output.event_count==2);
}
static void phased_20ms_updates(void) {
    reset();
    /* Both sources update every 20 ms, but the motor snapshot sometimes reaches
     * the main-loop poll before that cycle's IMU poll. The previous IMU remains
     * valid/fresh. Polling it again must neither advance nor reset evidence. */
    for(int i=1;i<=200;i++) {
        t=(uint64_t)i*20000;
        motor.left_speed_cm_s=3.18f;motor.right_speed_cm_s=3.09f;
        buddy4_obstruction_sdk_note_update(&a,&motor,&control,t,t+100,skipped);
        uint64_t previous=a.observer.output.observation_us;
        bool delayed=i%2;
        if(!delayed)imu.output.timestamp_us=t+500;
        buddy4_obstruction_output_t out=poll();
        if(delayed) {
            if(i>2)assert(out.observation_us==previous); /* No duplicate evidence. */
            imu.output.timestamp_us=t+2000;
            out=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,t+3000);
        }
        if(i>1)assert(out.valid);
        if(i>=28)assert(out.established);
        assert(!out.possible_obstruction&&out.event_count==0);
    }
    puts("Phased 20 ms motor/IMU updates: established throughout steady 4 s drive after warmup");
}
static void waiting_is_not_evidence(void) {
    reset();establish();pending();
    uint64_t previous_imu=imu.output.timestamp_us;
    uint64_t previous_motor=a.observer.output.observation_us;
    note(0,0);imu.output.timestamp_us=previous_imu;
    buddy4_obstruction_output_t out=poll();
    assert(a.pending&&out.valid&&out.established);
    assert(out.observation_us==previous_motor&&out.slow_ms==80&&out.event_count==0);
    for(uint64_t now=t+1100;now<t+3000;now+=100) {
        out=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,now);
        assert(a.pending&&!out.possible_obstruction&&out.event_count==0);
        assert(out.observation_us==previous_motor&&out.slow_ms==80);
    }
    imu.output.timestamp_us=t+3000;
    out=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,t+4000);
    assert(!a.pending&&out.possible_obstruction&&out.event_count==1);
    assert(out.event_us==t+100); /* Motor observation time is not rewritten. */

    /* The same would-be confirmation must still lose to S while awaiting IMU. */
    reset();establish();pending();previous_imu=imu.output.timestamp_us;
    note(0,0);imu.output.timestamp_us=previous_imu;poll();assert(a.pending);
    control.running=false;
    imu.output.timestamp_us=t+3000;
    out=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,t+4000);
    assert(!out.valid&&!out.established&&!out.possible_obstruction&&out.event_count==0);

    /* No IMU progress: pending evidence expires without confirming. */
    reset();establish();pending();previous_imu=imu.output.timestamp_us;
    note(0,0);imu.output.timestamp_us=previous_imu;poll();
    out=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,t+41001);
    assert(!out.valid&&!out.established&&!out.possible_obstruction&&out.event_count==0);
    /* Fresh IMU data cannot substitute for missing motor updates either. */
    reset();establish();
    for(int i=0;i<3;i++) {
        t+=20000;imu.output.timestamp_us=t+500;out=poll();
        assert(!out.possible_obstruction&&out.event_count==0);
        if(i>0)assert(!out.valid&&!out.established);
    }
    /* A missing motor/IMU pairing may not be overwritten into continuous data. */
    reset();establish();previous_imu=imu.output.timestamp_us;
    note(3,3);imu.output.timestamp_us=previous_imu;poll();assert(a.pending);
    note(3,3); /* Still pending at a second actual motor update. */
    assert(!poll().valid&&!a.observer.output.established);
    assert(a.observer.output.event_count==0);
    /* Known nonfinite encoder input cancels even before a NEW IMU is ready. */
    reset();establish();previous_imu=imu.output.timestamp_us;
    note(NAN,3);imu.output.timestamp_us=previous_imu;
    assert(!poll().valid&&!a.observer.output.established);
}
typedef struct { float pitch,roll,filtered_pitch,filtered_roll,left,right; } drive_row_t;
static const drive_row_t recorded_drive[]={
#include "fixtures/obstruction_drive_20261102_1346.inc"
};
typedef struct { uint64_t sample_us; drive_row_t row; } contact_row_t;
static const contact_row_t recorded_contact[]={
#include "fixtures/obstruction_foam_20261102_1424.inc"
};
static void actual_foam_transition(void) {
    reset();uint64_t event=0;bool established_before_asymmetry=false;
    for(unsigned i=0;i<sizeof(recorded_contact)/sizeof(recorded_contact[0]);i++) {
        const contact_row_t *sample=&recorded_contact[i];const drive_row_t *r=&sample->row;
        t=sample->sample_us-20000; /* Retain recorded gaps, simulate update phase. */
        imu.output.pitch_deg=r->pitch;imu.output.roll_deg=r->roll;
        imu.output.filtered_pitch_deg=r->filtered_pitch;
        imu.output.filtered_roll_deg=r->filtered_roll;
        buddy4_obstruction_output_t out=step(r->left,r->right);
        if(sample->sample_us>=87480295&&sample->sample_us<87600302&&out.established)
            established_before_asymmetry=true;
        if(out.possible_obstruction)event=out.event_us;
    }
    assert(established_before_asymmetry);
    assert(event>=87840000&&event<88224602&&a.observer.output.event_count==1);
    control.running=false;
    assert(!buddy4_obstruction_sdk_poll(&a,&control,&imu,true,88224602).possible_obstruction);
    assert(a.observer.output.event_count==1);
    printf("Actual foam speed/angle replay: possible obstruction at %.6f s before recorded S at 88.224602 s (callback phase simulated)\n",event/1e6);
}
static void recorded_angles_and_cooperative_schedule(void) {
    /* Use the ACTUAL recorded angles/speeds with the real phase-locked scheduler
     * and SDK call order. Loop/I2C/log durations are synthetic: the CSV does not
     * contain every intervening motor callback/main-loop poll. This isolates the
     * eligibility gates; it is not an exact reconstruction of hidden callbacks. */
    unsigned pitch_rejections=0,roll_rejections=0,angle_rejections=0;
    for(unsigned i=0;i<sizeof(recorded_drive)/sizeof(recorded_drive[0]);i++) {
        const drive_row_t *r=&recorded_drive[i];
        bool pitch=fabsf(r->pitch)>3,roll=fabsf(r->roll)>2;
        pitch_rejections+=pitch;roll_rejections+=roll;angle_rejections+=pitch||roll;
        assert(fabsf(r->filtered_pitch)<=3&&fabsf(r->filtered_roll)<=2);
    }
    assert(pitch_rejections==60&&roll_rejections==69&&angle_rejections==108);
    for(int flat_control=0;flat_control<2;flat_control++) {
        reset();
        task_t motor_release={.period=20000},imu_release={.period=20000};
        uint64_t clock=0,previous_observation=0,first_established=0;
        unsigned row=0,repeated=0,established=0;
        while(row<sizeof(recorded_drive)/sizeof(recorded_drive[0])) {
            const drive_row_t *r=&recorded_drive[row];
            uint64_t loop_now=clock;
            clock+=60; /* Safety/status work after main captures its 'now'. */
            if(due(&motor_release,loop_now)) {
                uint64_t begin=clock;clock+=100;
                motor.left_speed_cm_s=r->left;motor.right_speed_cm_s=r->right;
                buddy4_obstruction_sdk_note_update(&a,&motor,&control,begin,clock,motor_release.skipped);
            }
            bool sampled=due(&imu_release,clock);
            if(sampled) {
                imu.output.timestamp_us=clock;
                imu.output.pitch_deg=flat_control?0:r->pitch;
                imu.output.roll_deg=flat_control?0:r->roll;
                imu.output.filtered_pitch_deg=r->filtered_pitch;
                imu.output.filtered_roll_deg=r->filtered_roll;
                clock+=1500; /* Bounded acquisition duration similar to capture. */
            }
            clock+=30; /* Serial/safety decisions always precede assessment. */
            buddy4_obstruction_output_t out=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,clock);
            if(sampled) {
                repeated+=out.observation_us==previous_observation;
                previous_observation=out.observation_us;
                established+=out.established;
                if(out.established&&!first_established)first_established=clock;
                assert(out.event_count==0&&!out.possible_obstruction);
                /* Repeated CSV/getter observations are not new encoder inputs. */
                uint64_t since=a.observer.moving_since_us;
                for(int read=0;read<3;read++) {
                    clock+=10;
                    buddy4_obstruction_output_t again=buddy4_obstruction_sdk_poll(&a,&control,&imu,true,clock);
                    assert(again.observation_us==out.observation_us);
                    assert(again.established==out.established&&again.event_count==0);
                    assert(a.observer.moving_since_us==since);
                }
                row++;clock+=1500+(row%3)*800; /* CSV formatting/USB work. */
            }
            clock+=100;
        }
        if(!flat_control) {
            assert(established>100&&a.observer.output.established);
            printf("Actual normal-driving vectors: first establishment at %.3f s of synthetic replay, %u/230 established rows, no obstruction events\n",first_established/1e6,established);
        } else {
            /* Angle-control TEST INPUT only: same speeds/scheduling/filtered
             * angles, with raw angles set to zero. Finite raw amplitude must
             * not decide filtered-orientation eligibility. */
            assert(established>100&&a.observer.output.established);
            puts("Eligible-angle control with same speeds/scheduling establishes without changing thresholds");
        }
    }
}
static void recorded_ramp(const char *name,unsigned expected_rows,bool held) {
    /* Historical 69-column outputs already include their original calibration.
     * Retain actual sample gaps. Callback/I2C phase is synthetic, not a replay
     * of unlogged task releases. Legacy data lacks modern quality flags and
     * motor terrain-compensation fields: derive only finite/rail/magnitude
     * suitability, leaving motor terrain FLAT to exercise the IMU guard alone. */
    char path[1024];size_t prefix=0;
    for(size_t i=0;__FILE__[i];i++)if(__FILE__[i]=='/'||__FILE__[i]=='\\')prefix=i+1;
    snprintf(path,sizeof(path),"%.*s../evidence/2026-10-28/raw/%s",(int)prefix,__FILE__,name);
    FILE *file=fopen(path,"r");assert(file);
    reset();establish(); /* Stress the ramp against an already armed observer. */
    char line[8192];unsigned rows=0,guarded=0;uint64_t first=0,base=t+20000;
    while(fgets(line,sizeof(line),file)) {
        if(line[0]<'0'||line[0]>'9')continue;
        char *field[70];unsigned n=0;
        for(char *v=strtok(line,",");v&&n<70;v=strtok(NULL,","))field[n++]=v;
        if(n!=69)continue;
        uint64_t stamp=strtoull(field[4],NULL,10);
        if(!rows)first=stamp;
        t=base+(stamp-first)-20000;
        control.running=held?true:atoi(field[28])!=0;
        control.mode=(drive_mode_t)atoi(field[29]);
        motor.state=held?MOTION_STATE_EXECUTING:(motion_state_t)atoi(field[39]);
        motor.fault=held?MOTION_FAULT_NONE:(motion_fault_t)atoi(field[40]);
        imu.output.valid=atoi(field[3])!=0&&atoi(field[2])==0;
        float x=strtof(field[9],NULL),y=strtof(field[10],NULL),z=strtof(field[11],NULL);
        float mag=hypotf(hypotf(x,y),z);
        bool clipped=false;
        for(int i=6;i<=8;i++)if(atoi(field[i])<=INT16_MIN||atoi(field[i])>=INT16_MAX)clipped=true;
        imu.output.acceleration_valid=!clipped&&isfinite(mag);
        imu.output.tilt_valid=imu.output.acceleration_valid&&mag>=0.8f&&mag<=1.2f;
        imu.output.clipped_axes=clipped?7:0;
        imu.output.pitch_deg=strtof(field[12],NULL);imu.output.roll_deg=strtof(field[13],NULL);
        imu.output.filtered_pitch_deg=strtof(field[14],NULL);
        imu.output.filtered_roll_deg=strtof(field[15],NULL);
        imu.output.terrain=atoi(field[16]);imu.output.side=atoi(field[17]);
        buddy4_obstruction_output_t out=step(strtof(field[36],NULL),strtof(field[37],NULL));
        assert(!out.possible_obstruction&&out.event_count==0);
        if(imu.output.terrain||imu.output.side||fabsf(imu.output.filtered_pitch_deg)>3||
           fabsf(imu.output.filtered_roll_deg)>2) {
            assert(!out.valid&&!out.established);guarded++;
        }
        rows++;
    }
    fclose(file);assert(rows==expected_rows&&guarded>0);
    if(held)assert(guarded==rows); /* Artificial continuing demand, recorded held tilt/speeds. */
    printf("Recorded %s: %u rows, %u tilt-guarded, no possible obstruction\n",name,rows,guarded);
}
static void filtered_onset_and_recovery(void) {
    /* Representative sustained steps under the existing alpha=.15 filter:
     * filtered tilt crosses the unchanged exit-angle guard before the 100 ms
     * slowdown could confirm. The terrain/side flags may still be level. */
    const float roll_step[]={0.750f,1.388f,1.929f,2.390f};
    const float pitch_step[]={1.200f,2.220f,3.087f};
    for(int axis=0;axis<2;axis++)for(int sign=-1;sign<=1;sign+=2) {
        reset();establish();
        unsigned n=axis?4:3;
        for(unsigned i=0;i<n;i++) {
            imu.output.pitch_deg=axis?0:sign*8;
            imu.output.roll_deg=axis?sign*5:0;
            imu.output.filtered_pitch_deg=axis?0:sign*pitch_step[i];
            imu.output.filtered_roll_deg=axis?sign*roll_step[i]:0;
            assert(!step(0.1f,0.1f).possible_obstruction);
        }
        assert(!a.observer.output.valid&&!a.observer.output.established);
        for(int i=0;i<20;i++)assert(!step(0,0).possible_obstruction);
        assert(a.observer.output.event_count==0);
        imu.output.pitch_deg=imu.output.roll_deg=0;
        imu.output.filtered_pitch_deg=imu.output.filtered_roll_deg=0;
        for(int i=0;i<15;i++)step(3,3);
        assert(!a.observer.output.established); /* Return to flat still needs full warmup. */
        for(int i=0;i<15;i++)step(3,3);
        assert(a.observer.output.established);
        pending();assert(step(0,0).possible_obstruction);
    }
    reset();establish();pending();
    imu.output.filtered_roll_deg=2.01f;
    assert(!step(0,0).possible_obstruction&&a.observer.output.event_count==0);
    /* Hysteresis state still suppresses even at the angle guard boundary. */
    reset();establish();imu.output.filtered_roll_deg=2;imu.output.side=1;
    assert(!step(0,0).valid);
    reset();establish();imu.output.filtered_pitch_deg=-3;imu.output.terrain=-1;
    assert(!step(0,0).valid);
}
static void reported_contact_confirmation_limits(void) {
    /* Independent synthetic sequences from the operator's summary; actual
     * recorded-vector replays below exercise the separate 20 ms recovery. */
    reset();establish();pending(); /* Five low-speed observations span 80 ms. */
    assert(a.observer.output.established&&a.observer.output.slow_ms==80);
    note(0,0);control.running=false; /* S wins on the would-confirm release. */
    assert(!poll().possible_obstruction&&a.observer.output.event_count==0);
    assert(!a.observer.output.established);

    for(int transient=0;transient<2;transient++) {
        reset();establish();
        /* -4.41 degrees is below terrain's 6-degree entry, but outside the
         * adapter's stricter 3-degree ramp-onset guard. Both wheels still turn. */
        imu.output.filtered_pitch_deg=-4.41f;
        assert(imu.output.terrain==0&&imu.output.side==0);
        assert(!step(2.5f,2.5f).valid);
        assert(!a.observer.output.established);
        for(int i=0;i<25;i++) {
            if(transient)imu.output.filtered_pitch_deg*=0.85f;
            buddy4_obstruction_output_t out=step(0,0);
            assert(!out.established&&!out.possible_obstruction&&out.event_count==0);
        }
        if(transient) {
            /* Level recovery while stationary cannot re-establish prior motion.
             * The normal full 500 ms moving window is required again. */
            assert(fabsf(imu.output.filtered_pitch_deg)<3);
            for(int i=0;i<25;i++)assert(!step(3,3).established);
            assert(step(3,3).established);
            pending();assert(step(0,0).possible_obstruction);
        } else {
            /* The same filtered pitch and speed loss can be a shallow ramp
             * load: bypassing the guard would accept this ambiguous sequence. */
            assert(!a.observer.output.valid);
        }
    }
    /* Isolate the pitch gate: unchanged level input and the same slowdown
     * confirm after the original 100 ms, without changing any thresholds. */
    reset();establish();step(2.5f,2.5f);
    pending();assert(step(0,0).possible_obstruction);
    puts("Reported contact synthetic cases: S cancels 80 ms candidate; prolonged -4.41deg/recovery cancels history; flat 100 ms control confirms");
}
static void new_recorded_contact(const char *name,unsigned expected_rows,bool should_detect) {
    /* Original CSV vectors and sensor gaps, with a synthetic motor/IMU callback
     * phase. Unlogged callbacks and encoder acquisition times are not invented
     * as hardware evidence. Original files remain byte-identical. */
    char path[1024];size_t prefix=0;
    for(size_t i=0;__FILE__[i];i++)if(__FILE__[i]=='/'||__FILE__[i]=='\\')prefix=i+1;
    snprintf(path,sizeof(path),"%.*s../evidence/2026-11-02/%s",(int)prefix,__FILE__,name);
    FILE *file=fopen(path,"r");assert(file);
    reset();char line[8192];unsigned rows=0;uint64_t event=0;
    while(fgets(line,sizeof(line),file)) {
        char *data=strstr(line," -> ");data=data?data+4:line;
        if(*data<'0'||*data>'9')continue;
        char *field[87];unsigned n=0;
        for(char *v=strtok(data,",");v&&n<87;v=strtok(NULL,","))field[n++]=v;
        if(n!=86)continue;
        t=strtoull(field[10],NULL,10)-20000;
        control.running=atoi(field[34])!=0;control.mode=(drive_mode_t)atoi(field[35]);
        motor.state=(motion_state_t)atoi(field[45]);motor.fault=(motion_fault_t)atoi(field[46]);
        motor.terrain=(motion_terrain_t)atoi(field[48]);motor.terrain_speed_scale=strtof(field[49],NULL);
        motor.left_terrain_duty=strtof(field[50],NULL);motor.right_terrain_duty=strtof(field[51],NULL);
        imu.output.valid=atoi(field[3])!=0;imu.output.tilt_valid=atoi(field[4])!=0;
        imu.output.acceleration_valid=atoi(field[5])!=0;imu.output.clipped_axes=(uint8_t)atoi(field[7]);
        imu.output.pitch_deg=strtof(field[18],NULL);imu.output.roll_deg=strtof(field[19],NULL);
        imu.output.filtered_pitch_deg=strtof(field[20],NULL);imu.output.filtered_roll_deg=strtof(field[21],NULL);
        imu.output.terrain=atoi(field[22]);imu.output.side=atoi(field[23]);
        buddy4_obstruction_output_t out=step(strtof(field[42],NULL),strtof(field[43],NULL));
        if(out.possible_obstruction) { assert(control.running);event=out.event_us; }
        rows++;
    }
    fclose(file);assert(rows==expected_rows);
    if(should_detect)assert(event>9180000&&event<9498713&&a.observer.output.event_count==1);
    else assert(!event&&a.observer.output.event_count==0);
    printf("Recorded %s: %u rows, event_us=%llu (callback phase simulated)\n",name,rows,(unsigned long long)event);
}
static void brief_pitch_pause_safety(void) {
    reset();establish();pending();
    imu.output.filtered_pitch_deg=-4.41f;
    assert(!step(0,0).valid&&a.observer.paused);
    assert(!a.observer.output.established&&a.observer.output.slow_ms==0);
    imu.output.filtered_pitch_deg=-2.834f;
    /* The pre-pause 80 ms never contributes: need another full 100 ms. */
    for(int i=0;i<5;i++) {
        buddy4_obstruction_output_t out=step(0,0);
        assert(out.valid&&out.established&&!out.possible_obstruction);
    }
    assert(step(0,0).possible_obstruction);
    assert(a.observer.output.event_count==1);

    reset(); /* A pitch excursion cannot create established history. */
    imu.output.filtered_pitch_deg=-4.41f;step(3,3);
    imu.output.filtered_pitch_deg=0;
    for(int i=0;i<30;i++)assert(!step(0,0).possible_obstruction);
    assert(!a.observer.output.established);

    reset();establish();imu.output.filtered_pitch_deg=4.41f;
    step(0,0);step(0,0);step(0,0); /* Persistent for 40 ms cancels. */
    assert(!a.observer.paused&&!a.observer.output.established);
    imu.output.filtered_pitch_deg=0;
    for(int i=0;i<20;i++)assert(!step(0,0).possible_obstruction);
    assert(!a.observer.output.event_count);

    /* Every hard cancellation still wins while history is quarantined. */
    for(int kind=0;kind<13;kind++) {
        reset();establish();imu.output.filtered_pitch_deg=-4.41f;
        step(0,0);assert(a.observer.paused);
        imu.output.filtered_pitch_deg=0;
        if(kind==0)control.running=false;
        if(kind==1)control.mode=DRIVE_TURN_LEFT;
        if(kind==2)motor.fault=MOTION_FAULT_STALL;
        if(kind==3)++control.started_us;
        if(kind==4)imu.output.valid=false;
        if(kind==5)imu.output.tilt_valid=false;
        if(kind==6)imu.output.acceleration_valid=false;
        if(kind==7)imu.output.clipped_axes=1;
        if(kind==8)imu.output.filtered_roll_deg=2.01f;
        if(kind==9)imu.output.terrain=1;
        if(kind==10)motor.terrain_speed_scale=0.8f;
        if(kind==11)skipped++;
        if(kind==12)t+=40001;
        step(0,0);assert(!a.observer.paused&&!a.observer.output.established);
        for(int i=0;i<10;i++)assert(!step(0,0).possible_obstruction);
        assert(!a.observer.output.event_count);
    }
    /* Pauses and unequal-wheel grace cannot be chained in either order. */
    reset();establish();step(0,3);imu.output.filtered_pitch_deg=4.41f;
    step(0,0);assert(!a.observer.paused&&!a.observer.output.established);
    reset();establish();imu.output.filtered_pitch_deg=4.41f;
    step(0,3);assert(!a.observer.paused&&!a.observer.output.established);
    reset();establish();imu.output.filtered_pitch_deg=4.41f;step(0,0);
    imu.output.filtered_pitch_deg=0;step(0,3);
    assert(!a.observer.output.established&&!a.observer.asymmetric);
    puts("Brief pitch pause: no evidence bridging; full confirmation after recovery; persistent tilt, faults, turns and missing updates cancel");
}
int main(void) {
    brief_pitch_pause_safety();
    new_recorded_contact("foam_contact_1510.txt",773,false);
    new_recorded_contact("foam_contact_1513.txt",669,true);
    new_recorded_contact("normal_driving_1346.txt",996,false);
    reported_contact_confirmation_limits();
    actual_foam_transition();
    recorded_ramp("left_moving_after.txt",458,false);
    recorded_ramp("right_moving_after.txt",394,false);
    recorded_ramp("left_held_after.txt",373,true);
    recorded_ramp("right_held_after.txt",574,true);
    phased_20ms_updates();waiting_is_not_evidence();
    recorded_angles_and_cooperative_schedule();
    filtered_onset_and_recovery();
    ordinary_and_blocked();command_cancellation();ramps_turns_faults();timing_and_invalid();
    puts("Buddy 4 SDK obstruction adapter tests passed: command ordering, timing qualification, terrain/quality exclusions and history");
    return 0;
}
