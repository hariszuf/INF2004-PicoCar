#include "buddy4_imu.h"
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
typedef struct { uint8_t regs[256],ptr; int calls,fail_at,short_at; bool mismatch; } fake_t;
static int wr(void *ctx,uint8_t addr,const uint8_t *p,size_t n,bool restart) {
 fake_t *f=ctx; assert(addr==0x1D); ++f->calls;
 if(f->calls==f->fail_at) return -1;
 if(f->calls==f->short_at) return 0;
 if(n==1) { assert(restart); f->ptr=p[0]; }
 else { assert(n==2 && !restart); f->regs[p[0]]=p[1]; }
 return (int)n;
}
static int rd(void *ctx,uint8_t addr,uint8_t *p,size_t n,bool restart) {
 fake_t *f=ctx; assert(addr==0x1D && !restart); ++f->calls;
 if(f->calls==f->fail_at) return -1;
 if(f->calls==f->short_at) return (int)n-1;
 if(n==6) assert(f->ptr==0xA8);
 for(size_t i=0;i<n;++i) p[i]=f->regs[(f->ptr&0x7F)+i];
 if(f->mismatch && f->ptr==0x21) p[0]^=1;
 return (int)n;
}
static void setup(buddy4_imu_t *s,fake_t *f) {
 memset(f,0,sizeof(*f)); f->regs[15]=0x49;
 buddy4_init(s,(buddy4_bus_t){f,wr,rd},buddy4_default_config());
 /* Legacy event fixtures use identity acceleration calibration. */
 for(int i=0;i<3;++i) { s->config.accel_bias_g[i]=0; s->config.accel_scale[i]=1; }
 assert(buddy4_configure(s)); f->regs[0x27]=8;
}
static void vector(fake_t *f,float x,float y,float z) {
 float a[]={x,y,z};
 for(int i=0;i<3;++i) {
  int16_t v=(int16_t)lroundf(a[i]/0.000061f);
  f->regs[0x28+2*i]=(uint16_t)v&255; f->regs[0x29+2*i]=(uint16_t)v>>8;
 }
}
static void angle(fake_t *f,float pitch,float roll) {
 float x=sinf((pitch+0.97f)*0.01745329252f), y=sinf((roll+0.61f)*0.01745329252f);
 vector(f,x,y,sqrtf(1-x*x-y*y));
}
static void samples(buddy4_imu_t *s,fake_t *f,uint64_t *t,float p,float r,int n) {
 angle(f,p,r);
 for(int i=0;i<n;++i) { *t+=20000; assert(buddy4_poll(s,*t)==BUDDY4_FRESH); }
}
/* Every poll transfer can abort a confirmed event, including short transfers. */
static void test_error_recovery(void)
{
    for (int transfer = 1; transfer <= 4; ++transfer) {
        for (int short_transfer = 0; short_transfer < 2; ++short_transfer) {
            buddy4_imu_t imu;
            fake_t sensor;
            uint64_t timestamp_us = 0;
            setup(&imu, &sensor);
            samples(&imu, &sensor, &timestamp_us, 0, 12, 20);
            assert(imu.output.event.peak_confirmed);
            float previous_peak = imu.output.event.peak_roll_deg;

            /* Exercise real transient history that must not survive recovery. */
            vector(&sensor, imu.output.accel_g[0] * 1.4f,
                   imu.output.accel_g[1] * 1.4f, imu.output.accel_g[2] * 1.4f);
            timestamp_us += BUDDY4_PERIOD_US;
            assert(buddy4_poll(&imu, timestamp_us) == BUDDY4_FRESH);
            assert(imu.output.transient_event && imu.had_bump);
            assert(!imu.bump_armed && !imu.output.event.side); /* Dynamic input aborts tilt. */

            uint64_t last_sample_us = imu.output.timestamp_us;
            uint32_t errors_before = imu.output.error_count;
            if (short_transfer) {
                sensor.short_at = sensor.calls + transfer;
            } else {
                sensor.fail_at = sensor.calls + transfer;
            }
            timestamp_us += BUDDY4_PERIOD_US;
            assert(buddy4_poll(&imu, timestamp_us) == BUDDY4_IO_ERROR);
            assert(!imu.ready && !imu.output.valid);
            assert(imu.output.timestamp_us == last_sample_us);
            assert(imu.output.error_count == errors_before + 1);
            assert(imu.output.event.side == 0);
            assert(!imu.output.event.peak_confirmed);
            assert(imu.output.event.peak_roll_deg == 0);
            assert(imu.output.event.peak_height_cm == 0);
            assert(!imu.output.event.window_active);
            assert(!imu.output.event_ended);
            assert(imu.output.ended_event.side == 0);
            assert(!imu.output.ended_event.peak_confirmed);
            assert(imu.output.side == 0 && imu.output.terrain == 0);
            assert(!imu.filter_valid && !imu.had_bump && imu.bump_armed);
            assert(imu.last_bump_us == 0 && imu.baseline == 0);
            assert(!imu.output.transient_event);

            /* Waiting for main's retry performs no I/O and adds no errors. */
            int calls_before = sensor.calls;
            assert(buddy4_poll(&imu, timestamp_us) == BUDDY4_NOT_READY);
            assert(sensor.calls == calls_before);
            assert(imu.output.error_count == errors_before + 1);
            assert(!imu.output.event_ended);

            /* A failed retry preserves cumulative errors; a later retry succeeds. */
            sensor.fail_at = sensor.calls + 1;
            sensor.short_at = 0;
            assert(!buddy4_configure(&imu));
            assert(!imu.ready && !imu.output.event_ended);
            assert(imu.output.error_count == errors_before + 2);
            sensor.fail_at = 0;
            assert(buddy4_configure(&imu));
            assert(imu.ready && !imu.filter_valid && !imu.output.valid);
            assert(!imu.had_bump && imu.bump_armed && imu.last_bump_us == 0);
            assert(imu.output.error_count == errors_before + 2);

            /* Same-side lift starts a new event, with no inherited larger peak. */
            samples(&imu, &sensor, &timestamp_us, 0, 6, 1);
            assert(imu.output.event.side == 1);
            assert(imu.output.event.start_us == timestamp_us);
            assert(!imu.output.event_ended && !imu.output.event.peak_confirmed);
            assert(imu.output.event.peak_roll_deg == 0);
            assert(imu.output.event.peak_height_cm == 0);
            assert(fabsf(imu.output.filtered_roll_deg - 6) < 0.01f);
            assert(!imu.output.transient_event && imu.output.transient_g == 0);
            samples(&imu, &sensor, &timestamp_us, 0, 6, 9);
            assert(!imu.output.event.peak_confirmed);
            samples(&imu, &sensor, &timestamp_us, 0, 6, 1);
            assert(imu.output.event.peak_confirmed);
            assert(imu.output.event.peak_roll_deg < previous_peak);
            assert(fabsf(imu.output.event.peak_roll_deg - 6) < 0.01f);
            assert(fabsf(imu.output.event.peak_height_cm -
                         11.3f * sinf(6 * 0.01745329252f)) < 0.01f);
            assert(imu.output.error_count == errors_before + 2);

            /* New transient is allowed even inside the old event's cooldown. */
            vector(&sensor, imu.output.accel_g[0] * 1.4f,
                   imu.output.accel_g[1] * 1.4f, imu.output.accel_g[2] * 1.4f);
            timestamp_us += BUDDY4_PERIOD_US;
            assert(buddy4_poll(&imu, timestamp_us) == BUDDY4_FRESH);
            assert(imu.output.transient_event);
        }
    }
}

static void test_acceleration_calibration(void) {
 buddy4_imu_t s; fake_t f; setup(&s,&f);
 s.config=buddy4_default_config();
 assert(!s.config.mounting_offsets_need_recalibration);
 assert(s.config.pitch_offset_deg==0.97f && s.config.roll_offset_deg==0.61f);
 assert(s.config.pitch_sign==1.0f && s.config.roll_sign==1.0f);
 assert(s.config.centre_spacing_cm==11.3f && s.config.centre_spacing_confirmed);
 assert(buddy4_configure(&s));
 vector(&f,0.25f,-0.4f,1.1f);
 assert(buddy4_poll(&s,20000)==BUDDY4_FRESH);
 const float bias[]={0.03799f,0.03790f,0.08411f};
 const float scale[]={1.01301f,1.03419f,1.01223f};
 float a[3];
 for(int i=0;i<3;++i) {
  assert(s.config.accel_bias_g[i]==bias[i] && s.config.accel_scale[i]==scale[i]);
  int16_t raw=buddy4_decode_signed16(f.regs[0x28+2*i],f.regs[0x29+2*i]);
  assert(s.output.raw[i]==raw);
  assert(s.output.uncorrected_accel_g[i]==raw*0.000061f);
  a[i]=(raw*0.000061f-bias[i])*scale[i];
  assert(fabsf(s.output.accel_g[i]-a[i])<1e-6f);
 }
 float magnitude=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
 assert(fabsf(s.output.magnitude_g-magnitude)<1e-6f);
 assert(fabsf(s.output.pitch_deg-(atan2f(a[0],hypotf(a[1],a[2]))*57.2957795131f-0.97f))<1e-4f);
 assert(fabsf(s.output.roll_deg-(atan2f(a[1],hypotf(a[0],a[2]))*57.2957795131f-0.61f))<1e-4f);
 /* A tilted first sample is never zeroed. */
 assert(s.output.filtered_pitch_deg==s.output.pitch_deg);
 assert(s.output.filtered_roll_deg==s.output.roll_deg);
 vector(&f,0.25f,-0.4f,1.5f);
 assert(buddy4_poll(&s,40000)==BUDDY4_FRESH);
 assert(fabsf(s.output.transient_g-(s.output.magnitude_g-magnitude))<1e-6f);
 assert(s.output.transient_event);
 for(int i=0;i<3;++i) {
  const float invalid[]={0,-1,NAN,INFINITY};
  for(unsigned j=0;j<sizeof(invalid)/sizeof(invalid[0]);++j) {
   s.config=buddy4_default_config(); s.config.accel_scale[i]=invalid[j];
   int calls=f.calls; assert(!buddy4_configure(&s)); assert(f.calls==calls);
  }
  s.config=buddy4_default_config(); s.config.accel_bias_g[i]=NAN;
  assert(!buddy4_configure(&s));
 }
 s.config=buddy4_default_config();
 for(int i=0;i<3;++i) { s.config.accel_bias_g[i]=-0.1f; s.config.accel_scale[i]=0.5f; }
 assert(buddy4_configure(&s)); assert(buddy4_poll(&s,60000)==BUDDY4_FRESH);
 for(int i=0;i<3;++i) assert(fabsf(s.output.accel_g[i]-(s.output.uncorrected_accel_g[i]+0.1f)*0.5f)<1e-6f);
}

static void raw_vector(fake_t *f, int16_t x, int16_t y, int16_t z) {
 int16_t v[]={x,y,z};
 for(int i=0;i<3;++i) { f->regs[0x28+2*i]=(uint16_t)v[i]&255; f->regs[0x29+2*i]=(uint16_t)v[i]>>8; }
}
static void test_measurement_quality(void) {
 buddy4_imu_t s; fake_t f; setup(&s,&f); s.config=buddy4_default_config();
 assert(buddy4_configure(&s));
 raw_vector(&f,INT16_MIN,INT16_MIN,INT16_MIN);
 for(int i=1;i<=60;++i) {
  assert(buddy4_poll(&s,i*20000)==BUDDY4_FRESH);
  assert(s.output.valid && s.ready && !s.output.error_count);
  assert(s.output.quality==BUDDY4_QUALITY_CLIPPED && s.output.clipped_axes==7);
  assert(!s.output.tilt_valid && !s.output.acceleration_valid);
  assert(!s.output.event.side && !s.output.event.peak_confirmed && !s.output.event.median_candidate_valid);
  assert(!s.output.event_ended && !s.output.transient_event && isnan(s.output.wheel_height_cm));
 }
 assert(fabsf(s.output.pitch_deg+35.67f)<0.005f);
 assert(fabsf(s.output.roll_deg+36.14f)<0.005f);
 assert(fabsf(s.output.magnitude_g-3.6249f)<0.0001f);
 /* The old 6.66 cm is reproduced from raw angle, never a usable height now. */
 assert(fabsf(11.3f*sinf(fabsf(s.output.roll_deg)*0.01745329252f)-6.66f)<0.005f);
 assert(s.output.clipped_samples==60 && s.output.unsuitable_samples==60);
 puts("Endpoint: -35.67/-36.14 diagnostic angles, 3.6249 g; fresh acquisition, CLIPPED, no height/peaks");
 /* Clipping is decided before calibration, even when correction makes the
  * magnitude gravity-like. It cannot turn lost range into trustworthy data. */
 for(int i=0;i<3;++i) {s.config.accel_bias_g[i]=0;s.config.accel_scale[i]=0.5f;}
 raw_vector(&f,INT16_MAX,0,0);assert(buddy4_poll(&s,1220000)==BUDDY4_FRESH);
 assert(fabsf(s.output.magnitude_g-1)<0.01f && !s.output.tilt_valid && s.output.clipped_axes==1);
 s.config=buddy4_default_config();
 /* Both signs, each axis; adjacent representable values are not clipped.
  * Near-full-scale non-clipped values remain unsuitable for static gravity. */
 for(int axis=0;axis<3;++axis)for(int sign=0;sign<2;++sign)for(int adjacent=0;adjacent<2;++adjacent) {
  int16_t v[3]={0};v[axis]=sign ? INT16_MAX-adjacent : INT16_MIN+adjacent;
  raw_vector(&f,v[0],v[1],v[2]);
  assert(buddy4_poll(&s,(uint64_t)(62+axis*4+sign*2+adjacent)*20000)==BUDDY4_FRESH);
  assert(s.output.clipped_axes==(adjacent?0:(1<<axis)));
  assert(!s.output.tilt_valid && s.output.acceleration_valid==(bool)adjacent);
 }
 /* Stationary gravity can point in any direction and repeat indefinitely. */
 for(int axis=0;axis<3;++axis)for(int sign=-1;sign<=1;sign+=2) {
  setup(&s,&f);s.config=buddy4_default_config();assert(buddy4_configure(&s));
  float a[3]={0};a[axis]=(float)sign;
  vector(&f,a[0]/s.config.accel_scale[0]+s.config.accel_bias_g[0],
      a[1]/s.config.accel_scale[1]+s.config.accel_bias_g[1],
      a[2]/s.config.accel_scale[2]+s.config.accel_bias_g[2]);
  for(int i=1;i<=40;++i) {
   assert(buddy4_poll(&s,i*20000)==BUDDY4_FRESH);
   assert(s.output.tilt_valid && s.output.acceleration_valid && !s.output.clipped_samples);
  }
 }
 /* Bad input breaks pending AND accepted current-event evidence, preserving
  * completed history. One bad sample and a sustained run have the same rule. */
 for(int kind=0;kind<4;++kind)for(int count=1;count<=20;count+=19) {
  setup(&s,&f);uint64_t t=0;samples(&s,&f,&t,0,-10,30);samples(&s,&f,&t,0,0,80);
  assert(s.output.ended_event.peak_confirmed && s.output.ended_event.median_candidate_valid);
  side_event_t saved=s.output.ended_event;
  samples(&s,&f,&t,0,12,6);assert(!s.output.event.peak_confirmed);
  if(kind==0)raw_vector(&f,INT16_MIN,0,0);
  if(kind==1)vector(&f,0,0,1.4f);
  if(kind==2)vector(&f,0,0,0); /* Undefined gravity direction/freefall. */
  if(kind==3) { vector(&f,0,0,1);s.config.accel_scale[2]=FLT_MAX; }
  for(int i=0;i<count;++i) {
   t+=20000;assert(buddy4_poll(&s,t)==BUDDY4_FRESH);
   assert(!s.output.tilt_valid && !s.output.event.side && !s.output.event_ended);
   assert(memcmp(&saved,&s.output.ended_event,sizeof(saved))==0);
  }
  s.config.accel_scale[2]=1;
  samples(&s,&f,&t,0,6,10);assert(s.output.tilt_valid && !s.output.event.peak_confirmed && !s.output.event.median_candidate_valid);
  samples(&s,&f,&t,0,6,1);assert(s.output.event.peak_confirmed && s.output.event.median_candidate_valid);
  assert(fabsf(s.output.filtered_roll_deg-6)<0.01f);
  assert(memcmp(&saved,&s.output.ended_event,sizeof(saved))==0);
  raw_vector(&f,0,INT16_MAX,0);t+=20000;buddy4_poll(&s,t);
  assert(!s.output.event.side && !s.output.event_ended);
  assert(memcmp(&saved,&s.output.ended_event,sizeof(saved))==0);
 }
 /* Quantized magnitude boundaries, not an angle/axis gate. */
 setup(&s,&f);
 const float mag[]={0.7999f,0.8001f,1.1999f,1.2001f};
 for(int i=0;i<4;++i) { vector(&f,0,0,mag[i]);buddy4_poll(&s,(i+1)*20000);
  assert(s.output.tilt_valid==(i==1||i==2));assert(s.output.acceleration_valid); }
 /* STATUS overrun is expected with 100Hz ODR / 50Hz polling, not clipping. */
 f.regs[0x27]=0xff;vector(&f,0,0,1);buddy4_poll(&s,100000);
 assert(s.output.valid && s.output.tilt_valid && s.output.status_a==0xff);
 s.output.clipped_samples=s.output.unsuitable_samples=UINT32_MAX;
 raw_vector(&f,INT16_MIN,0,0);buddy4_poll(&s,120000);
 assert(s.output.clipped_samples==UINT32_MAX && s.output.unsuitable_samples==UINT32_MAX);
 /* Every checked setting, including identity, is observable and enforced. */
 const int regs[]={15,0x1f,0x20,0x21,0x22,0x23,0x26};
 for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);++i) {
  setup(&s,&f);assert(s.sensor_config.valid && s.sensor_config.who_am_i==0x49);
  f.regs[regs[i]]^=1;assert(!buddy4_verify_config(&s));
  assert(!s.ready && !s.sensor_config.valid && s.output.error_count==1);
 }
}
int main(void) {
 test_measurement_quality();
 test_acceleration_calibration();
 test_error_recovery();
 assert(buddy4_decode_signed16(0,0)==0); assert(buddy4_decode_signed16(255,127)==32767);
 assert(buddy4_decode_signed16(0,128)==-32768); assert(buddy4_decode_signed16(255,255)==-1);
 buddy4_imu_t s; fake_t f; setup(&s,&f);
 assert(f.regs[0x20]==0x6F && f.regs[0x21]==0xC0 && f.regs[0x26]==2);
 assert(f.regs[0x1F]==0 && f.regs[0x22]==0 && f.regs[0x23]==0);
 assert(s.config.centre_spacing_cm==11.3f && s.config.centre_spacing_confirmed);
 int configure_calls=f.calls;
 for(int k=1;k<=configure_calls;++k) {
  for(int short_transfer=0;short_transfer<2;++short_transfer) {
   setup(&s,&f); f.calls=0; f.fail_at=short_transfer?0:k; f.short_at=short_transfer?k:0;
   assert(!buddy4_configure(&s) && s.output.error_count==1 && !s.ready);
  }
 }
 setup(&s,&f); s.config.centre_spacing_cm=0; assert(!buddy4_configure(&s));
 setup(&s,&f); f.regs[15]=0x33; assert(!buddy4_configure(&s));
 setup(&s,&f); f.mismatch=true; assert(!buddy4_configure(&s));
 setup(&s,&f); uint64_t t=0;
 samples(&s,&f,&t,0,0,1); assert(s.output.valid && s.output.side==0 && s.output.terrain==0);
 for(int k=1;k<=4;++k) {
  f.fail_at=f.calls+k; uint32_t errors=s.output.error_count;
  assert(buddy4_poll(&s,t+20000)==BUDDY4_IO_ERROR);
  assert(!s.output.valid && s.output.timestamp_us==t && s.output.error_count==errors+1);
  f.fail_at=0;
  assert(!s.ready);
  assert(buddy4_configure(&s));
 }
 for(int k=1;k<=4;++k) {
  f.short_at=f.calls+k;
  assert(buddy4_poll(&s,t+20000)==BUDDY4_IO_ERROR); f.short_at=0;
  assert(!s.ready);
  assert(buddy4_configure(&s));
 }
 f.regs[0x27]=0; int calls=f.calls;
 assert(buddy4_poll(&s,t+20000)==BUDDY4_STALE && f.calls==calls+2);
 assert(!s.output.valid); f.regs[0x27]=8;
 samples(&s,&f,&t,10,0,40); assert(s.output.terrain==1);
 samples(&s,&f,&t,4,0,60); assert(s.output.terrain==1);
 samples(&s,&f,&t,0,0,60); assert(s.output.terrain==0);
 samples(&s,&f,&t,-10,0,60); assert(s.output.terrain==-1);
 samples(&s,&f,&t,0,0,60); assert(s.output.terrain==0);
 samples(&s,&f,&t,0,-10,60); assert(s.output.side==-1 && s.output.event.peak_confirmed);
 assert(fabsf(s.output.event.peak_height_cm-11.3f*sinf(10*0.01745329252f))<0.05f);
 /* Roll hysteresis holds the side at 3 degrees, exits below 2. */
 samples(&s,&f,&t,0,-3,60); assert(s.output.side==-1);
 samples(&s,&f,&t,0,0,60); assert(s.output.side==0);
 samples(&s,&f,&t,0,10,60); assert(s.output.side==1);
 samples(&s,&f,&t,0,3,60); assert(s.output.side==1);
 samples(&s,&f,&t,0,0,60); assert(s.output.side==0);
 samples(&s,&f,&t,0,-10,60);
 /* Confirmed conservative peak must not grow from one angular spike. */
 float peak=s.output.event.peak_roll_deg;
 samples(&s,&f,&t,0,-30,1); samples(&s,&f,&t,0,-10,5);
 assert(s.output.event.peak_roll_deg==peak);
 bool returned=false; angle(&f,0,0);
 for(int i=0;i<100;++i) { t+=20000; buddy4_poll(&s,t); if(s.output.event_ended && s.output.side==0) { returned=true; assert(s.output.ended_event.peak_confirmed); } }
 assert(returned && s.output.side==0 && s.output.event.side==0);
 bool ended=false; angle(&f,0,10);
 for(int i=0;i<80;++i) { t+=20000; buddy4_poll(&s,t); }
 assert(s.output.event.peak_confirmed);
 angle(&f,0,-10); t+=100000;
 for(int i=0;i<80;++i) { t+=20000; buddy4_poll(&s,t); if(s.output.event_ended && s.output.side==-1) { ended=true; assert(s.output.ended_event.side==1); } }
 assert(ended && s.output.event.side==-1);
 /* New stable window: missing data and long gaps cannot confirm. */
 setup(&s,&f); t=0; samples(&s,&f,&t,0,10,6); assert(!s.output.event.peak_confirmed);
 f.regs[0x27]=0; buddy4_poll(&s,t+20000); f.regs[0x27]=8;
 samples(&s,&f,&t,0,10,6); assert(!s.output.event.peak_confirmed);
 t+=100000; samples(&s,&f,&t,0,10,6); assert(!s.output.event.peak_confirmed);
 samples(&s,&f,&t,0,10,5); assert(s.output.event.peak_confirmed);
 assert(buddy4_poll(&s,t)==BUDDY4_STALE && !s.output.valid);
 /* Exactly 200 ms of consecutive stable data is required. */
 setup(&s,&f); t=0; samples(&s,&f,&t,0,10,10); assert(!s.output.event.peak_confirmed);
 samples(&s,&f,&t,0,10,1); assert(s.output.event.peak_confirmed);
 /* A >1 degree filtered change restarts the candidate window. */
 setup(&s,&f); t=0; samples(&s,&f,&t,0,10,6);
 samples(&s,&f,&t,0,20,1); assert(!s.output.event.peak_confirmed);
 samples(&s,&f,&t,0,20,5); assert(!s.output.event.peak_confirmed);
 setup(&s,&f); t=0; samples(&s,&f,&t,0,0,1);
 vector(&f,0,0,1.4f); t+=20000; buddy4_poll(&s,t); assert(s.output.transient_event);
 t+=20000; buddy4_poll(&s,t); assert(!s.output.transient_event);
 /* Calibration and geometry are caller configuration, without side corrections. */
 setup(&s,&f); s.config.pitch_offset_deg=0; s.config.roll_offset_deg=0;
 s.config.pitch_sign=-1; s.config.roll_sign=-1; s.config.centre_spacing_cm=12;
 s.config.centre_spacing_confirmed=true;
 vector(&f,0,0.5f,sqrtf(0.75f)); buddy4_poll(&s,20000);
 assert(fabsf(s.output.roll_deg+30)<0.01 && fabsf(s.output.wheel_height_cm-6)<0.01);
 assert(s.output.height_geometry_confirmed);
 puts("Buddy 4 IMU tests passed");
 return 0;
}
