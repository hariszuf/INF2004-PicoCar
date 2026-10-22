/* Reuse calibrated fixtures and run their complete regression suite as well. */
#define main legacy_regression_main
#include "../buddy4/tests/test_imu.c"
#undef main
/* Include implementation to exercise exact filtered-roll window boundaries. */
#include "../buddy4/buddy4_imu.c"
static void feed(side_event_t *e, uint64_t t, float roll) {
 update_median(e,t,fabsf(roll),11.3f);
}
static void test_windows(void) {
 side_event_t e={0};
 for(int i=0;i<10;i++) feed(&e,20000*i,6);
 assert(!e.median_candidate_valid);
 feed(&e,200000,60); /* One isolated spike cannot dominate a median. */
 assert(e.median_candidate_valid && e.median_candidate_roll_deg==6);
 assert(fabsf(e.median_candidate_height_cm-11.3f*sinf(6*DEG_TO_RAD))<1e-6f);
 for(int i=11;i<22;i++) feed(&e,20000*i,8);
 assert(e.median_candidate_roll_deg==8);
 for(int i=22;i<40;i++) feed(&e,20000*i,5);
 assert(e.median_candidate_roll_deg==8); /* Retain highest accepted median. */
 e=(side_event_t){0};
 for(int i=0;i<11;i++) feed(&e,20000*i,4+i);
 assert(e.median_candidate_valid && e.median_candidate_roll_deg==9);
 for(int span=189999;span<=210001;span++) {
  if(span!=189999 && span!=190000 && span!=210000 && span!=210001) continue;
  e=(side_event_t){0};
  for(int i=0;i<11;i++) feed(&e,(uint64_t)span*i/10,6);
  assert(e.median_candidate_valid==(span>=190000 && span<=210000));
 }
 e=(side_event_t){0};
 feed(&e,0,6); feed(&e,40001,6); assert(e.median_count==1);
 feed(&e,40001,6); assert(e.median_count==1);
 feed(&e,39999,6); assert(e.median_count==1);
 e=(side_event_t){0};
 for(int i=0;i<11;i++) feed(&e,40000*i,6);
 assert(!e.median_candidate_valid); /* Adjacent gaps legal, total span too long. */
 e=(side_event_t){0};
 for(int i=0;i<11;i++) feed(&e,(i<5?20000*i:20000*i+20000),6);
 assert(!e.median_candidate_valid); /* Missing one scheduled sample: span 220ms. */
}
static void test_comparison(void) {
 side_event_t stable={0}, changing={0};
 for(int i=0;i<11;i++) {
  uint64_t t=20000*i;
  update_side_event(&stable,1,t,6,11.3f*sinf(6*DEG_TO_RAD));
  update_median(&stable,t,6,11.3f);
  float r=4+i;
  update_side_event(&changing,1,t,r,11.3f*sinf(r*DEG_TO_RAD));
  update_median(&changing,t,r,11.3f);
 }
 assert(stable.peak_confirmed && stable.median_candidate_valid);
 assert(stable.peak_roll_deg==6 && stable.median_candidate_roll_deg==6);
 assert(!changing.peak_confirmed && changing.median_candidate_valid);
 assert(changing.median_candidate_roll_deg==9);
 puts("Synthetic comparison: sustained 6deg stability=6 median=6; ramp 4..14deg stability=unconfirmed median=9");
}
static void test_lifecycle(void) {
 buddy4_imu_t s; fake_t f; setup(&s,&f); uint64_t t=0;
 samples(&s,&f,&t,0,8,11);
 assert(s.output.event.median_candidate_valid);
 float saved=s.output.event.median_candidate_roll_deg;
 /* Stale, repeated and reversed polls restart window, preserve event maximum. */
 f.regs[0x27]=0; assert(buddy4_poll(&s,t+20000)==BUDDY4_STALE);
 assert(s.output.event.median_count==0);
 f.regs[0x27]=8;
 assert(buddy4_poll(&s,t)==BUDDY4_STALE);
 assert(buddy4_poll(&s,t-1)==BUDDY4_STALE);
 assert(s.output.event.median_candidate_roll_deg==saved);
 samples(&s,&f,&t,0,8,10); assert(s.output.event.median_count==10);
 /* A >40ms gap primes a new window, without discarding an accepted maximum. */
 t+=60000; angle(&f,0,8); assert(buddy4_poll(&s,t)==BUDDY4_FRESH);
 assert(s.output.event.median_count==1);
 /* Force a direct side reversal using existing filter gap reset semantics. */
 t+=60000; angle(&f,0,-9); assert(buddy4_poll(&s,t)==BUDDY4_FRESH);
 assert(s.output.event_ended && s.output.ended_event.median_candidate_valid);
 assert(s.output.ended_event.median_candidate_roll_deg==saved);
 assert(s.output.event.side==-1 && s.output.event.median_count==1);
 assert(!s.output.event.median_candidate_valid);
 t+=60000; angle(&f,0,0); assert(buddy4_poll(&s,t)==BUDDY4_FRESH);
 assert(s.output.event_ended && !s.output.ended_event.median_candidate_valid);
 assert(s.output.event.side==0 && s.output.event.median_count==0);
 samples(&s,&f,&t,0,8,40);
 assert(s.output.event.median_candidate_valid);
 t+=60000; angle(&f,0,0); assert(buddy4_poll(&s,t)==BUDDY4_FRESH);
 assert(s.output.ended_event.median_candidate_valid);
 saved=s.output.ended_event.median_candidate_roll_deg;
 samples(&s,&f,&t,0,8,40);
 f.fail_at=f.calls+1; t+=20000;
 assert(buddy4_poll(&s,t)==BUDDY4_IO_ERROR);
 assert(!s.output.event.median_candidate_valid && s.output.event.median_count==0);
 assert(s.output.ended_event.median_candidate_roll_deg==saved);
 f.fail_at=0; assert(buddy4_configure(&s));
 assert(!s.output.event.median_candidate_valid && s.output.event.median_count==0);
 assert(s.output.ended_event.median_candidate_valid);
 assert(s.output.ended_event.median_candidate_roll_deg==saved);
 samples(&s,&f,&t,0,8,10); assert(!s.output.event.median_candidate_valid);
 samples(&s,&f,&t,0,8,1); assert(s.output.event.median_candidate_valid);
 assert(buddy4_configure(&s)); assert(!s.output.event.median_candidate_valid);
 buddy4_init(&s,(buddy4_bus_t){&f,wr,rd},buddy4_default_config());
 assert(!s.output.ended_event.median_candidate_valid);
}
int main(void) {
 assert(legacy_regression_main()==0);
 test_windows(); test_comparison(); test_lifecycle();
 puts("Experimental rolling-median regression tests passed");
 return 0;
}

