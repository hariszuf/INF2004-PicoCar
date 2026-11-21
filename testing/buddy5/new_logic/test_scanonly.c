/* Scan-only build: a stationary car measures boxes of different widths. Build: see run_scanonly.sh */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "buddy5_plan.h"
#define D2R 0.017453292519943295
static double ray_box(double ox,double oy,double dx,double dy,double x0,double x1,double y0,double y1) {
 double t0=0,t1=1e9,o[2]={ox,oy},d[2]={dx,dy},lo[2]={x0,y0},hi[2]={x1,y1};
 for(int i=0;i<2;i++) {
  if(fabs(d[i])<1e-12) { if(o[i]<lo[i]||o[i]>hi[i]) return 1e9; }
  else { double a=(lo[i]-o[i])/d[i],b=(hi[i]-o[i])/d[i]; if(a>b){double t=a;a=b;b=t;} if(a>t0)t0=a; if(b<t1)t1=b; if(t0>t1) return 1e9; }
 }
 return t0;
}
static unsigned rs=1; static double rnd(void){ rs=rs*1664525u+1013904223u; return (rs>>8)/16777216.0; }
static int run(double face_x,double cy,double width,double noise,int *requests,int32_t *wout,int *valid_out,int32_t *le,int32_t *re) {
 b5_core_t c; b5_core_init(&c,0); b5_sampler_t sm; b5_sampler_reset(&sm);
 int servo=0; uint32_t settle=300,next=0; *requests=0; int done=0; *valid_out=0;
 for(uint32_t now=0;now<40000;now++) {
  int ang=b5_core_angle(&c); if(ang!=servo){servo=ang;b5_sampler_reset(&sm);settle=now+B5_SERVO_SETTLE_MS;}
  b5_range_t out; bool have=false;
  if(now>=next) { next=now+B5_PING_INTERVAL_MS; if(now>=settle) {
   double sx=B5_AXLE_TO_SENSOR_MM, best=1e9;
   for(double off=-B5_BEAM_HALF_DEG;off<=B5_BEAM_HALF_DEG+1e-9;off+=1.5) {
    double a=(servo+off)*D2R, d=ray_box(sx,0,cos(a),sin(a),face_x,face_x+150,cy-width/2,cy+width/2); if(d<best)best=d; }
   b5_range_t s={true,B5_NO_ECHO_MM}; if(best<1500) s.distance_mm=(uint32_t)(best+(rnd()-0.5)*2*noise);
   if(b5_sampler_push(&sm,&s,&out)) have=true; } }
  b5_feedback_t f={0}; f.valid=true; f.timestamp_ms=now; f.stopped=true;
  b5_core_update(&c,now,&f,have?&out:NULL);
  b5_request_t rq; if(b5_core_peek_request(&c,&rq)) ++*requests;
  if(c.state==B5_ERROR) return -1;
  if(c.profile.valid && c.state==B5_MONITORING) { *wout=c.profile.width_mm; *valid_out=c.profile.width_valid; *le=c.profile.left_edge_mm; *re=c.profile.right_edge_mm; done=1; break; }
 }
 return done?0:-2;
}
int main(void) {
 double widths[]={60,100,140,200,260}; double offs[]={0,-20,25}; int bad=0;
 for(int w=0;w<5;w++) for(int o=0;o<3;o++) {
  int rq,val; int32_t wm=0,le=0,re=0; int r=run(300+B5_AXLE_TO_SENSOR_MM-80+80,offs[o]*-1,widths[w],4,&rq,&wm,&val,&le,&re);
  double err=wm-widths[w];
  printf("true width %3.0f  centre offset %+3.0f -> reported %4d mm (L %4d R %4d) both edges seen=%d  err %+5.0f  requests=%d %s\n",widths[w],offs[o],wm,le,re,val,err,rq,r==0?"":"NO RESULT");
  if(r!=0||rq!=0) bad++;
 }
 return bad?1:0;
}
