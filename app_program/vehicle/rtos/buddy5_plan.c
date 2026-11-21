#include "buddy5_plan.h"
#include <math.h>
#define DEG2RAD 0.01745329252f
static int32_t i_abs(int32_t v) { return v<0?-v:v; }
static int32_t i_min(int32_t a,int32_t b) { return a<b?a:b; }
static int32_t round_f(float v) { return (int32_t)(v>=0.0f?v+0.5f:v-0.5f); }
/* ---- swept outline test (separating axis on two convex quads) ---- */
typedef struct { float x[4],y[4]; } quad_t;
static bool quads_overlap(const quad_t *a,const quad_t *b) {
 const quad_t *q[2]={a,b};
 for(int k=0;k<2;k++) for(int i=0;i<4;i++) {
  int j=(i+1)&3;
  float ax=-(q[k]->y[j]-q[k]->y[i]), ay=q[k]->x[j]-q[k]->x[i];
  float amin=1e30f,amax=-1e30f,bmin=1e30f,bmax=-1e30f;
  for(int p=0;p<4;p++) {
   float da=ax*a->x[p]+ay*a->y[p], db=ax*b->x[p]+ay*b->y[p];
   if(da<amin) amin=da;
   if(da>amax) amax=da;
   if(db<bmin) bmin=db;
   if(db>bmax) bmax=db;
  }
  if(amax<bmin || bmax<amin) return false;
 }
 return true;
}
bool b5_car_hits_box(float x,float u,float phi_deg,float bx0,float bx1,float bu0,float bu1,float margin) {
 /* Body coordinates: forward bx, left by. The car is at lateral u (away from the line, positive) and turns toward the
  * line (heading rotates toward -u). Mirror symmetry makes left and right identical. */
 float p=phi_deg*DEG2RAD, c=cosf(p), s=sinf(p);
 static const int sgn[4][2]={{1,1},{1,-1},{-1,-1},{-1,1}};
 quad_t car, box;
 for(int i=0;i<4;i++) {
  float bx=sgn[i][0]>0?(float)B5_AXLE_TO_FRONT_MM:-(float)B5_AXLE_TO_REAR_MM;
  float by=sgn[i][1]*(float)B5_CAR_HALF_WIDTH_MM;
  car.x[i]=x+bx*c+by*s;
  car.y[i]=u-bx*s+by*c;
 }
 box.x[0]=bx0-margin; box.y[0]=bu0-margin;
 box.x[1]=bx1+margin; box.y[1]=bu0-margin;
 box.x[2]=bx1+margin; box.y[2]=bu1+margin;
 box.x[3]=bx0-margin; box.y[3]=bu1+margin;
 return quads_overlap(&car,&box);
}
int32_t b5_pass_x(int32_t x_face_mm,int32_t depth_mm,int32_t near_edge_mm,int32_t far_edge_mm,int32_t offset_mm) {
#if B5_PASS_SIMPLE
 (void)depth_mm; (void)near_edge_mm; (void)far_edge_mm; (void)offset_mm;
 return x_face_mm+B5_BOX_LENGTH_MM;   /* distance to the box + box length, nothing else */
#else
 float bx0=(float)x_face_mm, bx1=(float)(x_face_mm+depth_mm);
 float bu0=(float)(-far_edge_mm), bu1=(float)near_edge_mm, m=(float)B5_SWEEP_MARGIN_MM;
 int32_t limit=x_face_mm+depth_mm+B5_AXLE_TO_REAR_MM+B5_AXLE_TO_FRONT_MM+B5_CAR_HALF_WIDTH_MM+200;
 for(int32_t x=x_face_mm;x<=limit;x+=10) {
  bool ok=true;
  /* beside the box, then the 90 degree pivot, then the drive to the line facing it */
  for(int phi=0;phi<=90 && ok;phi+=5) if(b5_car_hits_box((float)x,(float)offset_mm,(float)phi,bx0,bx1,bu0,bu1,m)) ok=false;
  for(int32_t u=offset_mm;u>=0 && ok;u-=20) if(b5_car_hits_box((float)x,(float)u,90.0f,bx0,bx1,bu0,bu1,m)) ok=false;
  /* finally the pivot back to face along the line, on the line (the rear swings through the box's lane) */
  for(int phi=90;phi>=0 && ok;phi-=5) if(b5_car_hits_box((float)x,0.0f,(float)phi,bx0,bx1,bu0,bu1,m)) ok=false;
  if(ok) return x+B5_PASS_EXTRA_MM;
 }
 return limit+B5_PASS_EXTRA_MM;
#endif
}
/* ---- scan analysis ---- */
static int32_t median_i(int32_t *v,int n) {
 for(int i=1;i<n;i++) { int32_t t=v[i]; int j=i-1; while(j>=0 && v[j]>t) { v[j+1]=v[j]; j--; } v[j+1]=t; }
 return v[n/2];
}
typedef enum { RAY_NONE, RAY_BOX, RAY_NEAR, RAY_FAR, RAY_CLEAR } ray_t;
void b5_plan_make(const b5_scan_t *scan,int32_t front_hint_mm,uint8_t blocked_mask,b5_plan_t *out) {
 *out=(b5_plan_t){0};
 /* 1. the box face: median of the straight-ahead cluster (-10..+10 degrees), measured square-on */
 int32_t face[5]; int nf=0;
 for(int i=0;i<B5_SCAN_POINTS;i++) {
  int a=b5_slot_angle(i);
  if(a<-10 || a>10 || scan->state[i]!=B5_SCAN_ECHO) continue;
  face[nf++]=round_f((float)scan->distance[i]*cosf((float)a*DEG2RAD));
 }
 if(nf>=2) { out->face_dist_mm=median_i(face,nf); out->face_valid=true; }
 else if(nf==1) { out->face_dist_mm=face[0]; out->face_valid=true; }
 else if(front_hint_mm>0 && front_hint_mm!=B5_CLEAR_FRONT_MM) {
  /* no echo straight ahead: only trust the stop-time reading if something is still that close */
  out->face_dist_mm=front_hint_mm+B5_FRONT_OVERHANG_MM;
#if B5_SCAN_ONLY
  out->face_valid=true;   /* measuring only: the trigger reading was a real close object, so use it as the face */
#else
  out->face_valid=false;
#endif
 }
 if(!out->face_valid) return;
 /* A "box" must be close: anything farther than the stop distance plus a margin is not what stopped the car. */
 if(out->face_dist_mm>B5_STOP_FRONT_MM+B5_FRONT_OVERHANG_MM+250) { out->face_valid=false; return; }
 out->x_face_mm=B5_AXLE_TO_SENSOR_MM+out->face_dist_mm;
 /* 2. label every ray against the face plane */
 ray_t cls[B5_SCAN_POINTS]; int32_t lat[B5_SCAN_POINTS];
 for(int i=0;i<B5_SCAN_POINTS;i++) {
  float a=(float)b5_slot_angle(i)*DEG2RAD;
  lat[i]=0; cls[i]=RAY_NONE;
  if(scan->state[i]==B5_SCAN_NO_ECHO) { cls[i]=RAY_CLEAR; continue; }
  if(scan->state[i]!=B5_SCAN_ECHO) continue;
  float d=(float)scan->distance[i], expect=(float)out->face_dist_mm/cosf(a);
  lat[i]=round_f(d*sinf(a));   /* + = left of the sonar axis */
  if(d<expect-(float)B5_FACE_TOL_MM) cls[i]=RAY_NEAR;
  else if(d>expect+(float)B5_FACE_TOL_MM) cls[i]=RAY_FAR;
  else cls[i]=RAY_BOX;
 }
 /* 3. per side: box edge (scan edge only ever ENLARGES the specified box) and the room beyond it */
 const int32_t spec_edge=B5_BOX_WIDTH_MM/2+B5_BOX_POS_TOL_MM;
 const int32_t depth=B5_BOX_LENGTH_MM+B5_BOX_DEPTH_TOL_MM;
 for(int s=0;s<2;s++) {
  int dir=s==0?1:-1;       /* slot step toward the left (+) or right (-) */
  int centre=-B5_SCAN_MIN_DEG/5;
  int first_miss=-1;
  for(int i=centre;i>=0 && i<B5_SCAN_POINTS;i+=dir) {
   if(cls[i]==RAY_BOX || cls[i]==RAY_NONE) continue;
   first_miss=i; break;
  }
  int32_t edge=spec_edge;
  out->seen[s]=false;
  out->meas_edge_mm[s]=0;
  {
   /* outermost ray that still hit the box: the box reaches at least that far */
   int last_box=-1;
   for(int i=centre;i>=0 && i<B5_SCAN_POINTS && i!=first_miss;i+=dir) if(cls[i]==RAY_BOX) last_box=i;
   if(last_box>=0) {
    float a_lo=(float)i_abs(b5_slot_angle(last_box))-(float)B5_BEAM_HALF_DEG;
    if(a_lo>0.0f) out->meas_edge_mm[s]=round_f((float)out->face_dist_mm*tanf(a_lo*DEG2RAD));
   }
  }
  if(first_miss>=0) {
   float a_miss=(float)i_abs(b5_slot_angle(first_miss));
   float a_edge=a_miss-(float)B5_BEAM_HALF_DEG;
   if(a_edge>0.0f) {
    int32_t e=round_f((float)out->face_dist_mm*tanf(a_edge*DEG2RAD));
    out->seen[s]=true;
    out->meas_edge_mm[s]=e;
    if(e>edge) edge=e;
   } else out->seen[s]=true;
  }
  out->edge_mm[s]=edge;
  /* room: nearest thing beyond the edge on that side (NEAR rays, or far rays that are not in the box shadow) */
  int32_t room=B5_ROOM_FAR_MM;
  for(int i=0;i<B5_SCAN_POINTS;i++) {
   int a=b5_slot_angle(i);
   if((s==0 && a<=0) || (s==1 && a>=0)) continue;
   if(cls[i]!=RAY_NEAR && cls[i]!=RAY_FAR) continue;
   int32_t u=i_abs(lat[i]);
   int32_t x_hit=B5_AXLE_TO_SENSOR_MM+round_f((float)scan->distance[i]*cosf((float)a*DEG2RAD));
   if(x_hit>out->x_face_mm+depth+B5_AXLE_TO_FRONT_MM+250) continue;   /* beyond the pass: not in the way */
   if(u<=edge) continue;                                              /* inside the box width: not "beside" */
   room=i_min(room,u-edge);
  }
  out->room_mm[s]=room;
 }
 /* 4. feasibility, cost, pass distance per side */
 for(int s=0;s<2;s++) {
  int32_t off=out->edge_mm[s]+B5_CAR_HALF_WIDTH_MM+B5_SIDE_GAP_MM;
  if(off<B5_MIN_BYPASS_OFFSET_MM) off=B5_MIN_BYPASS_OFFSET_MM;
  out->offset_mm[s]=off;
  bool blocked=(blocked_mask&(1u<<s))!=0;
  out->feasible[s]=!blocked && out->room_mm[s]>=B5_NEED_ROOM_MM && off<=B5_MAX_OFFSET_MM;
  int32_t cost=off;
  if(!out->seen[s]) cost+=B5_UNSEEN_EDGE_PENALTY_MM;
  if(out->room_mm[s]<B5_NEED_ROOM_MM+B5_TIGHT_ROOM_EXTRA_MM) cost+=B5_TIGHT_PENALTY_MM;
  out->cost_mm[s]=cost;
  out->pass_x_mm[s]=b5_pass_x(out->x_face_mm,depth,out->edge_mm[s],out->edge_mm[1-s],off);
 }
 /* 5. choose */
 bool fl=out->feasible[0], fr=out->feasible[1];
 if(!fl && !fr) { out->route_ok=false; out->reason=B5_WHY_NO_ROUTE; return; }
 out->route_ok=true;
 if(fl!=fr) { out->side=fl?1:-1; out->reason=B5_WHY_ONLY_FEASIBLE; return; }
 int32_t dc=out->cost_mm[0]-out->cost_mm[1];
 if(i_abs(dc)>=B5_TIE_COST_MM) { out->side=dc<0?1:-1; out->reason=B5_WHY_CHEAPER; return; }
 int32_t dr=out->room_mm[0]-out->room_mm[1];
 if(i_abs(dr)>=B5_TIE_ROOM_MM) { out->side=dr>0?1:-1; out->reason=B5_WHY_TIE_MORE_ROOM; return; }
 out->side=B5_DEFAULT_SIDE>0?1:-1; out->reason=B5_WHY_DEFAULT;
}
