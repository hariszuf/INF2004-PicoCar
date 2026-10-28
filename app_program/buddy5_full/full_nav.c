#ifdef B5_FULL_TEST
#include "full_nav.h"
#include <math.h>
#include "../buddy2/motion_config.h"
#include "../buddy3/line_follow.h"
#include "../buddy3/line_sensors.h"
enum { TURN=1, MOVE, CHECK_LINE };
static float absolute(float x) { return x<0?-x:x; }
static void fail(full_nav_t *n) { n->failed=true; n->active=false; n->moving=false; line_follow_stop(); motion_emergency_stop(); }
void full_nav_init(full_nav_t *n,const motion_status_t *m) {
 *n=(full_nav_t){0}; n->last_distance=m->distance_cm;
 n->last_heading=m->heading_deg/MOTION_TURN_SCALE;
 n->heading_origin=n->last_heading; n->observed=true;
}
static void trip(full_nav_t *n,float target_y,bool check) {
 float delta=target_y-n->y;
 n->index=0; n->count=check?4:3;
 n->actions[0]=(full_action_t){TURN,delta>=0?90.0f:-90.0f};
 n->actions[1]=(full_action_t){MOVE,absolute(delta)};
 n->actions[2]=(full_action_t){TURN,0};
 n->actions[3]=(full_action_t){CHECK_LINE,0};
 n->check_since=0; n->issued=false;
}
bool full_nav_accept(full_nav_t *n,const b5_request_t *r,uint32_t now) {
 if(n->failed || (n->active && r->kind!=B5_REQ_STOP)) return false;
 n->request=*r; n->active=true; n->issued=false; n->moving=false;
 n->resumed=false; n->index=n->count=n->search=0; n->started=now; n->check_since=0;
 switch(r->kind) {
 case B5_REQ_STOP: line_follow_stop(); motion_stop(); n->settled=0; break;
 case B5_REQ_BYPASS: trip(n,(float)r->lateral_mm,false); break;
 case B5_REQ_ADVANCE:
  if(r->forward_mm<=n->x || absolute(n->heading)>4) { fail(n); return false; }
  {
   float dx=(float)r->forward_mm-n->x, dy=(float)r->lateral_mm-n->y;
   /* Small corrections only while alongside the box: a large spin could
    * swing the long rear of the chassis into it. Correct residual error
    * gradually across successive waypoints. */
   float bearing=atan2f(dy,dx)*57.2957795f;
   if(bearing>8) bearing=8;
   if(bearing< -8) bearing=-8;
   n->count=3;
   n->actions[0]=(full_action_t){TURN,bearing};
   n->actions[1]=(full_action_t){MOVE,dx/cosf(bearing*0.01745329252f)};
   n->actions[2]=(full_action_t){TURN,0};
  }
  break;
 case B5_REQ_RECOVER_LINE: trip(n,0,true); break;
 case B5_REQ_RESUME_LINE: break;
 default: n->active=false; return false;
 }
 return true;
}
void full_nav_update(full_nav_t *n,uint32_t now,const motion_status_t *m,uint8_t mask,bool guard) {
 float heading=m->heading_deg/MOTION_TURN_SCALE;
 float mid=((heading+n->last_heading)*0.5f-n->heading_origin)*0.01745329252f;
 float ds=(m->distance_cm-n->last_distance)*10.0f;
 n->x+=ds*cosf(mid); n->y+=ds*sinf(mid);
 n->last_distance=m->distance_cm; n->last_heading=heading;
 n->heading=heading-n->heading_origin;
 bool still=absolute(m->left_speed_cm_s)<0.5f && absolute(m->right_speed_cm_s)<0.5f &&
             (m->state==MOTION_STATE_IDLE || m->state==MOTION_STATE_ERROR);
 if(still) { if(n->settled<3) ++n->settled; } else n->settled=0;
 if(mask&(LINE_SENSOR_LEFT|LINE_SENSOR_RIGHT)) { if(n->line_seen<3) ++n->line_seen; }
 else n->line_seen=0;
 if(!n->active || n->failed) return;
 if(m->fault!=MOTION_FAULT_NONE || (uint32_t)(now-n->started)>90000u) { fail(n); return; }
 if(n->request.kind==B5_REQ_STOP) {
  if(n->settled>=3 && motion_is_idle()) {
   n->x=n->y=n->heading=0; n->heading_origin=heading;
   n->active=false; n->completed_id=n->request.id;
  }
  return;
 }
 if(n->request.kind==B5_REQ_RESUME_LINE) {
  if(n->settled>=3 && guard && n->line_seen>=3 && absolute(n->heading)<=4) {
   if(!line_follow_start()) { fail(n); return; }
   n->completed_id=n->request.id; n->active=false; n->resumed=true;
  }
  return;
 }
 if(n->issued) {
  if((uint32_t)(now-n->check_since)>20000u) { fail(n); return; }
  if((uint32_t)(now-n->check_since)>=60u && motion_is_idle() && n->settled>=3) {
   n->issued=false; n->moving=false; ++n->index; n->check_since=0;
  } else return;
 }
 if(n->index>=n->count) {
  bool pose=absolute(n->heading)<=4;
  if(n->request.kind==B5_REQ_BYPASS)
   pose=pose && absolute(n->x)<=20 && absolute(n->y-n->request.lateral_mm)<=20;
  if(n->request.kind==B5_REQ_ADVANCE)
   pose=pose && absolute(n->x-n->request.forward_mm)<=5 && absolute(n->y-n->request.lateral_mm)<=20;
  if(n->request.kind==B5_REQ_RECOVER_LINE) pose=pose && n->line_seen>=3;
  if(!pose) { fail(n); return; }
  n->completed_id=n->request.id; n->active=false; return;
 }
 if(n->settled<3 || !motion_is_idle()) return;
 full_action_t a=n->actions[n->index];
 if(a.kind==CHECK_LINE) {
  if(!n->check_since) n->check_since=now;
  if(n->line_seen>=3 && absolute(n->heading)<=4) { ++n->index; return; }
  if((uint32_t)(now-n->check_since)<300u) return;
  static const float candidates[]={20,-20,40,-40,60,-60};
  if(n->search>=6) { fail(n); return; }
  trip(n,candidates[n->search++],true); return;
 }
 if(!guard) return; /* do not begin a primitive until forward sonar is ready */
 motion_result_t result=MOTION_OK;
 if(a.kind==TURN) {
  float error=a.value-n->heading;
  if(absolute(error)<=2) { ++n->index; return; }
  result=error>0?motion_turn_left(error):motion_turn_right(-error);
 } else {
  if(a.value<4) { ++n->index; return; }
  if(motion_set_speed(6.0f)!=MOTION_OK) { fail(n); return; }
  result=motion_move_forward(a.value/10.0f);
 }
 if(result!=MOTION_OK) { fail(n); return; }
 n->issued=true; n->moving=true; n->settled=0; n->check_since=now;
}
void full_nav_feedback(const full_nav_t *n,uint32_t timestamp,b5_feedback_t *f) {
 *f=(b5_feedback_t){0}; f->valid=true; f->timestamp_ms=timestamp;
 f->stopped=n->settled>=3 && !n->moving;
 f->heading_aligned=absolute(n->heading)<=4;
 f->forward_mm=(int32_t)lroundf(n->x); f->lateral_mm=(int32_t)lroundf(n->y);
 f->completed_id=n->completed_id; f->request_failed=n->failed;
 f->line_acquired=n->line_seen>=3;
}
#endif