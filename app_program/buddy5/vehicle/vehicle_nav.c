#ifdef B5_FULL_TEST
#include "vehicle_nav.h"
#include <math.h>
#include "../../buddy2/motion_config.h"
#include "../../buddy3/line_follow.h"
#include "../../buddy3/line_sensors.h"
#include "../../buddy3/line_config.h"
enum { TURN=1, MOVE, CHECK_LINE, SEARCH, BACK, SWEEP, CHECK_FRONT, REJOIN };
#define NAV_HEADING_ALIGNED_DEG 8.0f
#define NAV_TURN_DEADBAND_DEG 4.0f
static float absolute(float x) { return x<0?-x:x; }
static float corrected_heading(const motion_status_t *m) {
 return (m->heading_deg/MOTION_TURN_SCALE)*B5_TURN_ODOMETRY_CORRECTION;
}
static void fail(full_nav_t *n) { n->rejoining=false; n->search_phase=0; n->failed=true; n->active=false; n->moving=false; n->forward_moving=false; n->motion_seen=false; line_follow_stop(); motion_emergency_stop(); }
void full_nav_init(full_nav_t *n,const motion_status_t *m) {
 *n=(full_nav_t){0}; n->last_distance=m->distance_cm;
 n->last_heading=corrected_heading(m);
 n->heading_origin=n->last_heading; n->observed=true;
}
void full_nav_set_front(full_nav_t *n,bool fresh,int32_t clearance_mm,int angle_deg) { n->front_fresh=fresh; n->front_mm=clearance_mm; n->front_angle=angle_deg; }
/* Return to the line. side=+1: the line is on the left. extend=true: still beside the box, so first reverse
 * a little, face forward again and drive a bit further before facing the line. */
/* Distance driven on after the sensors first see the line, so the axle (the pivot point) is over the line. */
#ifndef B5_LINE_ADVANCE_MM
#define B5_LINE_ADVANCE_MM (LINE_JUNCTION_ADVANCE_CM*10.0f)
#endif
/* back_first: reverse this far (mm) first (the car stopped part-way to the line and goes back out). */
static void build_recover(full_nav_t *n,float side,bool extend,float back_first) {
 unsigned i=0;
 float sweep=absolute(n->y)+back_first+B5_LINE_SWEEP_EXTRA_MM+(extend?0.0f:(float)B5_CHECK_BACK_MM);
 if(back_first>=4.0f) n->actions[i++]=(full_action_t){BACK,back_first};
 if(extend) {
  /* The car is already B5_CHECK_BACK_MM further from the line (first attempt), so it can face forward safely. */
  n->actions[i++]=(full_action_t){TURN,0};
  n->actions[i++]=(full_action_t){MOVE,B5_EXTEND_MM};
 }
 n->actions[i++]=(full_action_t){TURN,side*90.0f};
 /* Facing the line the box may sit inside the sonar's 90 mm dead zone, which reads as "nothing there": step back
  * first so that a box still beside the car gives a real echo. */
 if(!extend) n->actions[i++]=(full_action_t){BACK,B5_CHECK_BACK_MM};
 n->actions[i++]=(full_action_t){CHECK_FRONT,0};
 /* No extra turns once the check is clear: the sonar looks back toward the box (B5_RECOVERY_LOOK_BACK),
  * so "clear" already means the car's tail will swing past the box rear at the line. */
 n->actions[i++]=(full_action_t){SWEEP,sweep};
#if LINE_JUNCTION
 /* Exactly the "+" junction turn (advance, turn, slow search), then centre the tape between the sensors. */
 n->actions[i++]=(full_action_t){REJOIN,-side};
#else
 n->actions[i++]=(full_action_t){MOVE,B5_LINE_ADVANCE_MM};
 n->actions[i++]=(full_action_t){TURN,side*LINE_JUNCTION_PRETURN_MARGIN_DEG};
 n->actions[i++]=(full_action_t){SEARCH,-side};
#endif
 n->count=i; n->index=0; n->issued=false; n->check_since=0;
 n->tape_entered=false; n->tape_measured=false; n->cand_valid=false;
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
 n->request=*r; n->active=true; n->issued=false; n->moving=false; n->forward_moving=false; n->motion_seen=false; n->recovery_stop=false;
 n->turn_tries=0; n->turn_recheck=false; n->fixing=false; n->block_since=0; n->block_fixes=0;
 n->last_turn_dir=r->lateral_mm<0?-1:1;
 n->resumed=false; n->rejoining=false; n->search_phase=0; n->index=n->count=n->search=0; n->started=now; n->check_since=0;
 switch(r->kind) {
 case B5_REQ_STOP: line_follow_stop(); motion_stop(); n->settled=0; n->turn_fixes_total=n->block_fixes_total=0; break;
 case B5_REQ_BYPASS: trip(n,(float)r->lateral_mm,false); break;
 case B5_REQ_ADVANCE:
  if(r->forward_mm<=n->x || absolute(n->heading)>NAV_HEADING_ALIGNED_DEG) { fail(n); return false; }
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
 case B5_REQ_RECOVER_LINE:
  {
   /* Back to the line, mirrored from the "+" junction turn:
    * face the line, sweep until a sensor sees it, drive on to put the axle over it, turn most of the way
    * back to the original heading, then pivot slowly until a sensor sees the line. */
   float side=(n->y>0.0f)?-1.0f:1.0f;   /* +1: line is on the left */
   n->acquired=false; n->search_phase=0; n->retries=0; n->front_ok=false; n->recover_side=side;
   n->sweep_block=false; n->check_y=0;
   build_recover(n,side,false,0.0f);
  }
  break;
 case B5_REQ_RESUME_LINE: break;
 case B5_REQ_BACKUP:
  line_follow_stop();
  n->settled=0; n->count=1;
  n->actions[0]=(full_action_t){BACK,(float)r->forward_mm};
  break;
 case B5_REQ_LOOK:
  /* Pivot on the spot to look past the box (lateral_mm = heading in degrees); the origin is kept. */
  line_follow_stop();
  n->settled=0; n->count=1;
  n->actions[0]=(full_action_t){TURN,(float)r->lateral_mm};
  break;
 case B5_REQ_BARCODE_LEFT:
 case B5_REQ_BARCODE_RIGHT:
  line_follow_stop();
  n->settled=0;
  n->count=1;
  n->actions[0]=(full_action_t){TURN,r->kind==B5_REQ_BARCODE_LEFT?90.0f:-90.0f};
  break;
 default: n->active=false; return false;
 }
 return true;
}
bool full_nav_rejoining(const full_nav_t *n) { return n->active && n->rejoining; }
bool full_nav_barcode_turn_active(const full_nav_t *n) {
 return n->active && (n->request.kind==B5_REQ_BARCODE_LEFT || n->request.kind==B5_REQ_BARCODE_RIGHT);
}
void full_nav_update(full_nav_t *n,uint32_t now,const motion_status_t *m,uint8_t mask,bool guard) {
 float heading=corrected_heading(m);
 float mid=((heading+n->last_heading)*0.5f-n->heading_origin)*0.01745329252f;
 float ds=(m->distance_cm-n->last_distance)*10.0f;
 n->x+=ds*cosf(mid); n->y+=ds*sinf(mid);
 n->last_distance=m->distance_cm; n->last_heading=heading;
 n->heading=heading-n->heading_origin;
 bool still=absolute(m->left_speed_cm_s)<0.5f && absolute(m->right_speed_cm_s)<0.5f;
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
  /* No forward-guard wait here: line following detects (and stops for) anything ahead within B5_STOP_FRONT_MM. */
  if(n->settled>=3 && (guard || B5_BYPASS_IGNORE_GUARD) && (n->line_seen>=3 || n->acquired)) {   /* heading not checked: like the junction, the line follower centres the car */
   if(!line_follow_start()) { fail(n); return; }
   n->completed_id=n->request.id; n->active=false; n->resumed=true;
  }
  return;
 }
 /* During line recovery, stop the lateral sweep as soon as the front IR
  * sensors consistently see the line. Finish by aligning the heading. */
 if(n->request.kind==B5_REQ_RECOVER_LINE && n->issued && n->index<n->count &&
    n->actions[n->index].kind==SWEEP && !n->recovery_stop && !n->sweep_block) {
  /* Sonar watch on the way to the line: an echo that is close and clearly short of the line is the box (or
   * something else in the way). Stop before touching it. */
  if(!n->tape_entered && n->front_fresh && n->front_angle==0 && n->front_mm!=B5_CLEAR_FRONT_MM && n->front_mm>=0) {
   float remain=absolute(n->y+B5_AXLE_TO_SENSOR_MM*sinf(n->heading*0.01745329252f));
   if((float)n->front_mm<remain-B5_CHECK_MARGIN_MM && n->front_mm<=B5_SWEEP_STOP_MM) { motion_stop(); n->sweep_block=true; }
  }
 }
 if(n->sweep_block) {
  if(n->settled<3) return;
  n->sweep_block=false; n->issued=false; n->moving=false; n->forward_moving=false; n->motion_seen=false;
  n->front_ok=false;
  if(n->retries>=B5_MAX_EXTENDS) { fail(n); return; }
  ++n->retries;
  {
   float back=n->check_y-absolute(n->y);
   build_recover(n,n->recover_side,true,back>0.0f?back:0.0f);
  }
  return;
 }
 if(n->request.kind==B5_REQ_RECOVER_LINE && n->issued && n->index<n->count &&
    n->actions[n->index].kind==SWEEP && !n->recovery_stop) {
  /* Drive across the tape and note where the sensors enter and leave it: the middle of the two is the
   * line centre. Stop once past it; the hand-back then drives exactly onto the line. */
  bool black=(mask&(LINE_SENSOR_LEFT|LINE_SENSOR_RIGHT))!=0;
  float sy=n->y+B5_AXLE_TO_SENSOR_MM*sinf(n->heading*0.01745329252f);
  if(black && !n->tape_entered) { n->tape_entered=true; n->tape_enter_y=sy; }
  else if(n->tape_entered && !black && absolute(sy-n->tape_enter_y)<B5_TAPE_MIN_MM) {
   /* Too narrow to be the tape: keep going, but remember it. If the real tape is never found, the candidate
    * nearest to where the line should be is used (the car reverses onto it). */
   float mid=0.5f*(n->tape_enter_y+sy);
   if(!n->cand_valid || absolute(mid)<absolute(n->cand_y)) { n->cand_y=mid; n->cand_valid=true; }
   n->tape_entered=false;
  }
  else if(n->tape_entered && (!black || absolute(sy-n->tape_enter_y)>=B5_TAPE_MAX_MM)) {
   n->tape_line_y=0.5f*(n->tape_enter_y+sy); n->tape_measured=true;
   motion_stop(); n->recovery_stop=true;
  }
 }
 if(n->recovery_stop) {
  if(n->settled<3) return;
  n->issued=false; n->moving=false; n->forward_moving=false;
  n->motion_seen=false; n->recovery_stop=false;
  ++n->index; n->check_since=0;
 }
 if(n->rejoining) {
  /* The line follower runs the junction turn; main.c calls line_follow_update() meanwhile. */
#if LINE_JUNCTION
  int r=line_follow_rejoin_status();
#else
  int r=-1;
#endif
  if(r<0) { fail(n); return; }
  if(r>0 && n->settled>=3 && motion_is_idle()) {
   n->rejoining=false; n->acquired=true; n->moving=false; n->issued=false; ++n->index; n->check_since=0;
  }
  return;
 }
 /* Slow pivot that looks for the line (last step of the recovery). */
 if(n->search_phase && n->index<n->count) {
  if(n->search_phase==1) {
   bool seen=(mask&(LINE_SENSOR_LEFT|LINE_SENSOR_RIGHT))!=0;
   /* Stop when the heading is back to the line direction, or earlier if a sensor already sees the line
    * and the car is nearly aligned (odometry under-reports the turn). */
   /* Same as the "+" junction: stop at the first black and let the line follower centre the car. */
   if(seen || absolute(n->heading)<=2.0f) {
    n->acquired=true; motion_stop(); n->search_phase=2; n->check_since=now;
   } else if((uint32_t)(now-n->check_since)>B5_LINE_SEARCH_MS) { fail(n); }
   return;
  }
  if(n->settled>=3 && motion_is_idle()) {
   n->search_phase=0; n->moving=false; n->issued=false; ++n->index; n->settled=3;
  }
  return;
 }
 if(n->issued) {
  if(!still) n->motion_seen=true;
  if((uint32_t)(now-n->check_since)>20000u) { fail(n); return; }
  /* A brief zero-speed interval during a turn is not a finished turn. */
  if(n->motion_seen && (uint32_t)(now-n->check_since)>=60u &&
     n->settled>=3 && motion_is_idle()) {
   n->issued=false; n->moving=false; n->forward_moving=false; n->motion_seen=false;
   n->check_since=0;
   if(n->fixing) n->fixing=false;   /* extra sonar turn done: look at the same action again */
   else if(n->actions[n->index].kind==TURN && !full_nav_barcode_turn_active(n)) n->turn_recheck=true;   /* verify the heading first */
   else ++n->index;
  } else return;
 }
 if(n->index>=n->count) {
  if(full_nav_barcode_turn_active(n)) {
   if(!guard) return;
   if(!line_follow_start()) { fail(n); return; }
   n->x=n->y=n->heading=0; n->heading_origin=heading;
   n->active=false;
   return;
  }
  if(n->request.kind==B5_REQ_BACKUP) {
   /* Like STOP: the new position becomes the origin for the next scan. */
   n->x=n->y=n->heading=0; n->heading_origin=heading;
   n->completed_id=n->request.id; n->active=false; return;
  }
  bool pose=absolute(n->heading)<=NAV_HEADING_ALIGNED_DEG;
  if(n->request.kind==B5_REQ_RECOVER_LINE) pose=absolute(n->heading)<=30.0f;
  if(n->request.kind==B5_REQ_LOOK) pose=absolute(n->heading-(float)n->request.lateral_mm)<=NAV_HEADING_ALIGNED_DEG+4.0f;
  if(n->request.kind==B5_REQ_BYPASS)
   pose=pose && (n->block_fixes_total>0 || absolute(n->x)<=B5_ADV_TOL_X_MM) && absolute(n->y-n->request.lateral_mm)<=B5_ADV_TOL_Y_MM;
  if(n->request.kind==B5_REQ_ADVANCE)
   pose=pose && absolute(n->x-n->request.forward_mm)<=B5_ADV_TOL_X_MM && absolute(n->y-n->request.lateral_mm)<=B5_ADV_TOL_Y_MM;
  if(n->request.kind==B5_REQ_RECOVER_LINE) pose=pose && (n->line_seen>=3 || n->acquired);
  /* Found by the line sensors (junction-style hand-back): they are the truth, encoder heading drift is not. */
  if(n->request.kind==B5_REQ_RECOVER_LINE && n->acquired) pose=true;
  if(!pose) { fail(n); return; }
  n->completed_id=n->request.id; n->active=false; return;
 }
 if(n->settled<3) return;
 full_action_t a=n->actions[n->index];
if(a.kind==CHECK_FRONT) {
  if(!n->check_since) { n->check_since=now; n->look[0]=n->look[1]=n->look[2]=0; }
  /* Judge only readings taken after the turn has finished: a ping from mid-turn can see the box corner. */
  if((uint32_t)(now-n->check_since)<B5_CHECK_SETTLE_MS) return;
  /* Three looks (core.c): ahead, half back, fully back toward the box. 1 = clear, 2 = blocked. All clear to go. */
  if(n->front_fresh) {
   float to_line=absolute(n->y)-B5_AXLE_TO_SENSOR_MM-B5_CHECK_MARGIN_MM+(n->front_angle?B5_CHECK_BEYOND_MM:0);
   float c=cosf((float)n->front_angle*0.01745329252f);
   uint8_t v=((float)n->front_mm>=to_line/(c>0.5f?c:0.5f))?1u:2u;
   int a=n->front_angle<0?-n->front_angle:n->front_angle;
   unsigned k=a==0?0u:a<=20?1u:2u;
   if(n->look[k]!=2u) n->look[k]=v;
  }
  bool blocked=n->look[0]==2u || n->look[1]==2u || n->look[2]==2u;
  bool done=n->look[0] && n->look[1] && n->look[2];
  if(!blocked && !done) {
   if((uint32_t)(now-n->check_since)<B5_CHECK_SETTLE_MS+4u*B5_LOOK_PERIOD_MS) return;
   /* A look back never came: count it as clear. The look straight ahead is the one that matters: without it
    * nothing is known about the path to the line, so that counts as blocked. */
   if(!n->look[0]) blocked=true;
  }
  if(!blocked) { n->front_ok=true; n->check_y=absolute(n->y); ++n->index; n->check_since=0; return; }
  /* Blocked only by something between the sonar and the line (the sonar is B5_AXLE_TO_SENSOR_MM ahead of
   * the axle). Anything farther is beyond the line and is not the box: no reverse. */
  if(n->retries>=B5_MAX_EXTENDS) { fail(n); return; }
  ++n->retries;
  build_recover(n,n->recover_side,true,0.0f);
  return;
 }
 if(a.kind==CHECK_LINE) {
  if(!n->check_since) n->check_since=now;
  if(n->line_seen>=3 && absolute(n->heading)<=NAV_HEADING_ALIGNED_DEG) { ++n->index; return; }
  if((uint32_t)(now-n->check_since)<300u) return;
  static const float candidates[]={20,-20,40,-40,60,-60};
  if(n->search>=6) { fail(n); return; }
  trip(n,candidates[n->search++],true); return;
 }
 motion_result_t result=MOTION_OK;
 if(a.kind==TURN) {
  bool barcode_turn=full_nav_barcode_turn_active(n);
  if(barcode_turn && !guard) return;
  float error=barcode_turn?a.value:a.value-n->heading;
  if(absolute(error)<=NAV_TURN_DEADBAND_DEG) { ++n->index; n->turn_recheck=false; n->turn_tries=0; return; }
  if(n->turn_recheck) {
   /* The turn finished short of (or past) its target: turn again by the remaining error, a few times at most. */
   n->turn_recheck=false;
   if(n->turn_tries>=B5_TURN_RETRIES) { ++n->index; n->turn_tries=0; return; }
   ++n->turn_tries;
   if(n->turn_fixes_total<255) ++n->turn_fixes_total;
  }
  if(!barcode_turn) n->last_turn_dir=error>0?1:-1;
  /* Convert desired physical rotation back into the under-reporting motion
   * controller's angle domain. */
  error/=B5_TURN_ODOMETRY_CORRECTION;
  result=error>0?motion_turn_left(error):motion_turn_right(-error);
 } else if(a.kind==BACK) {
  if(a.value<4) { ++n->index; return; }
  if(motion_set_speed(6.0f)!=MOTION_OK) { fail(n); return; }
  result=motion_move_backward(a.value/10.0f);
  if(result!=MOTION_OK) { fail(n); return; }
  n->issued=true; n->moving=true; n->forward_moving=false; n->motion_seen=false;
  n->settled=0; n->check_since=now;
  return;
 } else if(a.kind==REJOIN) {
#if LINE_JUNCTION
  float adv=B5_LINE_ADVANCE_MM;
  {
   /* Distance along the heading that puts the axle on the measured line centre. */
   float s=sinf(n->heading*0.01745329252f);
   /* The drive ran out without crossing a full-width tape: fall back to the best short black that was seen. */
   if(!n->tape_measured && n->cand_valid) { n->tape_line_y=n->cand_y; n->tape_measured=true; }
   if(n->tape_measured && absolute(s)>0.5f) adv=(n->tape_line_y-n->y)/s;
   if(adv>B5_LINE_ADVANCE_MM+80.0f) adv=B5_LINE_ADVANCE_MM+80.0f;
   if(adv< -(B5_LINE_SWEEP_EXTRA_MM+150.0f)) adv=-(B5_LINE_SWEEP_EXTRA_MM+150.0f);
  }
  if(!line_follow_rejoin(a.value>0?1:-1,adv/10.0f)) { fail(n); return; }
#else
  fail(n); return;
#endif
  n->rejoining=true; n->moving=true; n->forward_moving=false; n->settled=0; n->check_since=now;
  return;
 } else if(a.kind==SEARCH) {
  float v=B5_LINE_SEARCH_SPEED_CM_S;
  result=a.value>0?motion_drive(-v,v):motion_drive(v,-v);   /* +: pivot left */
  if(result!=MOTION_OK) { fail(n); return; }
  n->search_phase=1; n->moving=true; n->forward_moving=false; n->settled=0; n->check_since=now;
  return;
 } else {
  if(!guard) {
   if(n->request.kind!=B5_REQ_BYPASS) return;   /* only a forward translation needs clear forward sonar */
   /* Something is still in front after the turn: the car did not turn enough. Wait a moment (a passer-by),
    * then turn a little more the same way, up to B5_BLOCK_MAX_FIXES times. */
   if(!n->block_since) n->block_since=now;
   if((uint32_t)(now-n->block_since)<B5_BLOCK_WAIT_MS) return;
   n->block_since=0;
   if(n->block_fixes>=B5_BLOCK_MAX_FIXES) { fail(n); return; }
   ++n->block_fixes;
   if(n->block_fixes_total<255) ++n->block_fixes_total;
   {
    /* Each further attempt turns a bit more: 8, 12, 16, 20 degrees. */
    float extra=(B5_BLOCK_EXTRA_TURN_DEG+4.0f*(float)(n->block_fixes-1))/B5_TURN_ODOMETRY_CORRECTION;
    result=n->last_turn_dir>=0?motion_turn_left(extra):motion_turn_right(extra);
   }
   if(result!=MOTION_OK) { fail(n); return; }
   n->fixing=true; n->issued=true; n->moving=true; n->forward_moving=false; n->motion_seen=false;
   n->settled=0; n->check_since=now;
   return;
  }
  n->block_since=0;
  if(n->request.kind==B5_REQ_BYPASS && a.kind==MOVE) {
   /* Sideways leg: drive whatever is still needed to reach the lateral target along the CURRENT heading (an extra
    * turn, or a stop part-way, changes it). */
   float s=sinf(n->heading*0.01745329252f);
   if(absolute(s)>0.7f) {
    float dist=absolute((float)n->request.lateral_mm-n->y)/absolute(s);
    if(dist<=a.value*1.6f+100.0f) a.value=dist;
   }
  }
  if(a.value<4) { ++n->index; return; }
  if(motion_set_speed(B5_BYPASS_SPEED_CM_S)!=MOTION_OK) { fail(n); return; }
  result=motion_move_forward(a.value/10.0f);
 }
 if(result!=MOTION_OK) { fail(n); return; }
 n->issued=true; n->moving=true; n->forward_moving=(a.kind==MOVE || a.kind==SWEEP); n->motion_seen=false;
 n->settled=0; n->check_since=now;
}
bool full_nav_block_stop(full_nav_t *n) {
 if(!n->active || n->failed || n->request.kind!=B5_REQ_BYPASS || n->block_fixes>=B5_BLOCK_MAX_FIXES) return false;
 motion_stop();
 n->issued=false; n->moving=false; n->forward_moving=false; n->motion_seen=false; n->settled=0; n->block_since=0;
 return true;
}
void full_nav_feedback(const full_nav_t *n,uint32_t timestamp,b5_feedback_t *f) {
 *f=(b5_feedback_t){0}; f->valid=true; f->timestamp_ms=timestamp;
 f->stopped=n->settled>=3 && !n->moving;
 f->heading_aligned=absolute(n->heading)<=NAV_HEADING_ALIGNED_DEG || n->acquired;
 f->forward_mm=(int32_t)lroundf(n->x); f->lateral_mm=(int32_t)lroundf(n->y);
 f->completed_id=n->completed_id; f->request_failed=n->failed;
 f->line_acquired=n->line_seen>=3 || n->acquired;
 f->front_ok=n->front_ok; f->extends=(uint8_t)n->retries;
 f->turn_fixes=n->turn_fixes_total; f->block_fixes=n->block_fixes_total;
}
#endif
