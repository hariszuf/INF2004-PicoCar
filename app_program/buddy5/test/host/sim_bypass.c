/* Closed-loop host simulation of the whole obstacle bypass: scan core + vehicle_nav + a simple car model.
 *   cd app_program/buddy5/test/host
 *   gcc -std=gnu11 -Wall -Wextra -DB5_FULL_TEST=1 -I../../rtos -I../../vehicle -I../../../buddy2 -I../../../buddy3 -I../../.. \
 *       sim_bypass.c ../../vehicle/vehicle_nav.c ../../rtos/buddy5_core.c -lm -o /tmp/sim && /tmp/sim
 * Two/three boxes: -DBOX2_W=300 -DBOX2_Y=-330 [-DBOX3_W=300 -DBOX3_Y=330] (side by side, same front plane).
 * Extra: -DSIM_BLOCK_UNTIL_DEG=80 -DTSLIP=0.8 tests the sonar-triggered extra turn when the car under-turns.
 * With the real line follower (junction-style hand-back): add -DSIM_REAL_LINE=1 -DLINE_JUNCTION_USE_BARCODE=0 and
 *   ../../../buddy3/line_follow.c ../../../buddy3/junction_map.c to the source list.
 * Wide sonar beam (a flat face answers at its nearest point inside the cone, like a real HC-SR04): -DSIM_BEAM_DEG=15
 * Optional: -DSENSOR_HALF=55 -DMSLIP=0.95 -DTSLIP=1.05 -DBOX_W=350 -DBOX_D=300 -DBOX_Y=0 -DSTART_GAP=220
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vehicle_nav.h"
#include "buddy5_core.h"
#include "buddy5_config.h"
#include "line_sensors.h"
#include "line_config.h"
#include "motion_config.h"

#ifndef BOX_W
#define BOX_W 350.0
#endif
#ifndef BOX_D
#define BOX_D 300.0
#endif
#ifndef BOX_Y
#define BOX_Y 0.0
#endif
#ifndef START_GAP
#define START_GAP 220.0   /* sensor to box front, mm */
#endif
#define SENSOR_AHEAD 80.0 /* axle to IR sensors / ultrasonic, mm */
#ifndef TAPE_HALF
#define TAPE_HALF 9.0       /* 18 mm tape */
#endif
#ifndef SENSOR_HALF
#define SENSOR_HALF 55.0    /* half the spacing of the two straddling IR sensors (-DSENSOR_HALF=.. to change) */
#endif
#define PI 3.14159265358979
#ifndef MSLIP
#define MSLIP 1.0           /* real distance / odometer distance on straight moves (e.g. 0.95) */
#endif
#ifndef TSLIP
#define TSLIP 1.0           /* real turn / commanded turn (e.g. 1.05) */
#endif

/* ---------- car model ---------- */
static double X, Y, TH, REP, DIST_CM;          /* axle position, physical heading (deg), reported heading, odometer */
static enum { IDLE, TURN, MOVE, DRIVE, STOPPING } mode;
static double rem;                              /* remaining deg / mm */
static double dl, dr;                           /* DRIVE wheel speeds cm/s */
static double move_speed = 80.0;                /* mm/s */
static int stop_ms;
static int pending; static int p_type; static double p_val, p_l, p_r;
enum { P_STOP=1, P_TURN, P_MOVE, P_DRIVE };
static double cur_l, cur_r;
static int emergency;

motion_result_t motion_turn_left(float a)  { pending=1; p_type=P_TURN; p_val=a; return MOTION_OK; }
motion_result_t motion_turn_right(float a) { pending=1; p_type=P_TURN; p_val=-a; return MOTION_OK; }
motion_result_t motion_move_forward(float cm) { pending=1; p_type=P_MOVE; p_val=cm*10.0; return MOTION_OK; }
motion_result_t motion_move_backward(float cm) { pending=1; p_type=P_MOVE; p_val=-cm*10.0; return MOTION_OK; }
motion_result_t motion_set_speed(float s) { move_speed=s*10.0; return MOTION_OK; }
motion_result_t motion_drive(float l, float r) { pending=1; p_type=P_DRIVE; p_l=l; p_r=r; return MOTION_OK; }
void motion_stop(void) { pending=1; p_type=P_STOP; }
void motion_emergency_stop(void) { emergency=1; mode=IDLE; pending=0; }
bool motion_is_idle(void) { return mode==IDLE && !pending; }
void motion_get_status(motion_status_t *s)
{
    memset(s,0,sizeof *s);
    s->state=(mode==IDLE)?MOTION_STATE_IDLE:MOTION_STATE_EXECUTING;
    s->left_speed_cm_s=(float)cur_l; s->right_speed_cm_s=(float)cur_r;
    s->distance_cm=(float)DIST_CM; s->heading_deg=(float)REP;
}
static int line_starts;
#ifdef SIM_REAL_LINE
void line_follow_update(void);
/* Real Buddy 3 line follower (junction-style rejoin and centring) on the simulated sensors. */
static uint8_t read_mask(void);
uint8_t line_sensors_read(void) { return read_mask(); }
void line_sensors_init(void) {}
bool motion_get_tilt(float *p,float *r) { *p=*r=0.0f; return true; }
#else
static int s_line_on;
bool line_follow_start(void) { s_line_on=1; line_starts++; return true; }
void line_follow_stop(void) { s_line_on=0; }
#endif

static void physics(double dt)
{
    if(pending) {
        pending=0;
        if(p_type==P_STOP) { if(mode!=IDLE) { mode=STOPPING; stop_ms=100; } }
        else if(p_type==P_TURN) { mode=TURN; rem=p_val; }
        else if(p_type==P_MOVE) { mode=MOVE; rem=p_val; }
        else { mode=DRIVE; dl=p_l; dr=p_r; }
    }
    cur_l=cur_r=0;
    if(mode==TURN) {
        double step=60.0*dt; if(step>fabs(rem)) step=fabs(rem);
        double d=(rem>0?1:-1)*step; TH+=d*TSLIP; REP+=d*MOTION_TURN_SCALE; rem-=d;
        cur_l=-(d>0?1:-1)*10; cur_r=-cur_l;
        if(fabs(rem)<1e-6) { mode=STOPPING; stop_ms=100; }
    } else if(mode==MOVE) {
        double step=move_speed*dt; if(step>fabs(rem)) step=fabs(rem);
        double s=(rem>0?1:-1)*step;
        X+=s*MSLIP*cos(TH*PI/180); Y+=s*MSLIP*sin(TH*PI/180); DIST_CM+=s/10.0; rem-=s;
        cur_l=cur_r=(rem>0?1:-1)*move_speed/10.0;
        if(fabs(rem)<1e-6) { mode=STOPPING; stop_ms=100; }
    } else if(mode==DRIVE) {
        double omega=((dr-dl)/MOTION_WHEEL_BASE_CM)*180.0/PI;   /* deg/s */
        double v=(dl+dr)*0.5*10.0;
        TH+=omega*dt; REP+=omega*dt*MOTION_TURN_SCALE;
        X+=v*dt*cos(TH*PI/180); Y+=v*dt*sin(TH*PI/180); DIST_CM+=v*dt/10.0;
        cur_l=dl; cur_r=dr;
    } else if(mode==STOPPING) {
        stop_ms-=(int)(dt*1000); if(stop_ms<=0) mode=IDLE;
    }
}

/* ---------- world ---------- */
static double bx0, bx1, by0, by1;
static double ray_to_one(double ox,double oy,double ang_deg,int *hit)
{
    double a=ang_deg*PI/180, dx=cos(a), dy=sin(a), tmin=0, tmax=1e9;
    double lo[2]={bx0,by0}, hi[2]={bx1,by1}, o[2]={ox,oy}, d[2]={dx,dy};
    *hit=0;
    for(int i=0;i<2;i++) {
        if(fabs(d[i])<1e-9) { if(o[i]<lo[i]||o[i]>hi[i]) return 0; continue; }
        double t1=(lo[i]-o[i])/d[i], t2=(hi[i]-o[i])/d[i];
        if(t1>t2) { double t=t1; t1=t2; t2=t; }
        if(t1>tmin) { tmin=t1; }
        if(t2<tmax) { tmax=t2; }
        if(tmin>tmax) return 0;
    }
    if(tmin<=0) return 0;
    *hit=1; return tmin;
}
/* Up to two boxes side by side (-DBOX2_W=300 -DBOX2_Y=-330 adds a second one). */
static double BXa[3],BXb[3],BYa[3],BYb[3]; static int NBOX=1;
static double ray_to_box(double ox,double oy,double ang_deg,int *hit)
{
    double best=0; *hit=0;
    for(int k=0;k<NBOX;k++) {
        int h; bx0=BXa[k]; bx1=BXb[k]; by0=BYa[k]; by1=BYb[k];
        double d=ray_to_one(ox,oy,ang_deg,&h);
        if(h && (!*hit || d<best)) { best=d; *hit=1; }
    }
    return best;
}
static void sensor_xy(int side,double *sx,double *sy)
{
    double a=TH*PI/180;
    *sx=X+SENSOR_AHEAD*cos(a)-side*SENSOR_HALF*sin(a);
    *sy=Y+SENSOR_AHEAD*sin(a)+side*SENSOR_HALF*cos(a);
}
static uint8_t read_mask(void)
{
    uint8_t m=0; double x,y;
    sensor_xy(+1,&x,&y); if(fabs(y)<TAPE_HALF) m|=LINE_SENSOR_LEFT;
    sensor_xy(-1,&x,&y); if(fabs(y)<TAPE_HALF) m|=LINE_SENSOR_RIGHT;
#ifdef SIM_EDGE_BLIP
    /* The edge of the white paper strip (105 mm from the line) reads black for a few mm (-DSIM_EDGE_BLIP=4). */
    sensor_xy(+1,&x,&y); if(fabs(fabs(y)-105.0)<SIM_EDGE_BLIP/2.0) m|=LINE_SENSOR_LEFT;
    sensor_xy(-1,&x,&y); if(fabs(fabs(y)-105.0)<SIM_EDGE_BLIP/2.0) m|=LINE_SENSOR_RIGHT;
#endif
    return m;
}
#ifdef SIM_FLAT_SCAN
static int sim_scanning;   /* set while the core is in B5_SCANNING */
#endif
static b5_range_t range_at(double servo_deg)
{
#ifdef SIM_FLAT_SCAN
    /* Worst case seen on the real tubs: during the scan every ray that meets the box answers with the straight-ahead
     * range, so no edge is found and the width comes out far too small (-DSIM_FLAT_SCAN=1). */
    if(sim_scanning && servo_deg!=0) { sim_scanning=0; b5_range_t s=range_at(servo_deg), f=range_at(0); sim_scanning=1;
        return (s.distance_mm==B5_NO_ECHO_MM || f.distance_mm==B5_NO_ECHO_MM) ? s : f; }
#endif
    double ox=X+SENSOR_AHEAD*cos(TH*PI/180), oy=Y+SENSOR_AHEAD*sin(TH*PI/180); int hit;
    double d=ray_to_box(ox,oy,TH+servo_deg,&hit);
#ifdef SIM_BEAM_DEG
    /* The nearest point of each box (the foot of the perpendicular on a face, or a corner) answers when it lies inside
     * the cone, even though the axis of the beam points elsewhere. */
    for(int k=0;k<NBOX;k++) {
        double px=ox<BXa[k]?BXa[k]:ox>BXb[k]?BXb[k]:ox, py=oy<BYa[k]?BYa[k]:oy>BYb[k]?BYb[k]:oy;
        double e=hypot(px-ox,py-oy), be=atan2(py-oy,px-ox)*180/PI-(TH+servo_deg);
        while(be>180) be-=360; while(be<-180) be+=360;
        if(e>1 && fabs(be)<=SIM_BEAM_DEG && (!hit || e<d)) { d=e; hit=1; }
    }
#ifdef SIM_BEAM_DIFFUSE
    /* Rough (non-mirror-like) surface: any part of the box inside the cone answers, not only its nearest point. */
    for(double b=-SIM_BEAM_DEG;b<=SIM_BEAM_DEG;b+=1.0) {
        int h; double an=(TH+servo_deg+b)*PI/180, e=ray_to_box(ox,oy,TH+servo_deg+b,&h);
        if(!h) continue;
        /* ...but not a face met at a grazing angle (more than 50 degrees from square-on): that sound never returns. */
        double hx=ox+e*cos(an), hy=oy+e*sin(an); int ok=0;
        for(int k=0;k<NBOX;k++) {
            if((fabs(hx-BXa[k])<1||fabs(hx-BXb[k])<1) && fabs(cos(an))>=0.643) ok=1;
            if((fabs(hy-BYa[k])<1||fabs(hy-BYb[k])<1) && fabs(sin(an))>=0.643) ok=1;
        }
        if(ok && (!hit || e<d)) { d=e; hit=1; }
    }
#endif
#endif
    /* range_policy.h: an echo nearer than the 90 mm dead zone is reported as "no echo". */
    if(!hit || d>4000.0 || d<90.0) return (b5_range_t){true,B5_NO_ECHO_MM};
    return (b5_range_t){true,(uint32_t)lround(d)};
}
static int collides1(void)
{
    double a=TH*PI/180, c=cos(a), s=sin(a);
#ifndef CAR_FRONT
#define CAR_FRONT 177.5   /* axle to the front / rear of the body, mm (-DCAR_FRONT=108 -DCAR_REAR=120 for a short car) */
#endif
#ifndef CAR_REAR
#define CAR_REAR 177.5
#endif
    for(double u=-CAR_REAR;u<=CAR_FRONT;u+=10) for(double v=-85;v<=85;v+=10) {
        double px=X+u*c-v*s, py=Y+u*s+v*c;
        if(px>bx0&&px<bx1&&py>by0&&py<by1) return 1;
    }
    return 0;
}

static int collides(void)
{
    for(int k=0;k<NBOX;k++) { bx0=BXa[k]; bx1=BXb[k]; by0=BYa[k]; by1=BYb[k]; if(collides1()) return 1; }
    return 0;
}
int main(void)
{
    full_nav_t nav; b5_core_t core; motion_status_t ms; b5_feedback_t fb;
    uint32_t now=1000, delivered=0, last_range=0, servo_set=0; int servo=999;
    int collisions=0, ok=0, last_state=-1;
    double min_gap=1e9;
    bx0=SENSOR_AHEAD+START_GAP; bx1=bx0+BOX_D; by0=BOX_Y-BOX_W/2; by1=BOX_Y+BOX_W/2;
    BXa[0]=bx0; BXb[0]=bx1; BYa[0]=by0; BYb[0]=by1;
#ifdef BOX2_W
#ifndef BOX2_X
#define BOX2_X 0.0   /* second box nearer (-) or further (+) than the first */
#endif
    BXa[1]=bx0+BOX2_X; BXb[1]=bx1+BOX2_X; BYa[1]=BOX2_Y-BOX2_W/2.0; BYb[1]=BOX2_Y+BOX2_W/2.0; NBOX=2;
#endif
#ifdef BOX3_W
    BXa[2]=bx0; BXb[2]=bx1; BYa[2]=BOX3_Y-BOX3_W/2.0; BYb[2]=BOX3_Y+BOX3_W/2.0; NBOX=3;
#endif
    X=Y=TH=REP=DIST_CM=0;
    motion_get_status(&ms); full_nav_init(&nav,&ms); nav.settled=3;
    b5_core_init(&core,now);
    printf("box x %.0f..%.0f  y %.0f..%.0f (sensor starts %.0f from the front)\n",bx0,bx1,by0,by1,(double)START_GAP);
    for(int step=0; step<120*50; step++) {
        double dt=0.02; now+=20;
#ifdef SIM_REAL_LINE
        for(int k=0;k<4;k++) { physics(0.005); if(full_nav_rejoining(&nav) || (line_starts>0 && core.state==B5_MONITORING && !nav.active)) line_follow_update();   /* as main.c */ }
        if(nav.resumed) { line_starts++; nav.resumed=false; }
#else
        physics(dt);
#endif
        motion_get_status(&ms);
        double fwd=range_at(0).distance_mm;
        bool guard=(range_at(0).distance_mm==B5_NO_ECHO_MM) || (int32_t)(fwd-B5_FRONT_OVERHANG_MM)>B5_GUARD_FRONT_MM;
        /* main.c: B5_BYPASS_IGNORE_GUARD during the return */
        if(B5_BYPASS_IGNORE_GUARD && nav.active && (nav.request.kind==B5_REQ_ADVANCE || nav.request.kind==B5_REQ_RECOVER_LINE || nav.request.kind==B5_REQ_BACKUP)) guard=true;
#ifdef SIM_BLOCK_UNTIL_DEG
        /* Test of the "did not turn enough" correction: the sonar keeps seeing something ahead until the car has
         * really turned this far (deg) toward the side. */
        if(nav.active && nav.request.kind==B5_REQ_BYPASS && fabs(TH)<SIM_BLOCK_UNTIL_DEG) guard=false;
#endif
        { int a=b5_core_angle(&core); b5_range_t fr=range_at(a); full_nav_set_front(&nav,true,b5_range_no_echo(&fr)?B5_CLEAR_FRONT_MM:(int32_t)fr.distance_mm-B5_FRONT_OVERHANG_MM,a); }
        /* scan_task.c keeps core.front_clearance_mm fresh while positioning and during the straight pass. */
        if(core.state==B5_PROCESSING || (core.state==B5_AVOIDING && core.phase==2)) {
            b5_range_t r0=range_at(0); core.front_clearance_mm=b5_front_clearance(&r0);
        }
        full_nav_update(&nav,now,&ms,read_mask(),guard);
        full_nav_feedback(&nav,now,&fb);
        int want=b5_core_wants_range(&core); int ang=b5_core_angle(&core);
        if(ang!=servo) { servo=ang; servo_set=now; }
        b5_range_t r; b5_range_t *rp=NULL;
        if(core.state==B5_MONITORING) want=1;
#ifdef SIM_FLAT_SCAN
        sim_scanning=core.state==B5_SCANNING;
#endif
        if(want && now-servo_set>=300 && now-last_range>=100) { r=range_at(servo); rp=&r; last_range=now; }
#ifdef SIM_FLAT_SCAN
        sim_scanning=0;
#endif
        b5_core_update(&core,now,&fb,rp);
        if(core.request.id && core.request.id!=delivered) {
            delivered=core.request.id;
            static const char *k[]={"NONE","STOP","BYPASS","ADVANCE","RECOVER_LINE","RESUME_LINE","BC_L","BC_R","BACKUP","LOOK"};
            printf("[%6.1fs] request %s fwd=%d lat=%d\n",(now-1000)/1000.0,k[core.request.kind],core.request.forward_mm,core.request.lateral_mm);
            if(core.request.kind==B5_REQ_BYPASS) printf("          pivot looks: box out of view at left=%d right=%d deg (0 = not done, 90 = still in view), reason=%d\n",(int)core.probe_left_deg,(int)core.probe_right_deg,(int)core.profile.side_reason);
            if(!full_nav_accept(&nav,&core.request,now)) printf("  nav rejected\n");
        }
        if((int)core.state!=last_state) {
            last_state=core.state;
            printf("[%6.1fs] state=%s fault=%d  pos x=%.0f y=%.0f th=%.0f  width=%d side=%d offset=%d\n",(now-1000)/1000.0,
                   b5_state_name(core.state),core.fault,X,Y,TH,(core.profile.width_valid||core.profile.width_estimated)?core.profile.width_mm:-1,core.profile.side,core.profile.offset_mm);
        }
        if(collides()) { collisions++; }
        if(step%100==0) printf("   t=%5.1f x=%6.0f y=%6.0f th=%6.1f mode=%d\n",(now-1000)/1000.0,X,Y,TH,mode);
        if(core.state==B5_ERROR || nav.failed || emergency) { printf("FAILED: core fault=%d nav.failed=%d\n",core.fault,nav.failed); break; }
        if(line_starts>0 && core.state==B5_MONITORING) {
            /* Keep going for a few seconds: the line follower must now drive on along the tape, tape between the
             * two sensors (neither sensor on black for long). */
            static uint32_t t_ok; static double x_ok, worst; static int black_steps, steps_after;
            if(!ok) { ok=1; t_ok=now; x_ok=X; }
            { double sy=Y+SENSOR_AHEAD*sin(TH*PI/180); if(fabs(sy)>worst) worst=fabs(sy); }
            steps_after++; if(read_mask()) black_steps++;
            if(now-t_ok>=4000u) {
                printf("after the hand-back: followed the line %.0f mm further in 4 s, sensor-pair centre at most %.1f mm off the tape, a sensor on black %d%% of the time\n",
                       X-x_ok,worst,100*black_steps/steps_after);
                break;
            }
        }
    }
    { double sy=Y+SENSOR_AHEAD*sin(TH*PI/180); printf("sensor-pair centre %.1f mm from the tape centre, heading %.1f deg\n",sy,TH); }
    printf("\nturn re-checks=%d  sonar extra turns=%d\n",(int)nav.turn_fixes_total,(int)nav.block_fixes_total);
    printf("\nresult: %s | line follower restarted=%d | collisions(steps)=%d | final x=%.0f y=%.0f heading=%.1f deg\n",
           ok?"BYPASS COMPLETED":"NOT COMPLETED",line_starts,collisions,X,Y,TH);
    (void)min_gap;
    return (ok && !collisions)?0:1;
}
