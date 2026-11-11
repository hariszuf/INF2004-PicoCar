#ifdef B5_FULL_TEST
#include "buddy5_core.h"
#include <string.h>
#ifndef B5_TIGHT_COURSE
#define B5_TIGHT_COURSE 0
#endif
#define B5_TIGHT_OFFSET_MM 160

/* Q15 trig for 0,5,...60 degrees; no floating point runtime/library needed. */
static const int32_t sn[13] = {0,2856,5690,8481,11207,13848,16384,
                              18795,21063,23170,25102,26842,28378};
static const int32_t cs[13] = {32768,32643,32270,31651,30792,29698,28378,
                              26842,25102,23170,21063,18795,16384};
static int32_t ab(int32_t x) { return x < 0 ? -x : x; }
static int32_t mn(int32_t a, int32_t b) { return a < b ? a : b; }
static int32_t mx(int32_t a, int32_t b) { return a > b ? a : b; }
static bool good(const b5_range_t *r) {
    return r && r->valid && r->distance_mm >= 20u && r->distance_mm <= 4000u;
}
static bool fresh(uint32_t now, const b5_feedback_t *f) {
    return f && f->valid && !f->fault &&
           (uint32_t)(now-f->timestamp_ms) <= B5_FEEDBACK_MAX_AGE_MS;
}
static void state(b5_core_t *c, b5_state_t s, uint32_t now) {
    c->state=s; c->state_since_ms=now;
}
static void request(b5_core_t *c, b5_request_kind_t k, uint32_t now,
                    int32_t x, int32_t y) {
    if (++c->next_request_id == 0u) ++c->next_request_id;
    c->request=(b5_request_t){k,c->next_request_id,y,x};
    c->request_since_ms=now;
}
void b5_core_init(b5_core_t *c, uint32_t now) {
    memset(c,0,sizeof(*c));
    c->front_clearance_mm=-1;
    state(c,B5_MONITORING,now);
}
bool b5_core_reset(b5_core_t *c, uint32_t now, const b5_feedback_t *f) {
    uint32_t seq=c->next_request_id, obstacle=c->profile.obstacle_id;
    if (c->state!=B5_ERROR || !fresh(now,f) || !f->stopped) return false;
    b5_core_init(c,now);
    c->next_request_id=seq; c->profile.obstacle_id=obstacle;
    return true;
}
void b5_core_fail(b5_core_t *c, b5_fault_t why, uint32_t now) {
    if (c->state==B5_ERROR) return; /* latch first fault */
    c->fault=why;
    state(c,B5_ERROR,now);
    request(c,B5_REQ_STOP,now,0,0);
}
const char *b5_state_name(b5_state_t s) {
    static const char * const names[]={"MONITORING","WAIT_STOP","SCANNING",
        "PROCESSING","AVOIDING","LINE_RECOVERY","ERROR"};
    return (unsigned)s < sizeof(names)/sizeof(names[0]) ? names[s] : "UNKNOWN";
}
b5_range_t b5_filter(const b5_range_t samples[B5_SAMPLE_COUNT]) {
    uint32_t v[B5_SAMPLE_COUNT], n=0,clear=0;
    for(unsigned i=0;i<B5_SAMPLE_COUNT;++i) if(b5_range_no_echo(&samples[i])) ++clear;
    if(clear==B5_SAMPLE_COUNT) return (b5_range_t){true,B5_NO_ECHO_MM};
    for (unsigned i=0;i<B5_SAMPLE_COUNT;++i) if (good(&samples[i])) {
        uint32_t d=samples[i].distance_mm, j=n;
        while (j && v[j-1]>d) { v[j]=v[j-1]; --j; }
        v[j]=d; ++n;
    }
    if (n<B5_MIN_VALID_SAMPLES || v[n-1]-v[0]>B5_SAMPLES_MAX_SPREAD_MM)
        return (b5_range_t){false,0};
    /* Lower median for an even count: conservatively retain nearer return. */
    return (b5_range_t){true,v[(n-1u)/2u]};
}
int b5_core_angle(const b5_core_t *c) {
    if (c->state==B5_SCANNING) return (int)c->scan.slot*5+B5_SCAN_MIN_DEG;
    if (c->state==B5_AVOIDING && c->phase==1) return c->profile.side>0 ? B5_SCAN_MIN_DEG : B5_SCAN_MAX_DEG;
#if B5_RECOVERY_LOOK_BACK
    /* Returning to the line: aim the sonar back toward the box so the check after turning sees a box rear
     * that is still level with the car's tail, not only one level with the axle. */
    /* Alternate: straight ahead (box still level with the axle) and back toward the box (box level with the tail). */
    if (c->state==B5_LINE_RECOVERY && c->phase==0) {
        if (c->recover_checked) return 0;   /* driving to the line: watch straight ahead the whole way */
        /* Three looks in turn: ahead, half-way back, fully back toward the box. Together they cover the box
         * from level with the axle to well behind the car's tail. */
        unsigned k=(c->now_ms/B5_LOOK_PERIOD_MS)%3u;
        int far=c->profile.side>0 ? B5_SCAN_MIN_DEG : B5_SCAN_MAX_DEG;
        return k==0u ? 0 : k==1u ? far/2 : far;
    }
#endif
    return 0;
}
bool b5_core_wants_range(const b5_core_t *c) {
    if (c->state==B5_WAIT_STOP && c->probe==2) return true;
    return c->state==B5_MONITORING || c->state==B5_SCANNING ||
           c->state==B5_PROCESSING ||
           (c->state==B5_AVOIDING && c->phase<2);
}
static void scan_start(b5_core_t *c, uint32_t now) {
    memset(&c->scan,0,sizeof(c->scan));
    for (unsigned i=0;i<B5_SCAN_POINTS;i+=3) c->scan.wanted[i]=1;
    c->scan.wanted[B5_SCAN_ZERO_SLOT]=1;
    state(c,B5_SCANNING,now);
}
static bool scan_next(b5_scan_t *s) {
    for (unsigned i=0;i<B5_SCAN_POINTS;++i)
        if (s->wanted[i] && !s->sampled[i]) { s->slot=(uint8_t)i; return true; }
    if (!s->fine_pass) {
        s->fine_pass=true;
        int prev=-1;
        for (int i=0;i<B5_SCAN_POINTS;++i) if(s->sampled[i]) {
            if(prev>=0 && (!s->valid[prev] || !s->valid[i] ||
                ab((int32_t)s->distance[prev]-(int32_t)s->distance[i])>80))
                for(int j=prev+1;j<i;++j) s->wanted[j]=1;
            prev=i;
        }
        return scan_next(s);
    }
    return false;
}
static int32_t isqrt32(int32_t v) { int32_t r=0; if(v<=0) return 0; while((r+1)*(r+1)<=v) ++r; return r; }
/* Half-width on one side when no clear ray was seen there. The outermost echo is either the box face
 * (d = F/cos a, a lower bound) or the box corner caught by the wide sonar beam (d^2 = F^2 + e^2).
 * Either way e = sqrt(d^2 - F^2) is a usable estimate that never exceeds the real half-width. */
static int32_t side_extent(const b5_scan_t *s,int32_t front,bool left) {
    int i0=left?B5_SCAN_POINTS-1:0, step=left?-1:1;
    for (int i=i0; i!=B5_SCAN_ZERO_SLOT; i+=step)
        if (s->sampled[i] && s->valid[i]==B5_SCAN_ECHO && s->distance[i]>front)
            return isqrt32((int32_t)s->distance[i]*s->distance[i]-front*front);
    return 0;
}
static int32_t projected_x(const b5_scan_t *s, int slot) {
    return (int32_t)s->distance[slot]*cs[ab(slot-B5_SCAN_ZERO_SLOT)]/32768;
}
static int32_t plane_y(int32_t front, int slot) {
    int k=slot-B5_SCAN_ZERO_SLOT, a=ab(k);
    int32_t y=front*sn[a]/cs[a];
    return k<0 ? -y : y;
}
/* What one scan ray says about the box front, 0 = clear beside the box, 1 = on the box front, 2 = something nearer
 * than the box front. The sonar beam is wide: aimed at an angle at a flat face it often reports the nearest point of
 * that face (range ~ front) and not the point on its axis (range = front/cos a). Both count as the box front, so any
 * range from front-80 up to (front+80)/cos a is a hit. Judging by d*cos a alone called such a ray a nearer object
 * as soon as the car stood more than ~440 mm away, which is exactly where it stands after a rescan reverse. */
static int ray_class(const b5_scan_t *s,int i,int32_t front) {
    if (s->valid[i]!=B5_SCAN_ECHO) return 0;
    if (projected_x(s,i)>front+80) return 0;
    return (int32_t)s->distance[i]<front-80 ? 2 : 1;
}
/* Sideways offset (signed) needed to pass the box on the given side: edge + half car + clearance, but at least the
 * minimum bypass offset and enough room to pivot beside the box. */
static int32_t planned_offset(const b5_profile_t *p, int side) {
    int32_t edge=side>0 ? p->left_edge_mm : -p->right_edge_mm;
    int32_t mag=(p->width_mm>0 ? p->width_mm : 0)+B5_SIDE_EXTRA_MM;   /* box size + a few cm */
    int32_t need=edge+B5_PIVOT_CLEAR_MM+((side>0 ? p->left_measured : p->right_measured) ? 0 : B5_UNSEEN_EDGE_EXTRA_MM);
    if (need>B5_MAX_OFFSET_MM) need=B5_MAX_OFFSET_MM;
    if (mag<B5_MIN_BYPASS_OFFSET_MM) mag=B5_MIN_BYPASS_OFFSET_MM;
    if (mag<need) mag=need;
    return side>0 ? mag : -mag;
}
/* A local, approximately vertical-front box profile. Unknown sectors invalidate
 * edge estimates. These rays propose a side; they cannot certify swept volume. */
static bool process(b5_core_t *c) {
    b5_scan_t *s=&c->scan;
    b5_profile_t *p=&c->profile;
    int lo=B5_SCAN_ZERO_SLOT,hi=B5_SCAN_ZERO_SLOT,left_clear=-1,right_clear=-1;
    bool left_near=false,right_near=false;
    c->edges_seen=false;
    if (s->valid[B5_SCAN_ZERO_SLOT]!=B5_SCAN_ECHO) return false;
    p->front_mm=s->distance[B5_SCAN_ZERO_SLOT];
    for (int i=B5_SCAN_ZERO_SLOT+1;i<B5_SCAN_POINTS;++i) if (s->sampled[i]) {
        /* An isolated noisy ray must not discard valid hit/clear brackets
         * established by the neighbouring scan angles. */
        if (!s->valid[i]) continue;
        int k=ray_class(s,i,p->front_mm);
        if (!k) { left_clear=i; break; }
        if (k==2) left_near=true;   /* blocked nearer on this side: counts as box, and the other side is preferred */
        hi=i;
    }
    for (int i=B5_SCAN_ZERO_SLOT-1;i>=0;--i) if (s->sampled[i]) {
        if (!s->valid[i]) continue;
        int k=ray_class(s,i,p->front_mm);
        if (!k) { right_clear=i; break; }
        if (k==2) right_near=true;
        lo=i;
    }
    if (left_clear<0 || right_clear<0 || hi==B5_SCAN_POINTS-1 || lo==0) {
        /* The obstacle fills the scan window. Use the assignment's maximum
         * expected 200 mm width; the live forward guard still vetoes a
         * blocked bypass direction before translation begins. */
        int32_t half=100;
#if B5_SIMPLE_BYPASS
        /* Estimate each side that has no clear ray from its outermost echo (see side_extent). */
        (void)half;
        p->left_edge_mm=left_clear>=0 ? plane_y(p->front_mm,left_clear) : side_extent(s,p->front_mm,true);
        p->right_edge_mm=right_clear>=0 ? plane_y(p->front_mm,right_clear) : -side_extent(s,p->front_mm,false);
        /* An unseen edge is at least half-way to the outermost ray that still met the box (the wide beam makes
         * the range-based estimate above collapse toward zero on a flat face). */
        if (left_clear<0) p->left_edge_mm=mx(p->left_edge_mm,plane_y(p->front_mm,hi)/2);
        if (right_clear<0) p->right_edge_mm=mn(p->right_edge_mm,plane_y(p->front_mm,lo)/2);
        p->left_inner_mm=p->left_edge_mm;
        p->right_inner_mm=p->right_edge_mm;
        p->left_measured=left_clear>=0; p->right_measured=right_clear>=0;
        /* Reported edges: midway between the last hit ray and the first clear ray (the outer edge overstates by up to
         * half a scan step); an unseen edge keeps its lower-bound estimate. Planning below still uses the outer edge. */
        p->left_report_mm=left_clear>=0 ? (plane_y(p->front_mm,left_clear)+plane_y(p->front_mm,hi))/2 : p->left_edge_mm;
        p->right_report_mm=right_clear>=0 ? (plane_y(p->front_mm,right_clear)+plane_y(p->front_mm,lo))/2 : p->right_edge_mm;
        p->width_mm=p->left_report_mm-p->right_report_mm;
        p->width_valid=false; p->width_estimated=p->width_mm>0;
#else
        if (left_clear>=0) {
            p->left_edge_mm=plane_y(p->front_mm,left_clear);
            p->right_edge_mm=p->left_edge_mm-2*half;
        } else if (right_clear>=0) {
            p->right_edge_mm=plane_y(p->front_mm,right_clear);
            p->left_edge_mm=p->right_edge_mm+2*half;
        } else {
            p->left_edge_mm=half; p->right_edge_mm=-half;
        }
        p->left_inner_mm=p->left_edge_mm;
        p->right_inner_mm=p->right_edge_mm;
        p->width_mm=2*half; p->width_valid=true;   /* a guess, not a measurement */
#endif
        int32_t extra=B5_CAR_WIDTH_MM/2+B5_CLEARANCE_MM+B5_EDGE_ALLOWANCE_MM;
        int32_t left=p->left_edge_mm+extra;
        int32_t right=p->right_edge_mm-extra;
        if (left_near!=right_near) { p->side=left_near ? -1 : 1; p->side_reason=B5_WHY_NEARER_OBJECT; }
        else if (left_clear>=0 && right_clear<0) { p->side=1; p->side_reason=B5_WHY_LEFT_EDGE_ONLY; }
        else if (right_clear>=0 && left_clear<0) { p->side=-1; p->side_reason=B5_WHY_RIGHT_EDGE_ONLY; }
        else {
            /* Neither edge is inside the scan window (or both are, at its very edge). Compare how far the two outermost
             * rays reach ALONG the box front (d*cos a). The raw slant range cannot be compared: the window is wider
             * to the right (-35 deg) than to the left (+25 deg), so a ray on the right is always longer. */
            int32_t lx=s->valid[B5_SCAN_POINTS-1]==B5_SCAN_NO_ECHO ? 4001 : s->valid[B5_SCAN_POINTS-1]==B5_SCAN_ECHO ? projected_x(s,B5_SCAN_POINTS-1) : p->front_mm;
            int32_t rx=s->valid[0]==B5_SCAN_NO_ECHO ? 4001 : s->valid[0]==B5_SCAN_ECHO ? projected_x(s,0) : p->front_mm;
            if (ab(lx-rx)>=B5_SIDE_PREFER_MM) { p->side=lx>rx ? 1 : -1; p->side_reason=B5_WHY_FAR_RAY; }
            /* No information either way. The right half of the window is wider (-35 deg), so the right-hand estimate is
             * the larger lower bound and that side gets the bigger, safer offset: keep the previous default. */
            else { p->side=-1; p->side_reason=B5_WHY_NO_EDGES; }   /* nothing to go on from the scan; the pivot looks decide (probe_step) */
        }
        p->offset_mm=p->side>0 ? left : right;
        /* The edge here is only an estimate, so an over-long one is capped and not treated as "no route": the
         * forward check before the straight pass decides whether the side is really free. */
        if (p->offset_mm>B5_MAX_OFFSET_MM) p->offset_mm=B5_MAX_OFFSET_MM;
        if (p->offset_mm<-B5_MAX_OFFSET_MM) p->offset_mm=-B5_MAX_OFFSET_MM;
        return true;
    }
    /* Outer edge of observed hit/clear angular bracket, enclosing estimate. */
    c->edges_seen=true;
    p->left_edge_mm=plane_y(p->front_mm,left_clear);
    p->right_edge_mm=plane_y(p->front_mm,right_clear);
    p->left_inner_mm=plane_y(p->front_mm,hi);
    p->right_inner_mm=plane_y(p->front_mm,lo);
    p->left_measured=true; p->right_measured=true;
    p->left_report_mm=(plane_y(p->front_mm,left_clear)+plane_y(p->front_mm,hi))/2;
    p->right_report_mm=(plane_y(p->front_mm,right_clear)+plane_y(p->front_mm,lo))/2;
    p->width_mm=p->left_report_mm-p->right_report_mm;   /* bracket midpoints: more accurate than the outer edges */
    p->width_valid=p->width_mm>0;
    int32_t extra=B5_CAR_WIDTH_MM/2+B5_CLEARANCE_MM+B5_EDGE_ALLOWANCE_MM;
    int32_t left=p->left_edge_mm+extra, right=p->right_edge_mm-extra;
    int32_t left_score=4001,right_score=4001;
    for (int i=0;i<B5_SCAN_POINTS;++i) if (s->sampled[i]) {
        if (!s->valid[i]) continue;
        if (i>=left_clear)
            left_score=mn(left_score,s->valid[i]==B5_SCAN_NO_ECHO ? 4001 : s->distance[i]);
        if (i<=right_clear)
            right_score=mn(right_score,s->valid[i]==B5_SCAN_NO_ECHO ? 4001 : s->distance[i]);
    }
    bool l=p->width_valid && left<=B5_MAX_OFFSET_MM &&
           left_score+B5_ROUTE_UNCERTAINTY_MM>left+B5_CAR_LENGTH_MM+B5_CLEARANCE_MM;
    bool r=p->width_valid && -right<=B5_MAX_OFFSET_MM &&
           right_score+B5_ROUTE_UNCERTAINTY_MM>-right+B5_CAR_LENGTH_MM+B5_CLEARANCE_MM;
    if (!l && !r) return false;
    if (l && r && left_near!=right_near) { p->side=left_near ? -1 : 1; p->side_reason=B5_WHY_NEARER_OBJECT; }
    else if (l && r) {
        /* Take the side with the shorter detour: the box edge nearer to the car's path. (Before, the side whose
         * far echo was longer won, which has nothing to do with where the box is.) */
        int32_t lo_d=left, ro_d=-right;
        if (ab(lo_d-ro_d)>=B5_SIDE_PREFER_MM) { p->side=lo_d<ro_d ? 1 : -1; p->side_reason=B5_WHY_SHORTER_DETOUR; }
        else if (left_score!=right_score) { p->side=left_score>right_score ? 1 : -1; p->side_reason=B5_WHY_MORE_ROOM; }
        else { p->side=lo_d<=ro_d ? 1 : -1; p->side_reason=B5_WHY_SHORTER_DETOUR; }
    } else { p->side=l ? 1 : -1; p->side_reason=B5_WHY_ONLY_ROOM; }
    p->offset_mm=p->side>0 ? left : right;
    return true;
}
static void advance(b5_core_t *c,uint32_t now,const b5_feedback_t *f) {
    c->advance_start_x=f->forward_mm;
    c->phase=2;
    request(c,B5_REQ_ADVANCE,now,f->forward_mm+B5_STEP_MM,c->bypass_y);
}
static void side_sample(b5_core_t *c,uint32_t now,
                        const b5_feedback_t *f,const b5_range_t *r) {
    int32_t edge=c->profile.side>0 ? c->profile.left_edge_mm : c->profile.right_edge_mm;
    int32_t lateral=ab(c->bypass_y-edge);
    int angle=ab(c->profile.side>0 ? B5_SCAN_MIN_DEG : B5_SCAN_MAX_DEG)/5;
    int32_t inner=c->profile.side>0 ? c->profile.left_inner_mm : c->profile.right_inner_mm;
    int32_t expected=lateral*32768/sn[angle];
    int32_t upper=ab(c->bypass_y-inner)*32768/sn[angle];
    bool no_echo=b5_range_no_echo(r);
    int32_t d=no_echo?0:(int32_t)r->distance_mm;
    int32_t wall=c->side_seen ? c->wall_range_mm : upper;
    int32_t cross_x=f->forward_mm+wall*cs[angle]/32768;
    int32_t observed_x=f->forward_mm+d*cs[angle]/32768;
    int32_t observed_y=f->lateral_mm-c->profile.side*d*sn[angle]/32768;
    /* With a narrow forward-oblique view, the ray sees the FRONT face
     * before it reaches the side wall. Do not mistake that for a far edge. */
    if(!no_echo && !c->side_seen && ab(observed_x-c->profile.front_mm)<=40 &&
       observed_y>=c->profile.right_edge_mm-20 && observed_y<=c->profile.left_edge_mm+20) {
        advance(c,now,f); return;
    }
    if (!no_echo && d<expected-B5_SIDE_MATCH_MM) {
        b5_core_fail(c,B5_FAULT_PROFILE,now); return; /* unexpected nearer object */
    }
    if (!no_echo && d<=upper+B5_SIDE_MATCH_MM) {
        int32_t x=f->forward_mm+d*cs[angle]/32768;
        if (x<c->profile.front_mm-80) {
            b5_core_fail(c,B5_FAULT_PROFILE,now); return;
        }
        if (c->side_seen && x<c->last_hit_x-30) {
            b5_core_fail(c,B5_FAULT_POSE,now); return;
        }
        c->wall_range_mm=d; c->side_seen=true; c->last_hit_x=x; c->last_hit_pose_x=f->forward_mm;
        c->clear_count=0; c->profile.depth_valid=false;
    } else if (no_echo || d>=upper+B5_CLEAR_JUMP_MM) {
        /* A farther echo or a FILTERED group of no-echo observations marks a gap.
         * The ray crosses the inferred wall plane at cross_x, not at d/2. */
        if (c->side_seen && !c->profile.depth_valid) {
            if (!c->clear_count) {
                if (cross_x<c->last_hit_x ||
                    cross_x-c->last_hit_x>B5_MAX_EDGE_BRACKET_MM ||
                    f->forward_mm-c->last_hit_pose_x>B5_MAX_EDGE_BRACKET_MM) {
                    b5_core_fail(c,B5_FAULT_PROFILE,now); return;
                }
                c->first_clear_x=cross_x;
                c->last_clear_pose_x=f->forward_mm;
                c->clear_count=1;
            } else if (f->forward_mm-c->last_clear_pose_x>=B5_STEP_MM/2) {
                c->last_clear_pose_x=f->forward_mm;
                ++c->clear_count;
            }
            if (c->clear_count>=B5_EDGE_CONFIRMATIONS) {
                c->profile.depth_low_mm=mx(0,c->last_hit_x-c->profile.front_mm-B5_EDGE_ALLOWANCE_MM);
                c->profile.depth_high_mm=mx(0,c->first_clear_x-c->profile.front_mm+B5_EDGE_ALLOWANCE_MM);
                c->profile.depth_valid=true;
            }
        } else if (!c->side_seen && cross_x>c->profile.front_mm+100) {
            b5_core_fail(c,B5_FAULT_PROFILE,now); return;
        }
    } else {
        /* Ambiguous return: no progress/recovery based on uncertain boundary. */
        b5_core_fail(c,B5_FAULT_PROFILE,now); return;
    }
    if (c->profile.depth_valid &&
        f->forward_mm-B5_TURN_CLEARANCE_MM >=
          c->profile.front_mm+c->profile.depth_high_mm) {
        state(c,B5_LINE_RECOVERY,now);
        c->phase=0;
        request(c,B5_REQ_RECOVER_LINE,now,f->forward_mm,0);
    } else {
        advance(c,now,f);
    }
}
/* The scan has picked a side and an edge estimate: send the sideways leg. */
static void start_bypass(b5_core_t *c,uint32_t now) {
#if B5_TIGHT_COURSE
    /* Preliminary 10 cm obstacle / narrow-board demonstration.
     * This is deliberately not the final 20 cm safety profile. */
    if (ab(c->profile.offset_mm)>B5_TIGHT_OFFSET_MM)
        c->profile.offset_mm=c->profile.side*B5_TIGHT_OFFSET_MM;
#endif
#if B5_SIMPLE_BYPASS
    /* Sideways leg: box width + a few cm, with room to pivot beside the box (see planned_offset). */
    c->profile.offset_mm=planned_offset(&c->profile,c->profile.side);
#endif
    c->profile.alt_offset_mm=planned_offset(&c->profile,-c->profile.side);
    c->profile.alt_ok=ab(c->profile.alt_offset_mm)<=B5_MAX_OFFSET_MM;
    request(c,B5_REQ_BYPASS,now,c->profile.front_mm,c->profile.offset_mm);
}
#if B5_PROBE_DEG>0
/* Side check by pivoting the car (see B5_PROBE_DEG). Runs in B5_WAIT_STOP.
 * The car looks left at B5_PROBE_DEG, then B5_PROBE_STEP_DEG further each time, until the box is no longer in view;
 * then the same to the right. The angle where the box drops out of view is that side's edge angle. The sonar beam
 * blurs both edges by the same amount, so the smaller angle is the nearer edge, and the car passes on that side. */
static void probe_look(b5_core_t *c,uint32_t now) {
    c->probe=1;
    request(c,B5_REQ_LOOK,now,0,c->probe_side*(B5_PROBE_DEG+(int32_t)c->probe_idx*B5_PROBE_STEP_DEG));
}
static void probe_step(b5_core_t *c,uint32_t now,const b5_feedback_t *f,const b5_range_t *r) {
    bool done=f->completed_id==c->request.id && f->stopped;
    if (c->probe==1) {
        if (done) { c->probe=2; c->probe_since=now; }
    } else if (c->probe==2) {
        /* "Box in view" = an echo no further than a box at this distance could give when seen at an angle. */
        int32_t near=mn(2*c->profile.front_mm+100,900);
        int seen=-1, edge;
        if ((uint32_t)(now-c->probe_since)<B5_PROBE_SETTLE_MS) return;
        if (r && b5_range_no_echo(r)) seen=0;
        else if (r && good(r)) seen=(int32_t)r->distance_mm<=near;
        else if ((uint32_t)(now-c->probe_since)<B5_PROBE_TIMEOUT_MS) return;
        if (seen>0) {
            /* The farthest echo still belonging to the box on this side: the further the box reaches, the larger. */
            uint16_t *reach=c->probe_side>0 ? &c->probe_left_reach : &c->probe_right_reach;
            if (r->distance_mm>*reach) *reach=(uint16_t)r->distance_mm;
        }
        if (seen>0 && c->probe_idx+1u<B5_PROBE_LOOKS) { c->probe_idx++; probe_look(c,now); return; }
        /* Edge angle of this side: where the box dropped out of view (90 = still in view at the last look, 1 = no reading). */
        edge=seen<0 ? 1 : seen ? 90 : B5_PROBE_DEG+(int32_t)c->probe_idx*B5_PROBE_STEP_DEG;
        if (c->probe_side>0) { c->probe_left_deg=(uint8_t)edge; c->probe_side=-1; c->probe_idx=0; probe_look(c,now); }
        else { c->probe_right_deg=(uint8_t)edge; c->probe=3; request(c,B5_REQ_LOOK,now,0,0); }
    } else {
        if (!done) return;
        if (!f->heading_aligned) { b5_core_fail(c,B5_FAULT_POSE,now); return; }
        if (c->probe_left_deg>1 && c->probe_right_deg>1 && c->probe_left_deg!=c->probe_right_deg) {
            c->profile.side=c->probe_left_deg<c->probe_right_deg ? 1 : -1;   /* pass on the side of the nearer edge */
            c->profile.side_reason=B5_WHY_PROBE;
        } else if (c->probe_left_deg>1 && c->probe_left_deg==c->probe_right_deg &&
                   ab((int32_t)c->probe_left_reach-(int32_t)c->probe_right_reach)>=30) {
            /* Out of view at the same angle on both sides: the side whose last echoes were nearer has the nearer edge. */
            c->profile.side=c->probe_left_reach<c->probe_right_reach ? 1 : -1;
            c->profile.side_reason=B5_WHY_PROBE;
        }
        c->probe=0;
        state(c,B5_PROCESSING,now);
        start_bypass(c,now);
    }
}
#endif
void b5_core_update(b5_core_t *c,uint32_t now,
                    const b5_feedback_t *f,const b5_range_t *r) {
    c->now_ms=now;
    if (f && f->valid) { c->turn_fixes=f->turn_fixes; c->block_fixes=f->block_fixes; }
    if (c->state==B5_ERROR) return;
    if (!fresh(now,f)) { b5_core_fail(c,B5_FAULT_FEEDBACK,now); return; }
    if (f->request_failed) { b5_core_fail(c,B5_FAULT_CONTROLLER,now); return; }
    if (f->forward_mm < -10000 || f->forward_mm>10000 ||
        f->lateral_mm < -10000 || f->lateral_mm>10000) {
        b5_core_fail(c,B5_FAULT_POSE,now); return;
    }
    if ((c->state==B5_WAIT_STOP || c->state==B5_PROCESSING ||
         c->state==B5_LINE_RECOVERY || (c->state==B5_AVOIDING && c->phase==2)) &&
        (uint32_t)(now-c->request_since_ms)>B5_REQUEST_TIMEOUT_MS) {
        b5_core_fail(c,B5_FAULT_TIMEOUT,now); return;
    }
    if (c->state==B5_SCANNING &&
        (uint32_t)(now-c->state_since_ms)>B5_SCAN_TIMEOUT_MS) {
        b5_core_fail(c,B5_FAULT_TIMEOUT,now); return;
    }
    if ((c->state==B5_AVOIDING || c->state==B5_LINE_RECOVERY) &&
        (uint32_t)(now-c->avoid_since_ms)>B5_AVOID_TIMEOUT_MS) {
        b5_core_fail(c,B5_FAULT_TIMEOUT,now); return;
    }
    switch(c->state) {
    case B5_MONITORING:
        if (!r) return;
        if (!good(r) && !b5_range_no_echo(r)) { c->front_clearance_mm=-1; b5_core_fail(c,B5_FAULT_SENSOR,now); return; }
        c->front_clearance_mm=b5_front_clearance(r);
        if (c->front_clearance_mm<=B5_STOP_FRONT_MM) {
            uint32_t id=c->profile.obstacle_id+1u;
            memset(&c->profile,0,sizeof(c->profile)); c->profile.obstacle_id=id; c->rescans=0; c->extends=0; c->alt_tried=false; c->route_check_since=0; c->probe=0;
            state(c,B5_WAIT_STOP,now);
            request(c,B5_REQ_STOP,now,0,0);
        }
        break;
    case B5_WAIT_STOP:
#if B5_PROBE_DEG>0
        if (c->probe) { probe_step(c,now,f,r); break; }
#endif
        if (f->completed_id==c->request.id && f->stopped) {
            if (!f->heading_aligned || ab(f->forward_mm)>5 || ab(f->lateral_mm)>5)
                b5_core_fail(c,B5_FAULT_POSE,now);
            else scan_start(c,now);
        }
        break;
    case B5_SCANNING: {
        if (!f->stopped || !f->heading_aligned ||
            ab(f->forward_mm)>5 || ab(f->lateral_mm)>5) {
            b5_core_fail(c,B5_FAULT_POSE,now); return;
        }
        if (!r) return;
        unsigned i=c->scan.slot;
        c->scan.sampled[i]=1; c->scan.valid[i]=b5_range_no_echo(r)?B5_SCAN_NO_ECHO:(uint8_t)good(r);
        c->scan.distance[i]=good(r) ? (uint16_t)r->distance_mm : 0;
        if (!scan_next(&c->scan)) {
            state(c,B5_PROCESSING,now);
            if (!process(c)) b5_core_fail(c,B5_FAULT_NO_ROUTE,now);
            else {
#if B5_SIMPLE_BYPASS && B5_MAX_RESCANS>0
                if ((c->profile.side_reason==B5_WHY_NO_EDGES || c->profile.side_reason==B5_WHY_FAR_RAY) && (int)c->rescans<(int)B5_MAX_RESCANS) {
                    /* Neither box edge is inside the scan window, so no side can be judged: reverse a little (the window
                     * then covers more of the box) and measure again. */
                    c->rescans++;
                    state(c,B5_WAIT_STOP,now);
                    request(c,B5_REQ_BACKUP,now,B5_RESCAN_BACKUP_MM,0);
                    return;
                }
#endif
#if B5_SIMPLE_BYPASS && B5_PROBE_DEG>0
                if (!c->edges_seen) {
                    /* Not both edges measured: turn the car left and right to see which side the box extends to. */
                    c->probe_left_deg=c->probe_right_deg=0; c->probe_left_reach=c->probe_right_reach=0; c->probe_side=1; c->probe_idx=0;
                    state(c,B5_WAIT_STOP,now);
                    probe_look(c,now);
                    return;
                }
#endif
                start_bypass(c,now);
            }
        }
        break;
    }
    case B5_PROCESSING:
        if (f->completed_id==c->request.id && f->stopped) {
            /* After a sonar extra turn the sideways leg was deliberately re-aimed, so the forward position (which
             * the turn tilts) is not held to the tolerance. */
            if (!f->heading_aligned || ab(f->lateral_mm-c->profile.offset_mm)>B5_ADV_TOL_Y_MM ||
                (c->block_fixes==0 && ab(f->forward_mm)>B5_ADV_TOL_X_MM)) { b5_core_fail(c,B5_FAULT_POSE,now); return; }
#if B5_SIMPLE_BYPASS
            {
                /* The car now sits at the side position facing forward. Before the straight pass, the sonar must see
                 * clear space for the whole pass (a second box next to the first one would otherwise be driven into). */
                int32_t need=c->profile.front_mm+B5_AXLE_TO_SENSOR_MM+B5_PASS_BESIDE_MM+B5_PASS_TAIL_MM+B5_ROUTE_MARGIN_MM;
                {
                    /* The wide sonar beam also hears the corner of the box being passed, which is beside the path, not
                     * in it. That corner is hypot(front, gap) away, clearly more than the box-front distance;
                     * something really in the path next to the box (a second box) answers at about the box-front
                     * distance. Only that kind blocks the side. Anything further back is caught during the pass
                     * (B5_ADVANCE_STOP_MM). */
                    int32_t inpath=c->profile.front_mm+B5_ROUTE_CORNER_MARGIN_MM-B5_FRONT_OVERHANG_MM;
                    if (inpath<need) need=inpath;
                }
                if (!c->route_check_since) { c->route_check_since=now; return; }
                if ((uint32_t)(now-c->route_check_since)<B5_ROUTE_CHECK_MS) return;
                if (c->front_clearance_mm>=0 && c->front_clearance_mm!=B5_CLEAR_FRONT_MM && c->front_clearance_mm<need) {
                    if (!c->alt_tried && c->profile.alt_ok) {
                        /* Blocked on this side: go to the other side of the box instead. */
                        c->alt_tried=true;
                        c->profile.side=(int8_t)-c->profile.side;
                        c->profile.offset_mm=c->profile.alt_offset_mm;
                        c->route_check_since=0;
                        request(c,B5_REQ_BYPASS,now,c->profile.front_mm,c->profile.offset_mm);
                        return;
                    }
                    b5_core_fail(c,B5_FAULT_NO_ROUTE,now); return;
                }
                c->route_check_since=0;
            }
#endif
            c->bypass_y=f->lateral_mm; c->side_seen=false; c->clear_count=0;
            c->avoid_since_ms=now; c->phase=0;
            state(c,B5_AVOIDING,now);
#if B5_SIMPLE_BYPASS
            {
                /* One long move past the whole box: front + depth + car length + margin. */
                /* Pass: factor x the measured width from the first stop point. */
                int32_t width=c->profile.width_valid ? c->profile.width_mm : B5_UNKNOWN_WIDTH_MM;
                int32_t pass=B5_PASS_WIDTH_FACTOR*width;
                if (pass<B5_MIN_PASS_MM) pass=B5_MIN_PASS_MM;
                c->advance_start_x=f->forward_mm;
                c->phase=2;
                /* The origin moved back by every rescan reverse, so add those back. */
                (void)pass;
                request(c,B5_REQ_ADVANCE,now,f->forward_mm+(int32_t)c->rescans*B5_RESCAN_BACKUP_MM+c->profile.front_mm+B5_AXLE_TO_SENSOR_MM+B5_PASS_BESIDE_MM+B5_PASS_TAIL_MM,c->bypass_y);
            }
#endif
        }
        break;
    case B5_AVOIDING:
        if (f->forward_mm>c->profile.front_mm+B5_MAX_PROFILE_MM+B5_TURN_CLEARANCE_MM+150) {
            b5_core_fail(c,B5_FAULT_PROFILE,now); return;
        }
        if (c->phase==2) {
#if B5_SIMPLE_BYPASS
            /* Something appeared ahead during the straight pass: stop before touching it. */
            if (f->completed_id!=c->request.id && c->front_clearance_mm>=0 &&
                c->front_clearance_mm!=B5_CLEAR_FRONT_MM && c->front_clearance_mm<=B5_ADVANCE_STOP_MM) {
                b5_core_fail(c,B5_FAULT_NO_ROUTE,now); return;
            }
#endif
            if (f->completed_id==c->request.id && f->stopped) {
                if (!f->heading_aligned || ab(f->lateral_mm-c->bypass_y)>B5_ADV_TOL_Y_MM ||
                    ab(f->forward_mm-c->request.forward_mm)>B5_ADV_TOL_X_MM ||
                    f->forward_mm<=c->advance_start_x) {
                    b5_core_fail(c,B5_FAULT_POSE,now); return;
                }
#if B5_SIMPLE_BYPASS
                /* Past the box: head back to the line. */
                state(c,B5_LINE_RECOVERY,now);
                c->phase=0; c->recover_checked=false;
                request(c,B5_REQ_RECOVER_LINE,now,f->forward_mm,0);
                return;
#endif
                c->phase=0;
            }
            return;
        }
        if (!f->stopped || !f->heading_aligned || ab(f->lateral_mm-c->bypass_y)>20) {
            b5_core_fail(c,B5_FAULT_POSE,now); return;
        }
        if (!r) return;
        /* The car is stationary while sampling this direction. A noisy
         * ultrasonic batch gives no evidence about the obstacle edge, so
         * retry it; the avoidance timeout still bounds repeated failures. */
        if (!good(r) && !b5_range_no_echo(r)) return;
        if (c->phase==0) {
            c->front_clearance_mm=b5_front_clearance(r);
            if (c->front_clearance_mm<B5_STOP_FRONT_MM+B5_STEP_MM) {
                b5_core_fail(c,B5_FAULT_NO_ROUTE,now); return;
            }
            c->phase=1;
        } else side_sample(c,now,f,r);
        break;
    case B5_LINE_RECOVERY:
        c->extends=f->extends; c->recover_checked=f->front_ok;
        if (f->front_ok && !c->profile.depth_valid) {
            /* Sonar saw nothing beside the box after k extends: box depth is between the previous and this axle position. */
            int32_t gain=B5_EXTEND_MM, k=f->extends;
            int32_t first=B5_PASS_BESIDE_MM+B5_PASS_TAIL_MM;   /* where the first sonar check happens */
            c->profile.depth_high_mm=first+k*gain;
            c->profile.depth_low_mm=k>0 ? first+(k-1)*gain : 0;
            c->profile.depth_valid=true;
        }
        if (f->completed_id==c->request.id) {
            if (!c->phase) {
                if (!f->stopped || !f->line_acquired || !f->heading_aligned) {
                    b5_core_fail(c,B5_FAULT_CONTROLLER,now); return;
                }
                c->phase=1;
                request(c,B5_REQ_RESUME_LINE,now,0,0);
            } else {
                state(c,B5_MONITORING,now);
                c->request.kind=B5_REQ_NONE;
            }
        }
        break;
    default: break;
    }
}
#endif
