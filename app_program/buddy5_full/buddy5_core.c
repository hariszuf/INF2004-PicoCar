#ifdef B5_FULL_TEST
#include "buddy5_core.h"
#include <string.h>

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
    return 0;
}
bool b5_core_wants_range(const b5_core_t *c) {
    return c->state==B5_MONITORING || c->state==B5_SCANNING ||
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
static int32_t projected_x(const b5_scan_t *s, int slot) {
    return (int32_t)s->distance[slot]*cs[ab(slot-B5_SCAN_ZERO_SLOT)]/32768;
}
static int32_t plane_y(int32_t front, int slot) {
    int k=slot-B5_SCAN_ZERO_SLOT, a=ab(k);
    int32_t y=front*sn[a]/cs[a];
    return k<0 ? -y : y;
}
static bool hit(const b5_scan_t *s,int i,int32_t front) {
    return s->valid[i]==B5_SCAN_ECHO && ab(projected_x(s,i)-front)<=80;
}
/* A local, approximately vertical-front box profile. Unknown sectors invalidate
 * edge estimates. These rays propose a side; they cannot certify swept volume. */
static bool process(b5_core_t *c) {
    b5_scan_t *s=&c->scan;
    b5_profile_t *p=&c->profile;
    int lo=B5_SCAN_ZERO_SLOT,hi=B5_SCAN_ZERO_SLOT,left_clear=-1,right_clear=-1;
    if (s->valid[B5_SCAN_ZERO_SLOT]!=B5_SCAN_ECHO) return false;
    p->front_mm=s->distance[B5_SCAN_ZERO_SLOT];
    for (int i=B5_SCAN_ZERO_SLOT+1;i<B5_SCAN_POINTS;++i) if (s->sampled[i]) {
        if (!s->valid[i]) return false;
        if (!hit(s,i,p->front_mm)) {
            if (s->valid[i]==B5_SCAN_ECHO && projected_x(s,i)<=p->front_mm+80) return false;
            left_clear=i; break;
        }
        hi=i;
    }
    for (int i=B5_SCAN_ZERO_SLOT-1;i>=0;--i) if (s->sampled[i]) {
        if (!s->valid[i]) return false;
        if (!hit(s,i,p->front_mm)) {
            if (s->valid[i]==B5_SCAN_ECHO && projected_x(s,i)<=p->front_mm+80) return false;
            right_clear=i; break;
        }
        lo=i;
    }
    if (left_clear<0 || right_clear<0 || hi==B5_SCAN_POINTS-1 || lo==0) return false;
    /* Outer edge of observed hit/clear angular bracket, enclosing estimate. */
    p->left_edge_mm=plane_y(p->front_mm,left_clear);
    p->right_edge_mm=plane_y(p->front_mm,right_clear);
    p->left_inner_mm=plane_y(p->front_mm,hi);
    p->right_inner_mm=plane_y(p->front_mm,lo);
    p->width_mm=p->left_edge_mm-p->right_edge_mm;
    p->width_valid=p->width_mm>0;
    int32_t extra=B5_CAR_WIDTH_MM/2+B5_CLEARANCE_MM+B5_EDGE_ALLOWANCE_MM;
    int32_t left=p->left_edge_mm+extra, right=p->right_edge_mm-extra;
    int32_t left_score=4001,right_score=4001;
    for (int i=0;i<B5_SCAN_POINTS;++i) if (s->sampled[i]) {
        if (i>=left_clear)
            left_score=mn(left_score,s->valid[i]==B5_SCAN_NO_ECHO ? 4001 : s->valid[i] ? s->distance[i] : 0);
        if (i<=right_clear)
            right_score=mn(right_score,s->valid[i]==B5_SCAN_NO_ECHO ? 4001 : s->valid[i] ? s->distance[i] : 0);
    }
    bool l=p->width_valid && left<=B5_MAX_OFFSET_MM &&
           left_score>left+B5_CAR_LENGTH_MM+B5_CLEARANCE_MM;
    bool r=p->width_valid && -right<=B5_MAX_OFFSET_MM &&
           right_score>-right+B5_CAR_LENGTH_MM+B5_CLEARANCE_MM;
    if (!l && !r) return false;
    p->side=(l && (!r || left_score>right_score ||
                  (left_score==right_score && left<=-right))) ? 1 : -1;
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
void b5_core_update(b5_core_t *c,uint32_t now,
                    const b5_feedback_t *f,const b5_range_t *r) {
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
            memset(&c->profile,0,sizeof(c->profile)); c->profile.obstacle_id=id;
            state(c,B5_WAIT_STOP,now);
            request(c,B5_REQ_STOP,now,0,0);
        }
        break;
    case B5_WAIT_STOP:
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
            else request(c,B5_REQ_BYPASS,now,c->profile.front_mm,c->profile.offset_mm);
        }
        break;
    }
    case B5_PROCESSING:
        if (f->completed_id==c->request.id && f->stopped) {
            if (!f->heading_aligned || ab(f->lateral_mm-c->profile.offset_mm)>20 ||
                ab(f->forward_mm)>20) { b5_core_fail(c,B5_FAULT_POSE,now); return; }
            c->bypass_y=f->lateral_mm; c->side_seen=false; c->clear_count=0;
            c->avoid_since_ms=now; c->phase=0;
            state(c,B5_AVOIDING,now);
        }
        break;
    case B5_AVOIDING:
        if (f->forward_mm>c->profile.front_mm+B5_MAX_PROFILE_MM+B5_TURN_CLEARANCE_MM+150) {
            b5_core_fail(c,B5_FAULT_PROFILE,now); return;
        }
        if (c->phase==2) {
            if (f->completed_id==c->request.id && f->stopped) {
                if (!f->heading_aligned || ab(f->lateral_mm-c->bypass_y)>20 ||
                    ab(f->forward_mm-c->request.forward_mm)>5 ||
                    f->forward_mm<=c->advance_start_x) {
                    b5_core_fail(c,B5_FAULT_POSE,now); return;
                }
                c->phase=0;
            }
            return;
        }
        if (!f->stopped || !f->heading_aligned || ab(f->lateral_mm-c->bypass_y)>20) {
            b5_core_fail(c,B5_FAULT_POSE,now); return;
        }
        if (!r) return;
        if (!good(r) && !b5_range_no_echo(r)) { b5_core_fail(c,B5_FAULT_SENSOR,now); return; }
        if (c->phase==0) {
            c->front_clearance_mm=b5_front_clearance(r);
            if (c->front_clearance_mm<B5_STOP_FRONT_MM+B5_STEP_MM) {
                b5_core_fail(c,B5_FAULT_NO_ROUTE,now); return;
            }
            c->phase=1;
        } else side_sample(c,now,f,r);
        break;
    case B5_LINE_RECOVERY:
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
