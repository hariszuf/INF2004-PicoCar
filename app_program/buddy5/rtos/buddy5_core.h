#ifndef PICOCAR_BUDDY5_CORE_H
#define PICOCAR_BUDDY5_CORE_H
#include <stdbool.h>
#include <stdint.h>
#include "buddy5_config.h"
/* Portable logic: no heap, RTOS or libm. */
typedef enum {
 B5_MONITORING, B5_WAIT_STOP, B5_SCANNING, B5_PROCESSING,
 B5_AVOIDING, B5_LINE_RECOVERY, B5_ERROR
} b5_state_t;
typedef enum {
 B5_FAULT_NONE, B5_FAULT_SENSOR, B5_FAULT_FEEDBACK, B5_FAULT_TIMEOUT,
 B5_FAULT_NO_ROUTE, B5_FAULT_CONTROLLER, B5_FAULT_POSE,
 B5_FAULT_PROFILE, B5_FAULT_SERVO
} b5_fault_t;
typedef enum {
 B5_REQ_NONE, B5_REQ_STOP, B5_REQ_BYPASS, B5_REQ_ADVANCE,
 B5_REQ_RECOVER_LINE, B5_REQ_RESUME_LINE, B5_REQ_BARCODE_LEFT, B5_REQ_BARCODE_RIGHT,
 B5_REQ_BACKUP, /* reverse forward_mm millimetres, then re-zero the origin (re-scan from farther away) */
 B5_REQ_LOOK    /* pivot on the spot to heading lateral_mm (degrees, + = left); the position is kept */
} b5_request_kind_t;
typedef struct {
 b5_request_kind_t kind;
 uint32_t id;
 int32_t lateral_mm; /* signed target, +left */
 int32_t forward_mm; /* ADVANCE absolute target; BYPASS obstacle front */
} b5_request_t;
/* Why the bypass side was chosen (telemetry 5/59). */
#define B5_WHY_NONE 0
#define B5_WHY_LEFT_EDGE_ONLY 1   /* only the left box edge is inside the scan: the box fills the right of it */
#define B5_WHY_RIGHT_EDGE_ONLY 2
#define B5_WHY_SHORTER_DETOUR 3   /* both edges seen: the side whose edge is closer to the car's path */
#define B5_WHY_MORE_ROOM 4        /* edges about equal: the side with more free space beyond the box */
#define B5_WHY_NO_EDGES 5         /* box fills the scan: no side can be measured by the scan (right unless the pivot looks decide) */
#define B5_WHY_ONLY_ROOM 6        /* only one side has enough free space */
#define B5_WHY_NEARER_OBJECT 8    /* something sits nearer than the box front on the other side */
#define B5_WHY_PROBE 9            /* the car turned left and right to look: the box edge on this side is nearer */
#define B5_WHY_FAR_RAY 7          /* box fills the scan: a ray on one side reaches clearly further */
typedef struct {
 bool valid, stopped, fault, request_failed;
 uint8_t turn_fixes, block_fixes;   /* heading re-turns / sonar extra turns done since the last STOP */
 bool heading_aligned, line_acquired;
 bool front_ok;      /* sonar saw nothing ahead after turning toward the line */
 uint8_t extends;    /* reverse+extend retries used so far */
 uint32_t timestamp_ms, completed_id;
 /* Sensor-origin position in reference frame established when STOP completes.
  * +forward=original line direction, +lateral=left. Do not reset during bypass.
  * Controller must account for sensor offset from wheel axle in odometry. */
 int32_t forward_mm, lateral_mm;
} b5_feedback_t;
/* Explicit observation marker: no numerical distance was measured. */
#define B5_NO_ECHO_MM UINT32_MAX
#define B5_CLEAR_FRONT_MM INT32_MAX
#define B5_SCAN_ECHO 1u
#define B5_SCAN_NO_ECHO 2u
typedef struct { bool valid; uint32_t distance_mm; } b5_range_t;
static inline bool b5_range_no_echo(const b5_range_t *r) {
 return r && r->valid && r->distance_mm==B5_NO_ECHO_MM;
}
static inline int32_t b5_front_clearance(const b5_range_t *r) {
 return b5_range_no_echo(r)?B5_CLEAR_FRONT_MM:
        r && r->valid?(int32_t)r->distance_mm-B5_FRONT_OVERHANG_MM:-1;
}
#define B5_SCAN_POINTS 13 /* -35..+25, slots every 5 degrees */
typedef struct {
 uint16_t distance[B5_SCAN_POINTS];
 uint8_t sampled[B5_SCAN_POINTS], valid[B5_SCAN_POINTS], wanted[B5_SCAN_POINTS];
 uint8_t slot;
 bool fine_pass;
} b5_scan_t;
typedef struct {
 bool width_valid, depth_valid;
 bool width_estimated; /* box wider than the scan window: width estimated from the outermost echoes */
 int32_t front_mm, right_edge_mm, left_edge_mm, width_mm;
 int32_t depth_low_mm, depth_high_mm;
 int32_t left_inner_mm, right_inner_mm;
 int32_t offset_mm; /* candidate, NOT certified collision-free */
 int8_t side; /* +1 left, -1 right */
 int8_t side_reason;  /* B5_WHY_* */
 int32_t alt_offset_mm; bool alt_ok;   /* offset for the OTHER side, used if the chosen side turns out to be blocked */
 bool left_measured, right_measured;  /* the edge was bracketed by clear rays (not estimated) */
 int32_t left_report_mm, right_report_mm;  /* edge estimate midway in the hit/clear bracket (reported; planning uses the outer edge) */
 uint32_t obstacle_id;
} b5_profile_t;
typedef struct {
 b5_state_t state;
 b5_fault_t fault;
 b5_request_t request;
 b5_profile_t profile;
 b5_scan_t scan;
 uint32_t state_since_ms, request_since_ms, avoid_since_ms, next_request_id;
 int32_t front_clearance_mm;
 int32_t last_hit_x, last_hit_pose_x, first_clear_x, last_clear_pose_x;
 int32_t bypass_y, advance_start_x, wall_range_mm;
 uint8_t clear_count, phase;
 uint8_t rescans, extends;
 uint8_t turn_fixes, block_fixes;
 uint8_t probe;              /* side check by pivoting: 0 off, 1 turning to a look, 2 pinging, 3 turning back */
 uint8_t probe_idx;          /* which look on the current side */
 int8_t probe_side;          /* +1 looking left, -1 looking right */
 uint16_t probe_left_reach, probe_right_reach;   /* farthest range (mm) at which a look on that side still saw the box */
 uint8_t probe_left_deg, probe_right_deg;   /* angle where the box left the view: 0 not done, 1 no reading, 90 still in view at the last look */
 uint32_t probe_since;
 bool recover_checked;       /* returning to the line: the check after the turn passed, the sonar now looks straight ahead */
 bool alt_tried;             /* the first side was blocked ahead, so the other side was taken */
 uint32_t route_check_since;
 uint32_t now_ms;            /* time of the last update (servo look pattern) */   /* backups done to see both box edges */
 bool edges_seen;   /* the last scan saw both edges, so the width is measured, not assumed */
 bool side_seen;
} b5_core_t;
void b5_core_init(b5_core_t *c, uint32_t now_ms);
bool b5_core_reset(b5_core_t *c, uint32_t now_ms, const b5_feedback_t *f);
void b5_core_fail(b5_core_t *c, b5_fault_t fault, uint32_t now_ms);
/* Call every loop, even without a sample. range!=NULL means NEW filtered data
 * at b5_core_angle(), settled, never replayed; f describes acquisition pose. */
void b5_core_update(b5_core_t *c, uint32_t now_ms,
                    const b5_feedback_t *f, const b5_range_t *range);
int b5_core_angle(const b5_core_t *c);
bool b5_core_wants_range(const b5_core_t *c);
b5_range_t b5_filter(const b5_range_t samples[B5_SAMPLE_COUNT]);
const char *b5_state_name(b5_state_t s);
#endif
