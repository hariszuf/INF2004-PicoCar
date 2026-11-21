#ifndef PICOCAR_BUDDY5_CORE_H
#define PICOCAR_BUDDY5_CORE_H
#include <stdbool.h>
#include <stdint.h>
#include "buddy5_config.h"
/* Buddy 5 core (portable logic: no heap, no RTOS, no hardware). See buddy5_plan.h for the side choice and
 * buddy5_core.c for the state machine. The scan task (buddy5_task.c) feeds it sonar ranges and hands its
 * requests to the vehicle controller. */
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
 B5_REQ_BACKUP, /* reverse forward_mm millimetres, then re-zero the origin */
 B5_REQ_LOOK    /* pivot on the spot to heading lateral_mm (degrees, + = left); the position is kept */
} b5_request_kind_t;
typedef struct {
 b5_request_kind_t kind;
 uint32_t id;
 int32_t lateral_mm; /* signed target, +left (LOOK: heading in degrees) */
 int32_t forward_mm; /* ADVANCE absolute target */
} b5_request_t;
/* Why a side was chosen (telemetry). */
#define B5_WHY_NONE 0
#define B5_WHY_ONLY_FEASIBLE 1   /* the other side has no room */
#define B5_WHY_CHEAPER 2         /* both possible, this one needs the shorter sideways move */
#define B5_WHY_TIE_MORE_ROOM 3   /* costs tied (sonar noise), this side has more free room */
#define B5_WHY_DEFAULT 4         /* nothing decided: the configured default side */
#define B5_WHY_NO_ROUTE 5
typedef struct {
 bool valid, stopped, fault, request_failed;
 uint8_t turn_fixes, block_fixes;
 bool heading_aligned, line_acquired;
 bool front_ok;      /* sonar saw nothing ahead after turning toward the line */
 uint8_t extends;
 uint32_t timestamp_ms, completed_id;
 /* Wheel-axle position relative to the stop point: +forward = original line direction, +lateral = left. */
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
 uint8_t state[B5_SCAN_POINTS];   /* 0 not measured / invalid, B5_SCAN_ECHO, B5_SCAN_NO_ECHO */
} b5_scan_t;
static inline int b5_slot_angle(int slot) { return B5_SCAN_MIN_DEG+5*slot; }

/* What the planner decided (also reported through the status for telemetry/printing). */
typedef struct {
 bool valid;                      /* a box was found and a route exists */
 bool width_valid, depth_valid;
 int32_t front_mm;                /* car front to box face at the stop (for printing) */
 int32_t width_mm, depth_low_mm, depth_high_mm;
 int32_t offset_mm;               /* chosen side: sideways distance from the line (positive magnitude) */
 int32_t pass_x_mm;               /* chosen side: wheel-axle x (from the stop point) where the car turns back */
 int8_t side;                     /* +1 left, -1 right */
 int8_t side_reason;              /* B5_WHY_* */
 bool left_seen, right_seen;      /* the scan really saw that box edge */
 int32_t left_edge_mm, right_edge_mm, left_room_mm, right_room_mm;
 int32_t left_cost_mm, right_cost_mm;
} b5_profile_t;

/* Sonar sampler: turns single pings into one filtered range. Used by the scan task and by the host simulation. */
#define B5_SAMPLER_N 3
typedef struct { b5_range_t s[B5_SAMPLER_N]; uint8_t n; uint8_t fresh; } b5_sampler_t;
void b5_sampler_reset(b5_sampler_t *s);
/* Push one ping (valid + distance, or distance==B5_NO_ECHO_MM for no echo, or valid=false for a bad ping).
 * Returns true when a filtered range is ready in *out (sliding window, only after a full set since reset). */
bool b5_sampler_push(b5_sampler_t *s,const b5_range_t *sample,b5_range_t *out);
b5_range_t b5_filter(const b5_range_t samples[B5_SAMPLER_N]);

typedef struct {
 b5_state_t state;
 b5_fault_t fault;
 b5_request_t request;            /* last request issued */
 bool request_pending;            /* issued but not yet accepted by the controller */
 bool request_active;             /* accepted, waiting for completion */
 b5_profile_t profile;
 b5_scan_t scan;
 uint32_t state_since_ms, request_since_ms, avoid_since_ms, next_request_id, now_ms;
 int32_t front_clearance_mm;      /* latest filtered forward reading (angle 0) */
 uint32_t front_ms; uint32_t front_seq;
 uint8_t slot;                    /* SCANNING: current servo slot */
 uint8_t phase;
 uint8_t hit_count, false_alarms;
 uint32_t scan_count, noface_count;   /* scan-only: completed scans with a front face / scans that saw none */
 int8_t side;                     /* committed / current candidate side */
 uint8_t blocked_mask;            /* bit0 left, bit1 right: tried and found blocked */
 int32_t offset_mm, pass_x_mm;    /* for the current side */
 int32_t off_arr[2], pass_arr[2], edge_arr[2];   /* per side [0]=left [1]=right, from the plan */
 bool feas_arr[2];
 int32_t stop_front_mm;           /* forward clearance when the stop was triggered */
 uint32_t check_since_ms, cooldown_until_ms, recover_since_ms;
 uint8_t chk_n; int32_t chk_min;
 bool fb_front_ok;
 bool plan_edges_valid;
 /* kept for the box-corner test in the pass check */
 int32_t x_face_mm, edge_near_mm;
  uint8_t adv_close;      /* consecutive close echoes while passing the box */
} b5_core_t;
void b5_core_init(b5_core_t *c,uint32_t now_ms);
/* Re-arm after a stop / restart: back to monitoring, nothing pending. */
void b5_core_reset(b5_core_t *c,uint32_t now_ms);
void b5_core_fail(b5_core_t *c,b5_fault_t fault,uint32_t now_ms);
/* Call every loop. range!=NULL means NEW filtered data taken at b5_core_angle() after the servo settled.
 * f is the latest controller feedback (may be NULL before the controller runs). */
void b5_core_update(b5_core_t *c,uint32_t now_ms,const b5_feedback_t *f,const b5_range_t *range);
/* Servo angle wanted now (degrees, + = left). */
int b5_core_angle(const b5_core_t *c);
/* The controller request waiting to be sent: returns true and fills *r; call b5_core_request_sent() once accepted. */
bool b5_core_peek_request(const b5_core_t *c,b5_request_t *r);
void b5_core_request_sent(b5_core_t *c,uint32_t now_ms);
const char *b5_state_name(b5_state_t s);
#endif
