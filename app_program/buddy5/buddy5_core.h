#ifndef PICOCAR_BUDDY5_CORE_H
#define PICOCAR_BUDDY5_CORE_H
#include "../barrc_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "buddy5_config.h"
#include "../bitpack.h"
/* Buddy 5 core (portable logic: no heap, no RTOS, no hardware). See
 * buddy5_plan.h for the side choice and
 * buddy5_core.c for the state machine. The scan task (buddy5_task.c) feeds it
 * sonar ranges and hands its
 * requests to the vehicle controller. */
typedef enum
{
    B5_MONITORING, B5_WAIT_STOP, B5_SCANNING, B5_PROCESSING,
    B5_AVOIDING, B5_LINE_RECOVERY, B5_ERROR
} b5_state_t;
typedef enum
{
    B5_FAULT_NONE, B5_FAULT_SENSOR, B5_FAULT_FEEDBACK, B5_FAULT_TIMEOUT,
    B5_FAULT_NO_ROUTE, B5_FAULT_CONTROLLER, B5_FAULT_POSE,
    B5_FAULT_PROFILE, B5_FAULT_SERVO
} b5_fault_t;
typedef enum
{
    B5_REQ_NONE, B5_REQ_STOP, B5_REQ_BYPASS, B5_REQ_ADVANCE,
    B5_REQ_RECOVER_LINE, B5_REQ_RESUME_LINE, B5_REQ_BARCODE_LEFT,
    B5_REQ_BARCODE_RIGHT,
    B5_REQ_BACKUP, /* reverse forward_mm millimetres, then re-zero the origin */
    /* pivot on the spot to heading lateral_mm (degrees, + = left); the position
     * is
     * kept */
    B5_REQ_LOOK
} b5_request_kind_t;
typedef struct
{
    b5_request_kind_t kind;
    uint32_t id;
    int32_t lateral_mm; /* signed target, +left (LOOK: heading in degrees) */
    int32_t forward_mm; /* ADVANCE absolute target */
} b5_request_t;
/* Why a side was chosen (telemetry). */
#define B5_WHY_NONE 0
#define B5_WHY_ONLY_FEASIBLE 1   /* the other side has no room */
/* both possible, this one needs the shorter sideways move */
#define B5_WHY_CHEAPER 2
/* costs tied (sonar noise), this side has more free room */
#define B5_WHY_TIE_MORE_ROOM 3
/* nothing decided: the configured default side */
#define B5_WHY_DEFAULT 4
#define B5_WHY_NO_ROUTE 5
typedef struct
{
    bool valid:1, stopped:1, fault:1, request_failed:1;
    bool heading_aligned:1, line_acquired:1;
    /* sonar saw nothing ahead after turning toward the line */
    bool front_ok:1;
    uint8_t turn_fixes;
    uint8_t block_fixes;
    uint8_t extends;
    uint32_t timestamp_ms;
    uint32_t completed_id;
    /* Wheel-axle position relative to the stop point: +forward = original line
     * direction, +lateral = left. */
    int32_t forward_mm;
    int32_t lateral_mm;
} b5_feedback_t;
/* Explicit observation marker: no numerical distance was measured. */
#define B5_NO_ECHO_MM UINT32_MAX
#define B5_CLEAR_FRONT_MM INT32_MAX
#define B5_SCAN_ECHO 1u
#define B5_SCAN_NO_ECHO 2u
typedef struct
{
    bool valid;
    uint32_t distance_mm;
} b5_range_t;
static inline bool b5_range_no_echo (const b5_range_t * p_r)
{
    return p_r && (p_r->valid) && (B5_NO_ECHO_MM == p_r->distance_mm);
}

static inline int32_t b5_front_clearance (const b5_range_t * p_r)
{
    return b5_range_no_echo(p_r) ? B5_CLEAR_FRONT_MM :
    p_r && (p_r->valid) ? (int32_t)p_r->distance_mm - B5_FRONT_OVERHANG_MM : -1;
}
#define B5_SCAN_POINTS 13 /* -35..+25, slots every 5 degrees */
/* Packed scan: per slot a 13-bit distance (0..8191 mm, the sonar tops out near
 * 4116 mm) and a 2-bit state
 * (0 not measured / invalid, B5_SCAN_ECHO, B5_SCAN_NO_ECHO), 15 bits per slot,
 * stored in a byte array. */
#define B5_SCAN_DIST_BITS 13u
#define B5_SCAN_STATE_BITS 2u
#define B5_SCAN_SLOT_BITS (B5_SCAN_DIST_BITS+B5_SCAN_STATE_BITS)
#define B5_SCAN_DIST_MAX ((1u<<B5_SCAN_DIST_BITS)-1u)
typedef struct
{
    uint8_t bits[BITPACK_BYTES(B5_SCAN_POINTS * B5_SCAN_SLOT_BITS)];
} b5_scan_t;
static inline uint16_t b5_scan_dist (const b5_scan_t * s, int32_t idx)
{
    return (uint16_t)bitpack_get(s->bits, (uint32_t)idx * B5_SCAN_SLOT_BITS,
                                 B5_SCAN_DIST_BITS);
}

static inline uint8_t b5_scan_state (const b5_scan_t * s, int32_t idx)
{
    return (uint8_t)bitpack_get(s->bits,
                                (uint32_t)idx * B5_SCAN_SLOT_BITS
                                + B5_SCAN_DIST_BITS, B5_SCAN_STATE_BITS);
}

static inline void b5_scan_set (b5_scan_t * s, int32_t idx, uint8_t state,
                                uint32_t dist)
{
    if (dist > B5_SCAN_DIST_MAX)
    {
        dist = B5_SCAN_DIST_MAX;
    }

    bitpack_put(s->bits, (uint32_t)idx * B5_SCAN_SLOT_BITS, B5_SCAN_DIST_BITS,
                dist);
    bitpack_put(s->bits, (uint32_t)idx * B5_SCAN_SLOT_BITS + B5_SCAN_DIST_BITS,
                B5_SCAN_STATE_BITS, state);
}

static inline int32_t b5_slot_angle (int32_t slot)
{
    return B5_SCAN_MIN_DEG + 5 * slot;
}

/* What the planner decided (also reported through the status for
 * telemetry/printing). */
typedef struct
{
    bool valid:1;                    /* a box was found and a route exists */
    bool width_valid:1, depth_valid:1;
    bool left_seen:1, right_seen:1;  /* the scan really saw that box edge */
    int8_t side:2;                   /* +1 left, -1 right */
    uint8_t side_reason:3;           /* B5_WHY_* */
    /* car front to box face at the stop (for printing) */
    int32_t front_mm;
    int32_t width_mm;
    int32_t depth_low_mm;
    int32_t depth_high_mm;
    /* chosen side: sideways distance from the line (positive magnitude) */
    int32_t offset_mm;
    /* chosen side: wheel-axle x (from the stop point) where the car turns back
     */
    int32_t pass_x_mm;
    int32_t left_edge_mm;
    int32_t right_edge_mm;
    int32_t left_room_mm;
    int32_t right_room_mm;
    int32_t left_cost_mm;
    int32_t right_cost_mm;
} b5_profile_t;

/* Sonar sampler: turns single pings into one filtered range. Used by the scan
 * task and by the host simulation. */
#define B5_SAMPLER_N 3
typedef struct
{
    b5_range_t s[B5_SAMPLER_N];
    uint8_t n;
    uint8_t fresh;
} b5_sampler_t;
void b5_sampler_reset (b5_sampler_t * s);
/* Push one ping (valid + distance, or distance==B5_NO_ECHO_MM for no echo, or
 * valid=false for a bad ping).
 * Returns true when a filtered range is ready in *out (sliding window, only
 * after a full set since reset). */
bool b5_sampler_push (b5_sampler_t * s, const b5_range_t * p_sample,
                      b5_range_t * out);
b5_range_t b5_filter (const b5_range_t samples[B5_SAMPLER_N]);

typedef struct
{
    b5_state_t state:3;
    b5_fault_t fault:4;
    /* issued but not yet accepted by the controller */
    bool request_pending:1;
    bool request_active:1;           /* accepted, waiting for completion */
    b5_request_t request;            /* last request issued */
    b5_profile_t profile;
    b5_scan_t scan;
    uint32_t state_since_ms;
    uint32_t request_since_ms;
    uint32_t avoid_since_ms;
    uint32_t next_request_id;
    uint32_t now_ms;
    /* latest filtered forward reading (angle 0) */
    int32_t front_clearance_mm;
    uint32_t front_ms;
    uint32_t front_seq;
    uint8_t slot:4;                  /* SCANNING: current servo slot (0..13) */
    uint8_t phase:4;
    uint8_t hit_count;
    uint8_t false_alarms;
    uint32_t scan_count;
    /* scan-only: completed scans with a front face / scans that saw none */
    uint32_t noface_count;
    int8_t side:2;                   /* committed / current candidate side */
    /* bit0 left, bit1 right: tried and found blocked */
    uint8_t blocked_mask:2;
    int32_t offset_mm;
    int32_t pass_x_mm;    /* for the current side */
    int32_t off_arr[2];
    int32_t pass_arr[2];
    int32_t edge_arr[2];   /* per side [0]=left [1]=right, from the plan */
    bool feas_arr[2];
    /* forward clearance when the stop was triggered */
    int32_t stop_front_mm;
    uint32_t check_since_ms;
    uint32_t cooldown_until_ms;
    uint32_t recover_since_ms;
    uint8_t chk_n;
    int32_t chk_min;
    bool fb_front_ok:1;
    bool plan_edges_valid:1;
    /* kept for the box-corner test in the pass check */
    int32_t x_face_mm;
    int32_t edge_near_mm;
    uint8_t adv_close;      /* consecutive close echoes while passing the box */
} b5_core_t;
/* The packed fields above must hold every value they are given. */
_Static_assert(B5_SCAN_POINTS <= 15, "slot field is 4 bits");
_Static_assert(B5_ERROR < 8, "b5_state_t fits 3 bits");
_Static_assert(B5_FAULT_SERVO < 16, "b5_fault_t fits 4 bits");
_Static_assert(sizeof(b5_scan_t) == (B5_SCAN_POINTS * B5_SCAN_SLOT_BITS + 7)
               / 8,
               "packed scan size");
void b5_core_init (b5_core_t * c, uint32_t now_ms);
/* Re-arm after a stop / restart: back to monitoring, nothing pending. */
void b5_core_reset (b5_core_t * c, uint32_t now_ms);
void b5_core_fail (b5_core_t * c, b5_fault_t fault, uint32_t now_ms);
/* Call every loop. range!=NULL means NEW filtered data taken at b5_core_angle()
 * after the servo settled.
 * f is the latest controller feedback (may be NULL before the controller runs).
 */
void b5_core_update (b5_core_t * c, uint32_t now_ms, const b5_feedback_t * f,
                     const b5_range_t * range);
/* Servo angle wanted now (degrees, + = left). */
int32_t b5_core_angle (const b5_core_t * c);
/* The controller request waiting to be sent: returns true and fills *r; call
 * b5_core_request_sent() once accepted. */
bool b5_core_peek_request (const b5_core_t * c, b5_request_t * p_r);
void b5_core_request_sent (b5_core_t * c, uint32_t now_ms);
const char * b5_state_name (b5_state_t s);
#endif

/*** end of file ***/
