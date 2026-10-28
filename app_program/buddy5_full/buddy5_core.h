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
 B5_REQ_RECOVER_LINE, B5_REQ_RESUME_LINE
} b5_request_kind_t;
typedef struct {
 b5_request_kind_t kind;
 uint32_t id;
 int32_t lateral_mm; /* signed target, +left */
 int32_t forward_mm; /* ADVANCE absolute target; BYPASS obstacle front */
} b5_request_t;
typedef struct {
 bool valid, stopped, fault, request_failed;
 bool heading_aligned, line_acquired;
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
 int32_t front_mm, right_edge_mm, left_edge_mm, width_mm;
 int32_t depth_low_mm, depth_high_mm;
 int32_t left_inner_mm, right_inner_mm;
 int32_t offset_mm; /* candidate, NOT certified collision-free */
 int8_t side; /* +1 left, -1 right */
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