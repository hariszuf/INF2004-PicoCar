#ifndef PICOCAR_BUDDY5_PLAN_H
#define PICOCAR_BUDDY5_PLAN_H
#include "../barrc_types.h"
#include "buddy5_core.h"
/* Pure planning (no state, no hardware): turn one stop-point scan into a side,
 * a sideways distance and the wheel-axle
 * position where the car may turn back. Frame: origin = wheel axle at the stop
 * point, +x along the line, +y left. */
typedef struct
{
    bool face_valid;                 /* a box face was found straight ahead */
    /* sonar to the face plane, measured square-on */
    int32_t face_dist_mm;
    int32_t x_face_mm;               /* face position in the axle frame */
    /* [0]=left [1]=right: that edge was really seen by the scan */
    bool seen[2];
    /* planned distance from the line to the box edge on that side */
    int32_t edge_mm[2];
    /* edge from the scan alone (no minimum box size); a lower bound when !seen
     */
    int32_t meas_edge_mm[2];
    /* free width beyond that edge (B5_ROOM_FAR_MM = nothing seen) */
    int32_t room_mm[2];
    bool feasible[2];
    /* sideways distance from the line for the car's axle */
    int32_t offset_mm[2];
    int32_t cost_mm[2];
    /* axle x where the car can turn back toward the line */
    int32_t pass_x_mm[2];
    bool route_ok;
    /* +1 left, -1 right (valid when route_ok) */
    int8_t side;
    int8_t reason;                   /* B5_WHY_* */
} b5_plan_t;
static inline int32_t b5_side_index (int32_t side)
{
    return side > 0 ? 0 : 1;
}

/* blocked_mask: bit0 left, bit1 right (sides already found blocked).
 * front_hint_mm: forward clearance when the stop
 * was triggered (used only when the scan has no usable straight-ahead echo). */
void b5_plan_make (const b5_scan_t * p_scan, int32_t front_hint_mm,
                   uint8_t blocked_mask, b5_plan_t * out);
/* Smallest wheel-axle x (stop-point frame) at which the car, at sideways offset
 * offset_mm, can pivot 90 degrees toward
 * the line and drive to it without its outline coming within B5_SWEEP_MARGIN_MM
 * of the box, plus B5_PASS_EXTRA_MM.
 * The box occupies x in [x_face, x_face+depth]; near_edge_mm / far_edge_mm are
 * its edges from the line (near = the
 * side the car passes on). */
int32_t b5_pass_x (int32_t x_face_mm, int32_t depth_mm, int32_t near_edge_mm,
                   int32_t far_edge_mm, int32_t offset_mm);
/* Does a car with its axle at (x,u) heading rotated phi_deg toward the line
 * touch the (inflated) box? Exposed for tests. */
bool b5_car_hits_box (float32_t x, float32_t u, float32_t phi_deg,
                      float32_t bx0, float32_t bx1,
                      float32_t bu0, float32_t bu1, float32_t margin);
#endif

/*** end of file ***/
