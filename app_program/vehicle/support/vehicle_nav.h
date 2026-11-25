#ifndef VEHICLE_NAV_H
#define VEHICLE_NAV_H
#include "../../barrc_types.h"
#include <stdint.h>
#include <stdbool.h>
#include "../../buddy5/buddy5_core.h"
#include "../../buddy2/motion_control.h"
typedef struct
{
    uint32_t kind;
    float32_t value;
} full_action_t;
#define FULL_NAV_MAX_ACTIONS 12
typedef struct
{
    float32_t x;
    float32_t y;
    float32_t heading;
    float32_t last_distance;
    float32_t last_heading;
    float32_t heading_origin;
    bool observed;
    bool active;
    bool issued;
    bool moving;
    bool forward_moving;
    bool motion_seen;
    bool failed;
    bool resumed;
    bool recovery_stop;
    bool acquired;
    uint32_t search_phase;
    uint32_t retries;
    bool front_ok;
    int32_t front_angle;
    /* sonar check after facing the line (ahead, half back, back): 0 none yet, 1
     * clear, 2 blocked */ /* servo angle of the last range reading */
    uint8_t look[3];
    bool rejoining;
    bool tape_entered;
    bool tape_measured;   /* line crossing measured during the sweep back */
    float32_t tape_enter_y;
    /* line follower is running the junction-style hand-back */
    float32_t tape_line_y;
    float32_t recover_side;
    bool front_fresh;
    /* the sonar saw something ahead while driving to the line: stopping, then
     * drive further past */
    bool sweep_block;
    bool cand_valid;
    /* short black seen on the way to the line (not accepted as the tape): the
     * one
     * nearest the expected line */
    float32_t cand_y;
    /* distance from the line where the last sonar check passed */
    float32_t check_y;
    /* Turn / blocked-front corrections */
    uint8_t turn_fixes_total;
    uint8_t block_fixes_total;   /* since the last STOP (telemetry 5/59) */
    uint8_t turn_tries;
    uint8_t block_fixes;               /* current action / current request */
    /* a turn just finished: verify the heading before moving on */
    bool turn_recheck;
    /* the running motion is a sonar-triggered extra turn: repeat the same
     * action
     * afterwards */
    bool fixing;
    /* +1 left, -1 right: direction of the last turn (the extra turn goes the
     * same
     * way) */
    int32_t last_turn_dir;
    uint32_t block_since;
    int32_t front_mm;
    uint32_t settled;
    uint32_t line_seen;
    uint32_t index;
    uint32_t count;
    uint32_t search;
    uint32_t completed_id;
    uint32_t started;
    uint32_t check_since;
    b5_request_t request;
    full_action_t actions[FULL_NAV_MAX_ACTIONS];
} full_nav_t;
void full_nav_init (full_nav_t * n, const motion_status_t * m);
bool full_nav_accept (full_nav_t * n, const b5_request_t * p_r, uint32_t now);
bool full_nav_barcode_turn_active (const full_nav_t * n);
bool full_nav_rejoining (const full_nav_t * n);
void full_nav_set_front (full_nav_t * n, bool b_fresh, int32_t clearance_mm,
                         int32_t angle_deg);
void full_nav_update (full_nav_t * n, uint32_t now, const motion_status_t * m,
                      uint8_t mask, bool b_guard);
/* Forward range lost while a bypass leg is moving: stop and let the extra-turn
 * correction run instead of halting.
 * Returns false when the correction does not apply (or is used up) and the
 * caller should halt. */
bool full_nav_block_stop (full_nav_t * n);
void full_nav_feedback (const full_nav_t * n, uint32_t timestamp,
                        b5_feedback_t * f);
#endif

/*** end of file ***/
