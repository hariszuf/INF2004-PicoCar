#ifndef VEHICLE_NAV_H
#define VEHICLE_NAV_H
#include <stdint.h>
#include <stdbool.h>
#include "../rtos/buddy5_core.h"
#include "../../buddy2/motion_control.h"
typedef struct { unsigned kind; float value; } full_action_t;
#define FULL_NAV_MAX_ACTIONS 12
typedef struct {
 float x,y,heading,last_distance,last_heading,heading_origin;
 bool observed,active,issued,moving,forward_moving,motion_seen,failed,resumed;
 bool recovery_stop,acquired;
 unsigned search_phase,retries;
 bool front_ok;
 int front_angle;
 uint8_t look[3];   /* sonar check after facing the line (ahead, half back, back): 0 none yet, 1 clear, 2 blocked */   /* servo angle of the last range reading */
 bool rejoining;
 bool tape_entered, tape_measured;   /* line crossing measured during the sweep back */
 float tape_enter_y, tape_line_y;   /* line follower is running the junction-style hand-back */
 float recover_side;
 bool front_fresh;
 /* Turn / blocked-front corrections */
 uint8_t turn_fixes_total, block_fixes_total;   /* since the last STOP (telemetry 5/59) */
 uint8_t turn_tries, block_fixes;               /* current action / current request */
 bool turn_recheck;   /* a turn just finished: verify the heading before moving on */
 bool fixing;         /* the running motion is a sonar-triggered extra turn: repeat the same action afterwards */
 int last_turn_dir;   /* +1 left, -1 right: direction of the last turn (the extra turn goes the same way) */
 uint32_t block_since;
 int32_t front_mm;
 unsigned settled,line_seen,index,count,search;
 uint32_t completed_id,started,check_since;
 b5_request_t request;
 full_action_t actions[FULL_NAV_MAX_ACTIONS];
} full_nav_t;
void full_nav_init(full_nav_t *n,const motion_status_t *m);
bool full_nav_accept(full_nav_t *n,const b5_request_t *r,uint32_t now);
bool full_nav_barcode_turn_active(const full_nav_t *n);
bool full_nav_rejoining(const full_nav_t *n);
void full_nav_set_front(full_nav_t *n,bool fresh,int32_t clearance_mm,int angle_deg);
void full_nav_update(full_nav_t *n,uint32_t now,const motion_status_t *m,uint8_t mask,bool guard);
/* Forward range lost while a bypass leg is moving: stop and let the extra-turn correction run instead of halting.
 * Returns false when the correction does not apply (or is used up) and the caller should halt. */
bool full_nav_block_stop(full_nav_t *n);
void full_nav_feedback(const full_nav_t *n,uint32_t timestamp,b5_feedback_t *f);
#endif
