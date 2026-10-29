#ifndef FULL_NAV_H
#define FULL_NAV_H
#include <stdint.h>
#include <stdbool.h>
#include "buddy5_core.h"
#include "../buddy2/motion_control.h"
typedef struct { unsigned kind; float value; } full_action_t;
typedef struct {
 float x,y,heading,last_distance,last_heading,heading_origin;
 bool observed,active,issued,moving,forward_moving,motion_seen,failed,resumed;
 unsigned settled,line_seen,index,count,search;
 uint32_t completed_id,started,check_since;
 b5_request_t request;
 full_action_t actions[4];
} full_nav_t;
void full_nav_init(full_nav_t *n,const motion_status_t *m);
bool full_nav_accept(full_nav_t *n,const b5_request_t *r,uint32_t now);
void full_nav_update(full_nav_t *n,uint32_t now,const motion_status_t *m,uint8_t mask,bool guard);
void full_nav_feedback(const full_nav_t *n,uint32_t timestamp,b5_feedback_t *f);
#endif
