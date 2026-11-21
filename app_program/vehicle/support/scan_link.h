#ifndef B5_SCAN_LINK_H
#define B5_SCAN_LINK_H
#include <stdint.h>
#include <stdbool.h>
#include "../../buddy5/buddy5_core.h"
typedef struct {
 b5_state_t state;
 b5_fault_t fault;
 b5_profile_t profile;
 int32_t front_clearance_mm;
 int angle;
 bool front_valid;
 uint32_t front_timestamp_ms, heartbeat_ms;
 int32_t raw_range_mm;      /* last single ping, mm, -1 = none/stale */
 int32_t raw_status;        /* 0 valid, 1 stuck high, 2 no echo, 3 incomplete echo, 4 out of range, -1 stale */
 uint32_t obstacle_id;      /* obstacles handled since boot (counts when a stop is triggered) */
 uint8_t rescans;           /* sides found blocked and rescanned during the current obstacle */
} line_scan_status_t;
uint32_t buddy5_clock_ms(void);   /* ms since boot (b5_stub.c) */
int32_t line_scan_start(void);
void line_scan_reset(void);
void line_scan_abort(void);
void line_scan_get_status(line_scan_status_t *status);
bool line_controller_feedback(b5_feedback_t *feedback);
bool line_controller_request(const b5_request_t *request);
#endif
