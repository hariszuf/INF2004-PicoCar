#ifndef B5_SCAN_LINK_H
#define B5_SCAN_LINK_H
#include <stdint.h>
#include <stdbool.h>
#include "../buddy5/buddy5_core.h"
typedef struct {
 b5_state_t state;
 b5_fault_t fault;
 b5_profile_t profile;
 int32_t front_clearance_mm;
 int angle;
 bool front_valid;
 uint32_t front_timestamp_ms, heartbeat_ms;
} line_scan_status_t;
int32_t line_scan_start(void);
void line_scan_reset(void);
void line_scan_abort(void);
void line_scan_get_status(line_scan_status_t *status);
bool line_controller_feedback(b5_feedback_t *feedback);
bool line_controller_request(const b5_request_t *request);
#endif
