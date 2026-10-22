#include "drive_control.h"
#include "line_follow.h"
#include "motion_config.h"
#include <math.h>
void drive_stop(drive_control_t *c, const char *why, motion_fault_t fault) {
    line_follow_stop();
    motion_emergency_stop();
    c->running = c->armed = false;
    c->reason = why; c->stop_fault = fault; c->event_pending = true;
}
void drive_init(drive_control_t *c) {
    *c = (drive_control_t){.mode = DRIVE_MANUAL};
    c->selected_left_cm_s = c->selected_right_cm_s = MANUAL_SPEED_CM_S;
    drive_stop(c, "BOOT", MOTION_FAULT_NONE);
}
void drive_safety(drive_control_t *c, bool usb, bool has_fresh,
                  uint64_t now, uint64_t last_fresh) {
    motion_status_t s; motion_get_status(&s);
    if (!usb && (c->running || c->armed))
        drive_stop(c, "USB_DISCONNECTED", MOTION_FAULT_NONE);
    else if ((c->running || c->armed) &&
             (!has_fresh || now - last_fresh >= INVALID_LIMIT_US))
        drive_stop(c, "IMU_INVALID_TIMEOUT", MOTION_FAULT_NONE);
    else if ((c->running || c->armed) &&
             (s.fault != MOTION_FAULT_NONE || s.state == MOTION_STATE_ERROR))
        drive_stop(c, "MOTOR_FAULT", s.fault);
    else if (c->running && (c->mode == DRIVE_TURN_LEFT || c->mode == DRIVE_TURN_RIGHT) &&
             (now < c->started_us || now - c->started_us >= TURN_TEST_LIMIT_US))
        drive_stop(c, "TURN_TEST_TIMEOUT", MOTION_FAULT_NONE);
}
void drive_command(drive_control_t *c, int key, bool usb, bool healthy, uint64_t now) {
    if (key == 'S' || key == ' ') {
        drive_stop(c, "SERIAL_STOP", MOTION_FAULT_NONE); return;
    }
    if (!usb) return;
    if ((key == 'M' || key == 'L' || key == 'A' || key == 'D') && !c->running) {
        drive_stop(c, "MODE_SELECTED", MOTION_FAULT_NONE);
        c->mode = key == 'M' ? DRIVE_MANUAL : key == 'L' ? DRIVE_LINE :
                  key == 'A' ? DRIVE_TURN_LEFT : DRIVE_TURN_RIGHT;
        c->selected_left_cm_s = c->mode == DRIVE_MANUAL ? MANUAL_SPEED_CM_S :
            c->mode == DRIVE_TURN_LEFT ? -TURN_TEST_SPEED_CM_S :
            c->mode == DRIVE_TURN_RIGHT ? TURN_TEST_SPEED_CM_S : 0;
        c->selected_right_cm_s = c->mode == DRIVE_MANUAL ? MANUAL_SPEED_CM_S :
            c->mode == DRIVE_TURN_LEFT ? TURN_TEST_SPEED_CM_S :
            c->mode == DRIVE_TURN_RIGHT ? -TURN_TEST_SPEED_CM_S : 0;
        return;
    }
    motion_status_t s; motion_get_status(&s);
    if (key == 'R' && !c->running && healthy && s.state == MOTION_STATE_ERROR) {
        if (motion_reset() == MOTION_OK) {
            c->armed = true; c->reason = "ARMED";
            c->stop_fault = MOTION_FAULT_NONE; c->event_pending = true;
        }
    } else if (key == 'G' && !c->running && c->armed && healthy &&
               s.state == MOTION_STATE_IDLE) {
        if (c->mode != DRIVE_MANUAL && c->mode != DRIVE_LINE &&
            (!isfinite(TURN_TEST_SPEED_CM_S) || TURN_TEST_SPEED_CM_S <= 0 ||
             TURN_TEST_SPEED_CM_S > MOTION_MAX_SPEED_CM_S)) {
            drive_stop(c, "INVALID_TURN_TARGET", MOTION_FAULT_NONE); return;
        }
        bool ok = c->mode == DRIVE_LINE ? line_follow_start() :
            motion_drive(c->selected_left_cm_s, c->selected_right_cm_s) == MOTION_OK;
        if (ok) {
            c->started_us = now;
            c->running = true; c->armed = false;
            c->reason = "START"; c->event_pending = true;
        } else drive_stop(c, "START_FAILED", s.fault);
    }
}
void drive_line_update(drive_control_t *c) {
    /* Buddy 3 updates run only in explicitly selected line mode. */
    if (!c->running || c->mode != DRIVE_LINE) return;
    line_follow_update();
    line_status_t s; line_follow_get_status(&s);
    if (s.state == LINE_STATE_STOPPED) drive_stop(c, "LINE_LOST", MOTION_FAULT_NONE);
    else if (s.state == LINE_STATE_FAULT) {
        motion_status_t m; motion_get_status(&m);
        drive_stop(c, "MOTOR_FAULT", m.fault);
    }
}
