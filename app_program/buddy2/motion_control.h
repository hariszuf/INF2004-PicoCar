/**
 * @file motion_control.h
 * @brief Public API of the Motion Control subsystem (Buddy 2).
 *
 * Owned by the Motion Control RTOS task. Other tasks (Vehicle Controller,
 * Safety Monitor) use only the functions in this header; they never touch
 * the motors or encoders directly.
 *
 * Usage:
 *   - Call motion_init() once at start-up.
 *   - Call motion_update() from the Motion Control task every
 *     MOTION_UPDATE_PERIOD_MS.
 *   - Movement commands are non-blocking. They are queued and applied at the
 *     next motion_update(). Poll motion_is_idle() or motion_get_status() to
 *     see when a move has finished.
 *
 * State machine (matches the Week 6 statechart):
 *   IDLE -> EXECUTING -> STOPPING -> IDLE, and any state -> ERROR.
 *   ERROR stops and latches the motors until motion_reset() is called.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/** Result of an API call. */
typedef enum
{
    MOTION_OK = 0,
    MOTION_ERR_NOT_INIT,
    MOTION_ERR_INVALID_ARG,
    MOTION_ERR_FAULT,       /* subsystem is in ERROR, call motion_reset() */
    MOTION_ERR_BUSY         /* operation needs the subsystem to be idle */
} motion_result_t;

/** Motion Control state (Week 6 Motion Control statechart). */
typedef enum
{
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_EXECUTING,
    MOTION_STATE_STOPPING,
    MOTION_STATE_ERROR
} motion_state_t;

/** Reason for the ERROR state. */
typedef enum
{
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_ESTOP,     /* motion_emergency_stop() was called */
    MOTION_FAULT_STALL      /* motor driven but wheel not turning */
} motion_fault_t;

/** Terrain condition Buddy 2 is currently compensating for. */
typedef enum
{
    MOTION_TERRAIN_FLAT = 0,
    MOTION_TERRAIN_UPHILL,
    MOTION_TERRAIN_DOWNHILL,
    MOTION_TERRAIN_HUMP_LEFT,   /* left wheel raised (on a hump / one-side ramp) */
    MOTION_TERRAIN_HUMP_RIGHT   /* right wheel raised */
} motion_terrain_t;

/**
 * Terrain report from the IMU (Buddy 4). Buddy 4 only observes; Buddy 2
 * decides the speed and power. pitch_state / roll_state are Buddy 4's
 * hysteresis outputs (buddy4_output_t.terrain / .side).
 */
typedef struct
{
    bool   valid;         /* false = no usable IMU data, treat as flat */
    int8_t pitch_state;   /* -1 downhill, 0 flat, +1 uphill */
    int8_t roll_state;    /* -1 left wheel up, 0 level, +1 right wheel up */
    float  pitch_deg;     /* filtered pitch, positive = nose up */
    float  roll_deg;      /* filtered roll */
} motion_terrain_input_t;

/** Snapshot for telemetry and the Vehicle Controller. */
typedef struct
{
    motion_state_t state;
    motion_fault_t fault;
    int32_t  left_count;        /* signed encoder counts since odometry reset */
    int32_t  right_count;
    float    left_speed_cm_s;   /* filtered, signed (negative = backward) */
    float    right_speed_cm_s;
    float    speed_cm_s;        /* average of both wheels */
    float    distance_cm;       /* signed distance since odometry reset */
    float    heading_deg;       /* signed, left turn positive */
    motion_terrain_t terrain;   /* condition being compensated (FLAT if stale) */
    float    terrain_speed_scale;     /* multiplier applied to speed targets */
    float    left_terrain_duty;       /* duty assist added on top of the speed loop */
    float    right_terrain_duty;
} motion_status_t;

/** Initialise motors, encoders and controllers. Safe to call twice. */
motion_result_t motion_init(void);

/** Periodic update. Must be called every MOTION_UPDATE_PERIOD_MS. */
void motion_update(void);

/** Drive forward by distance_cm (> 0) in a straight line. */
motion_result_t motion_move_forward(float distance_cm);

/** Drive backward by distance_cm (> 0) in a straight line. */
motion_result_t motion_move_backward(float distance_cm);

/** Spin left in place by angle_deg (> 0, up to 720). */
motion_result_t motion_turn_left(float angle_deg);

/** Spin right in place by angle_deg (> 0, up to 720). */
motion_result_t motion_turn_right(float angle_deg);

/**
 * Continuous drive with independent wheel speeds in cm/s (signed, forward
 * positive). Runs until another command or motion_stop(). Intended for line
 * following steering. Both speeds zero is treated as motion_stop().
 */
motion_result_t motion_drive(float left_cm_s, float right_cm_s);

/** Cruise speed used by later forward / backward moves (cm/s, > 0). */
motion_result_t motion_set_speed(float speed_cm_s);

/**
 * Latest terrain report from the IMU task (Buddy 4). Call after every fresh
 * IMU sample (about every 20 ms). Buddy 2 then scales the speed of forward /
 * backward moves and motion_drive() (so Buddy 3 line following slows down
 * automatically) and adds a gravity / hump power assist. If no report arrives
 * for MOTION_TERRAIN_STALE_MS, or valid is false, the car behaves as on flat
 * ground. Safe to call from any task.
 */
motion_result_t motion_set_terrain(const motion_terrain_input_t *p_input);

/** Controlled stop: brake and return to IDLE. */
void motion_stop(void);

/**
 * Emergency stop. Brakes both motors immediately from the caller's context
 * and latches them off, then the subsystem enters ERROR (fault ESTOP).
 * Movement commands cannot override it. Safe to call from any task.
 */
void motion_emergency_stop(void);

/** Leave ERROR after the cause is cleared. No-op outside ERROR. */
motion_result_t motion_reset(void);

/** Zero distance and heading. Only allowed while IDLE. */
motion_result_t motion_reset_odometry(void);

/** True when no move is running or queued. */
bool motion_is_idle(void);

/** Copy the current status. p_status must not be null. */
void motion_get_status(motion_status_t *p_status);

#endif /* MOTION_CONTROL_H */
