/**
 * @file line_follow.h
 * @brief Two-sensor line following on top of the Buddy 2 Motion Control library.
 *
 * Call line_follow_update() every LINE_UPDATE_PERIOD_MS right after
 * motion_update(). All wheel control goes through motion_drive().
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LINE_STATE_IDLE = 0,     /* not started */
    LINE_STATE_STRAIGHT,     /* both sensors on the line */
    LINE_STATE_STEER_LEFT,   /* only the left sensor on the line */
    LINE_STATE_STEER_RIGHT,  /* only the right sensor on the line */
    LINE_STATE_RECOVERING,   /* line just lost, turning toward the last side */
    LINE_STATE_STOPPED,      /* no line, motors stopped (resumes if the line returns) */
    LINE_STATE_FAULT         /* Motion Control reported an error */
} line_state_t;

typedef struct
{
    line_state_t state;
    uint8_t sensor_mask;     /* LINE_SENSOR_* bits */
    float left_cmd_cm_s;
    float right_cmd_cm_s;
} line_status_t;

/** Motion Control must already be initialised (motion_init()). */
void line_follow_init(void);
bool line_follow_start(void);
void line_follow_stop(void);
void line_follow_update(void);
void line_follow_get_status(line_status_t *p_status);

#endif /* LINE_FOLLOW_H */
