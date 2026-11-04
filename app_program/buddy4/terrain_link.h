/**
 * @file terrain_link.h
 * @brief Glue between Buddy 4 (IMU terrain observer) and Buddy 2 (Motion Control).
 *
 * Buddy 4 only observes and classifies the terrain; Buddy 2 decides the speed
 * and motor power. Call terrain_link_feed() after every buddy4_poll(). Neither
 * buddy's library depends on the other: only this header knows both.
 */
#ifndef TERRAIN_LINK_H
#define TERRAIN_LINK_H

#include "buddy4_imu.h"
#include "motion_control.h"

/** Forward the latest IMU result to Motion Control. A non-fresh or invalid
 *  poll is reported as invalid, so Buddy 2 drives as on flat ground. */
static inline void terrain_link_feed(const buddy4_imu_t *p_imu, buddy4_result_t result)
{
    motion_terrain_input_t input;
    const buddy4_output_t *o = &p_imu->output;

    input.valid = (result == BUDDY4_FRESH) && o->valid && p_imu->ready;
    input.pitch_state = (int8_t)((o->terrain > 0) ? 1 : ((o->terrain < 0) ? -1 : 0));
    input.roll_state = (int8_t)((o->side > 0) ? 1 : ((o->side < 0) ? -1 : 0));
    input.pitch_deg = o->filtered_pitch_deg;
    input.roll_deg = o->filtered_roll_deg;

    if (result == BUDDY4_STALE)
    {
        return;   /* no new sample this poll: keep the last report until it ages out */
    }
    (void)motion_set_terrain(&input);
}

#endif /* TERRAIN_LINK_H */
