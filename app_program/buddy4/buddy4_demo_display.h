#ifndef BUDDY4_DEMO_DISPLAY_H
#define BUDDY4_DEMO_DISPLAY_H

#include "buddy4_imu.h"
#include <stddef.h>

/* Display-only labels; never feed these back into the IMU classifier. */
typedef struct {
    bool has_last;
    int last_side;
    bool stable_valid;
    float stable_height_cm;
    bool median_valid;
    float median_height_cm;
    bool highest_valid;
    float highest_height_cm;
    uint64_t last_event_timestamp_us;
} buddy4_demo_state_t;

const char *buddy4_demo_pitch_direction(float pitch_deg);
const char *buddy4_demo_roll_direction(float roll_deg);
void buddy4_demo_format(char *buffer, size_t size, buddy4_result_t result,
                        const buddy4_output_t *output);
bool buddy4_demo_record_event(buddy4_demo_state_t *state,
                              buddy4_result_t result,
                              const buddy4_output_t *output);
void buddy4_demo_format_full(char *buffer, size_t size, buddy4_result_t result,
                             const buddy4_output_t *output,
                             const buddy4_demo_state_t *state);
void buddy4_demo_format_event(char *buffer, size_t size,
                              const buddy4_demo_state_t *state);

#endif
