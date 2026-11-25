#include "../../app_program/barrc_types.h"
#include <assert.h>
#include <stdio.h>
#include "../../app_program/buddy5/obstacle_state.h"

static buddy5_state_t sample (buddy5_state_t state, uint32_t mm)
{
    ultrasonic_result_t reading = {US_OK, 0u, mm};
    return buddy5_next_state(state, reading);
}

int main (void)
{
    assert(BUDDY5_SENSOR_FAULT == sample(BUDDY5_SENSOR_FAULT, 225));
    assert(BUDDY5_OBSTACLE == sample(BUDDY5_SENSOR_FAULT, 200));
    assert(BUDDY5_CLEAR == sample(BUDDY5_SENSOR_FAULT, 250));
    assert(BUDDY5_OBSTACLE == sample(BUDDY5_CLEAR, 200));
    assert(BUDDY5_OBSTACLE == sample(BUDDY5_OBSTACLE, 201));
    assert(BUDDY5_OBSTACLE == sample(BUDDY5_OBSTACLE, 249));
    assert(BUDDY5_CLEAR == sample(BUDDY5_OBSTACLE, 250));
    assert(BUDDY5_CLEAR == sample(BUDDY5_CLEAR, 249));
    assert(BUDDY5_CLEAR == sample(BUDDY5_CLEAR, 201));

    for (int32_t status = US_ECHO_STUCK_HIGH; status <= US_OUT_OF_RANGE;
         ++status)
    {
        ultrasonic_result_t bad = {(ultrasonic_status_t)status, 0u, 0u};
        assert(BUDDY5_SENSOR_FAULT == buddy5_next_state(BUDDY5_CLEAR, bad));
        assert(BUDDY5_SENSOR_FAULT == buddy5_next_state(BUDDY5_OBSTACLE, bad));
    }

    puts("Buddy 5 obstacle state tests passed");
    return 0;
}

/*** end of file ***/
