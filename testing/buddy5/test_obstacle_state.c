#include <assert.h>
#include <stdio.h>
#include "../../app_program/buddy5/obstacle_state.h"

static buddy5_state_t sample(buddy5_state_t state, unsigned mm)
{
    ultrasonic_result_t reading = {US_OK, 0u, mm};
    return buddy5_next_state(state, reading);
}

int main(void)
{
    assert(sample(BUDDY5_SENSOR_FAULT, 225) == BUDDY5_SENSOR_FAULT);
    assert(sample(BUDDY5_SENSOR_FAULT, 200) == BUDDY5_OBSTACLE);
    assert(sample(BUDDY5_SENSOR_FAULT, 250) == BUDDY5_CLEAR);
    assert(sample(BUDDY5_CLEAR, 200) == BUDDY5_OBSTACLE);
    assert(sample(BUDDY5_OBSTACLE, 201) == BUDDY5_OBSTACLE);
    assert(sample(BUDDY5_OBSTACLE, 249) == BUDDY5_OBSTACLE);
    assert(sample(BUDDY5_OBSTACLE, 250) == BUDDY5_CLEAR);
    assert(sample(BUDDY5_CLEAR, 249) == BUDDY5_CLEAR);
    assert(sample(BUDDY5_CLEAR, 201) == BUDDY5_CLEAR);
    for (int status = US_ECHO_STUCK_HIGH; status <= US_OUT_OF_RANGE; ++status) {
        ultrasonic_result_t bad = {(ultrasonic_status_t)status, 0u, 0u};
        assert(buddy5_next_state(BUDDY5_CLEAR, bad) == BUDDY5_SENSOR_FAULT);
        assert(buddy5_next_state(BUDDY5_OBSTACLE, bad) == BUDDY5_SENSOR_FAULT);
    }
    puts("Buddy 5 obstacle state tests passed");
    return 0;
}
