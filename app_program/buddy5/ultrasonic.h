#ifndef BUDDY5_ULTRASONIC_H
#define BUDDY5_ULTRASONIC_H
#include <stdint.h>

typedef enum {
    US_OK = 0,
    US_ECHO_STUCK_HIGH,
    US_RISE_TIMEOUT,
    US_FALL_TIMEOUT,
    US_OUT_OF_RANGE
} ultrasonic_status_t;

typedef struct {
    ultrasonic_status_t status;
    uint32_t echo_us;
    uint32_t distance_mm;
} ultrasonic_result_t;

/* GP28 = TRIG, GP7 = ECHO (VERIFY WIRING AND LEVEL SHIFTING FIRST). */
void ultrasonic_init(void);
ultrasonic_result_t ultrasonic_read(void);
/* Integer conversion, rounded to nearest millimetre at 343 m/s. */
uint32_t ultrasonic_echo_to_mm(uint32_t echo_us);
#endif
