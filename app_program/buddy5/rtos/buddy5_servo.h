#ifndef BUDDY5_SERVO_H
#define BUDDY5_SERVO_H
#include <stdbool.h>
#include <stdint.h>
bool buddy5_servo_init(void);
bool buddy5_servo_set_angle(int degrees);
uint32_t buddy5_clock_ms(void);
#endif
