/**
 * @file host_platform.h
 * @brief Host-only stand-ins for Pico time, used by the simulation test.
 */
#ifndef HOST_PLATFORM_H
#define HOST_PLATFORM_H

#include <stdint.h>

/** Simulated time in microseconds. */
uint64_t host_time_us(void);

#endif /* HOST_PLATFORM_H */
