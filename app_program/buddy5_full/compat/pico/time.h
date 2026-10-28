#ifndef B5_COMPAT_TIME_H
#define B5_COMPAT_TIME_H
#include <stdint.h>
uint64_t b5p_time_us_64(void);
#define time_us_64 b5p_time_us_64
#endif