#ifndef B5_COMPAT_CLOCKS_H
#define B5_COMPAT_CLOCKS_H
#include <stdint.h>
enum { clk_sys=0 };
static inline uint32_t clock_get_hz(int clock) { (void)clock; return 125000000u; }
#endif
