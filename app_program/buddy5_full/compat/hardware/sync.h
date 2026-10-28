#ifndef B5_COMPAT_SYNC_H
#define B5_COMPAT_SYNC_H
#include <stdint.h>
static inline uint32_t save_and_disable_interrupts(void) {
 uint32_t old;
 __asm__ volatile("mrs %0, primask\n cpsid i" : "=r"(old) : : "memory");
 return old;
}
static inline void restore_interrupts(uint32_t old) {
 __asm__ volatile("msr primask, %0" : : "r"(old) : "memory");
}
#endif