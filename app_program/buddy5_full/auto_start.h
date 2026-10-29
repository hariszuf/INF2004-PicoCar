#ifndef B5_AUTO_START_H
#define B5_AUTO_START_H
#include <stdint.h>
#include <stdbool.h>
typedef enum { AUTO_BOOT_DELAY, AUTO_WAIT_READY, AUTO_RUNNING, AUTO_LATCHED } auto_phase_t;
typedef enum { AUTO_NOTHING, AUTO_RESET, AUTO_START } auto_action_t;
typedef struct { auto_phase_t phase; uint32_t since; } auto_start_t;
static inline void auto_start_init(auto_start_t *s,uint32_t now) { s->phase=AUTO_BOOT_DELAY; s->since=now; }
static inline void auto_start_stop(auto_start_t *s) { s->phase=AUTO_LATCHED; }
static inline auto_action_t auto_start_update(auto_start_t *s,uint32_t now,bool stopped,bool ready) {
 if(s->phase==AUTO_LATCHED) return AUTO_NOTHING;
 if(s->phase==AUTO_BOOT_DELAY && (uint32_t)(now-s->since)>=3000u && stopped) {
  s->phase=AUTO_WAIT_READY; s->since=now; return AUTO_RESET;
 }
 if(s->phase==AUTO_WAIT_READY && (uint32_t)(now-s->since)>=700u && stopped && ready) {
  s->phase=AUTO_RUNNING; return AUTO_START;
 }
 return AUTO_NOTHING;
}
#endif