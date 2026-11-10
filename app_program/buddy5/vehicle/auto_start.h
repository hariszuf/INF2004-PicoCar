#ifndef B5_AUTO_START_H
#define B5_AUTO_START_H
#include <stdint.h>
#include <stdbool.h>
/* Start/stop policy of the integrated vehicle controller (pure logic, host-tested).
 *   BOOT_DELAY -> WAIT_READY -> RUNNING       automatic start after the applicable startup gates
 *   any phase  -> LATCHED  (stopped; recoverable; only a new START leaves it)
 *   any phase  -> FAULTED  (needs the operator: motor stall, motor reset failure; only a power cycle leaves it)
 * A START intent never resets a running maneuver; a STOP always wins. */
typedef enum { AUTO_BOOT_DELAY, AUTO_WAIT_READY, AUTO_RUNNING, AUTO_LATCHED, AUTO_FAULTED } auto_phase_t;
typedef enum { AUTO_NOTHING, AUTO_RESET, AUTO_START } auto_action_t;
typedef enum { START_NONE=0, START_ACCEPTED=1, START_ALREADY_ACTIVE=2, START_REJECTED_FAULT=3 } auto_start_result_t;
/* Execution state reported to the dashboard (telemetry src5 type 60 value_1). */
typedef enum { EXEC_STARTUP_WAIT=0, EXEC_WAIT_READY=1, EXEC_RUNNING=2, EXEC_STOPPED=3, EXEC_RESTART_PENDING=4, EXEC_FAULTED=5 } auto_exec_t;
typedef struct {
 auto_phase_t phase; uint32_t since;
 bool restarting;              /* a START after a stop is waiting for the readiness gates */
 int32_t reason;               /* halt reason code of the last stop/fault (0 = none) */
 auto_start_result_t last_start; uint32_t start_count;
 uint32_t stopped_at;          /* when the last stop/fault happened */
 uint32_t auto_retries;        /* automatic restarts after an uncommanded, recoverable stop */
} auto_start_t;
#ifndef B5_BOOT_DELAY_MS
#define B5_BOOT_DELAY_MS 3000u
#endif
/* An uncommanded, recoverable stop restarts the whole process automatically after this delay (ms).
 * A user STOP and a persistent fault (motor stall, motor reset failure) never auto-restart. 0 = off. */
#ifndef B5_AUTO_RETRY_MS
#define B5_AUTO_RETRY_MS 3000u
#endif
#define B5_HALT_USER_STOP 1
#ifndef B5_READY_DELAY_MS
#define B5_READY_DELAY_MS 700u
#endif
static inline void auto_start_init(auto_start_t *s,uint32_t now) {
 s->phase=AUTO_BOOT_DELAY; s->since=now; s->restarting=false; s->reason=0; s->last_start=START_NONE; s->start_count=0;
 s->stopped_at=now; s->auto_retries=0;
}
/* STOP (dashboard, USB S, Space). Never downgrades a fault; clears any pending restart. */
static inline void auto_start_stop(auto_start_t *s) {
 if(s->phase==AUTO_FAULTED) return;
 s->phase=AUTO_LATCHED; s->restarting=false; s->reason=B5_HALT_USER_STOP;
}
/* The controller stopped itself. persistent=true: stays stopped until power cycle. */
static inline void auto_start_halt(auto_start_t *s,int32_t reason,bool persistent,uint32_t now) {
 if(s->phase==AUTO_FAULTED) return;
 s->phase=persistent?AUTO_FAULTED:AUTO_LATCHED; s->restarting=false; s->reason=reason; s->stopped_at=now;
}
/* START (dashboard) / USB R: a restart request. now must be the controller clock. */
static inline auto_start_result_t auto_start_request(auto_start_t *s,uint32_t now) {
 s->start_count++;
 if(s->phase==AUTO_FAULTED) return s->last_start=START_REJECTED_FAULT;
 if(s->phase!=AUTO_LATCHED) return s->last_start=START_ALREADY_ACTIVE;   /* starting or running: do nothing */
 s->phase=AUTO_BOOT_DELAY; s->since=now-B5_BOOT_DELAY_MS; s->restarting=true;   /* reset now, then wait for readiness */
 return s->last_start=START_ACCEPTED;
}
/* Call every controller tick: restarts the whole process after an uncommanded, recoverable stop.
 * Returns true when it issued the restart (same path as START). */
static inline bool auto_start_auto_retry(auto_start_t *s,uint32_t now,bool stopped) {
 if(!B5_AUTO_RETRY_MS || s->phase!=AUTO_LATCHED || s->reason==B5_HALT_USER_STOP || s->reason==0 || !stopped) return false;
 if((uint32_t)(now-s->stopped_at)<B5_AUTO_RETRY_MS) return false;
 s->auto_retries++;
 s->phase=AUTO_BOOT_DELAY; s->since=now-B5_BOOT_DELAY_MS; s->restarting=true;
 return true;
}
static inline auto_exec_t auto_start_exec_state(const auto_start_t *s) {
 switch(s->phase) {
 case AUTO_BOOT_DELAY: return s->restarting?EXEC_RESTART_PENDING:EXEC_STARTUP_WAIT;
 case AUTO_WAIT_READY: return s->restarting?EXEC_RESTART_PENDING:EXEC_WAIT_READY;
 case AUTO_RUNNING: return EXEC_RUNNING;
 case AUTO_LATCHED: return EXEC_STOPPED;
 default: return EXEC_FAULTED;
 }
}
static inline auto_action_t auto_start_update(auto_start_t *s,uint32_t now,bool stopped,bool ready) {
 if(s->phase==AUTO_LATCHED || s->phase==AUTO_FAULTED) return AUTO_NOTHING;
 if(s->phase==AUTO_BOOT_DELAY && (uint32_t)(now-s->since)>=B5_BOOT_DELAY_MS && stopped) {
  s->phase=AUTO_WAIT_READY; s->since=now; return AUTO_RESET;
 }
 if(s->phase==AUTO_WAIT_READY && (uint32_t)(now-s->since)>=B5_READY_DELAY_MS && stopped && ready) {
  s->phase=AUTO_RUNNING; s->restarting=false; return AUTO_START;
 }
 return AUTO_NOTHING;
}
#endif
