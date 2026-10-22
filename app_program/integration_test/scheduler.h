#ifndef INTEGRATION_SCHEDULER_H
#define INTEGRATION_SCHEDULER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint64_t next;
    uint32_t period, skipped, late_max_us, runtime_max_us;
} task_t;
/* Phase locked; execute once, skip obsolete slots, never burst catch up. */
static inline bool due(task_t *t, uint64_t now) {
    if (now < t->next) return false;
    uint64_t late = now - t->next;
    if (late > t->late_max_us) t->late_max_us = (uint32_t)late;
    t->skipped += (uint32_t)(late / t->period);
    t->next += (late / t->period + 1) * t->period;
    return true;
}
#endif
