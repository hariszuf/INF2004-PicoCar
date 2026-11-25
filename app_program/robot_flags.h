/*
 * robot_flags - several yes/no flags packed into one 32-bit word, safe to share
 * between tasks and ISRs.
 *
 * Why a lock: setting or clearing one bit is a read-modify-write. Two writers
 * (for example the controller task and
 * the MQTT command path) could otherwise overwrite each other's bit. Every
 * writer therefore runs with interrupts
 * masked (DI/EI, the same critical section the controller already uses).
 * Reading one aligned 32-bit word is a
 * single load on Cortex-M, so a plain read needs no lock.
 *
 * The *_nl ("no lock") variants are for code that is already inside a DI/EI
 * section.
 */
#ifndef ROBOT_FLAGS_H
#define ROBOT_FLAGS_H

#include <stdbool.h>
#include <stdint.h>

#ifndef ROBOT_FLAGS_LOCK
#include <tk/tkernel.h>
#define ROBOT_FLAGS_IRQ_T UW
#define ROBOT_FLAGS_LOCK(s) DI(s)
#define ROBOT_FLAGS_UNLOCK(s) EI(s)
#endif

typedef volatile uint32_t robot_flags_t;

static inline bool robot_flags_test (const robot_flags_t * f, uint32_t mask)
{
    return ((*f) & mask) != 0u;
}

static inline void robot_flags_set_nl (robot_flags_t * f, uint32_t mask)
{
    *f |= mask;
}

static inline void robot_flags_clear_nl (robot_flags_t * f, uint32_t mask)
{
    *f &= ~mask;
}

static inline void robot_flags_set (robot_flags_t * f, uint32_t mask)
{
    ROBOT_FLAGS_IRQ_T s;
    ROBOT_FLAGS_LOCK(s);
    *f |= mask;
    ROBOT_FLAGS_UNLOCK(s);
}

static inline void robot_flags_clear (robot_flags_t * f, uint32_t mask)
{
    ROBOT_FLAGS_IRQ_T s;
    ROBOT_FLAGS_LOCK(s);
    *f &= ~mask;
    ROBOT_FLAGS_UNLOCK(s);
}

/* Set or clear the bits in mask according to on. */
static inline void robot_flags_assign (robot_flags_t * f, uint32_t mask,
                                       bool on)
{
    ROBOT_FLAGS_IRQ_T s;
    ROBOT_FLAGS_LOCK(s);
    *f = on ? (*f | mask) : (*f & ~mask);
    ROBOT_FLAGS_UNLOCK(s);
}

/* Replace every bit covered by mask with the same bits of value, in one step
 * (no window where a reader sees a
 * half-updated word). */
static inline void robot_flags_update (robot_flags_t * f, uint32_t mask,
                                       uint32_t value)
{
    ROBOT_FLAGS_IRQ_T s;
    ROBOT_FLAGS_LOCK(s);
    *f = (*f & ~mask) | (value & mask);
    ROBOT_FLAGS_UNLOCK(s);
}

/* Return whether any bit in mask was set, and clear those bits, in one step. */
static inline bool robot_flags_test_and_clear (robot_flags_t * f, uint32_t mask)
{
    ROBOT_FLAGS_IRQ_T s;
    bool was;
    ROBOT_FLAGS_LOCK(s);
    was = ((*f) & mask) != 0u;
    *f &= ~mask;
    ROBOT_FLAGS_UNLOCK(s);
    return was;
}

#endif /* ROBOT_FLAGS_H */

/*** end of file ***/
