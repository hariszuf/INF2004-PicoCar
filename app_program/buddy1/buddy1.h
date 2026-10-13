
#ifndef BUDDY1_H
#define BUDDY1_H

#include <tk/tkernel.h>

/*
 * Buddy 1: Communication, Command and Telemetry
 *
 * Public subsystem interface.
 *
 * Phase 1 creates shared kernel objects.
 * Phase 2 starts application tasks.
 */

/*
 * Phase 1: Initialise Buddy 1 resources.
 *
 * Must be called before buddy1_start().
 */
ER buddy1_init(void);

/*
 * Phase 2: Start Buddy 1 tasks.
 *
 * All subsystem initialisation must already
 * be completed before this function is called.
 */
ER buddy1_start(void);

#endif /* BUDDY1_H */
