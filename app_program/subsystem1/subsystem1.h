/*
 * subsystem1.h
 * Subsystem 1 (Network Gatekeeper): public interface.
 * Owner: 2502578 (DynamoLiquidDetergent)
 *
 * Only what other subsystems may use goes here.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_TELEM        (22)    /* TEAM.md sec 4: lowest app task */

IMPORT ID g_s1_tid_telem;           /* telemetry task ID (for tests) */

/*
 * TEMPORARY: IF-05 compatibility for S4's example monitor, which
 * suspends/resumes tid_led.  tid_led is a stub task that only sleeps.
 * Set to 0 and DELETE once S4 removes IF-05 code.
 */
#define S1_IF05_COMPAT      (1)
#if S1_IF05_COMPAT
#define S1_PRI_LED          (S1_PRI_TELEM)
IMPORT ID tid_led;
#endif

#endif /* SUBSYSTEM1_H */