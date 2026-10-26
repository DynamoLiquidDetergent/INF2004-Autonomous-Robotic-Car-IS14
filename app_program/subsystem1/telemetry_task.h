/*
 * telemetry_task.h
 * Subsystem 1: telemetry task (private to subsystem 1).
 */

#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include "../app.h"

#define TELEM_TICK_MS       (100U)  /* base loop period, ms */
#define TELEM_ALIVE_TICKS   (10U)   /* Initial: print every 1 s */

IMPORT void telemetry_task(INT stacd, void *exinf);

#endif /* TELEMETRY_TASK_H */