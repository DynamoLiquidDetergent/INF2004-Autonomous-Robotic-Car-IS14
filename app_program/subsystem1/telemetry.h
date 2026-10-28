/*
 * telemetry.h
 * Subsystem 1: telemetry task (private to subsystem 1).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "../app.h"

#define TELEM_TICK_MS        (100U)  /* base loop period, ms            */
#define TELEM_FRAME_TICKS    (5U)    /* telemetry frame every 500 ms    */
#define TELEM_HB_TICKS       (10U)   /* heartbeat every 1 s             */
#define TELEM_RX_PER_TICK    (4U)    /* max commands/maps handled a tick */

/* 1 = print every frame sent as hex on the USB console (debug) */
#define TELEM_DEBUG_HEX      (1)

IMPORT void telemetry_task(INT stacd, void *exinf);

#endif /* TELEMETRY_H */