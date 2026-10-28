/*
 * telemetry.h
 * Subsystem 1: telemetry task (private to subsystem 1).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "../app.h"

#define TELEM_TICK_MS        (100U)  /* base loop period, ms             */
#define TELEM_FRAME_TICKS    (5U)    /* telemetry frame every 500 ms     */
#define TELEM_HB_TICKS       (10U)   /* heartbeat every 1 s              */
#define TELEM_RX_PER_TICK    (4U)    /* max commands/maps handled a tick */
#define TELEM_REPLAY_PER_TICK (1U)   /* outage replay: 1 frame / 100 ms  */

/* WiFi driver's "joined and has an IP" value (CYW43_LINK_UP in cyw43.h,
   not included here because it pulls in the Pico SDK headers). */
#define TELEM_WIFI_LINK_UP   (3)

IMPORT void telemetry_task(INT stacd, void *exinf);

#if APP_TEST
/* Tests only: force the link state the task sees. */
#define TELEM_LINK_AUTO      (-1)    /* use the real link (default) */
#define TELEM_LINK_DOWN      (0)
#define TELEM_LINK_UP        (1)
IMPORT void telem_test_force_link(INT mode);
#endif

#endif /* TELEMETRY_H */