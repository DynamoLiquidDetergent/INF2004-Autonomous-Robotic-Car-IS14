/*
 * telemetry_task.c
 * Subsystem 1: telemetry task.
 *
 * Initial: runs a drift-free 100 ms loop and prints "alive" every 1 s.
 * Deadlines are computed from tk_get_tim(), so the period does not
 * drift however long each pass takes.
 */

#include <stdint.h>
#include "telemetry_task.h"

EXPORT void telemetry_task(INT stacd, void *exinf)
{
    SYSTIM   now;
    uint32_t next_ms;
    uint32_t tick = 0U;
    int32_t  wait_ms;

    (void)stacd;
    (void)exinf;

    (void)tk_get_tim(&now);
    next_ms = now.lo + TELEM_TICK_MS;

    for (;;)
    {
        (void)tk_get_tim(&now);
        wait_ms = (int32_t)(next_ms - now.lo);   /* wrap-safe */

        if (wait_ms > 0)
        {
            (void)tk_dly_tsk((RELTIM)wait_ms);
        }
        else if (wait_ms < -(int32_t)TELEM_TICK_MS)
        {
            next_ms = now.lo;                    /* far behind: resync */
        }
        else
        {
            /* slightly late: run now, keep the schedule */
        }

        next_ms += TELEM_TICK_MS;
        tick++;

        if ((tick % TELEM_ALIVE_TICKS) == 0U)
        {
            (void)tk_get_tim(&now);
            tm_printf((UB *)"s1: alive tick=%u t=%u ms\n", tick, now.lo);
        }
    }
}