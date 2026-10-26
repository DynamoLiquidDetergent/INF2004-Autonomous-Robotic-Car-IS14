/*
 * subsystem1.c
 * Subsystem 1 (Network Gatekeeper): creates objects, starts tasks.
 */

#include "subsystem1.h"
#include "telemetry_task.h"

EXPORT ID g_s1_tid_telem = 0;

#if S1_IF05_COMPAT
EXPORT ID tid_led = 0;

/* Stub for S4's IF-05: sleeps forever, safe to suspend/resume. DELETE when S4 removes IF-05 code */
static void if05_stub_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
#endif

/* Create kernel objects.  Message buffers are added in step 2. */
EXPORT ER subsystem1_init(void)
{
    return E_OK;
}

/* Create and start subsystem 1's tasks. */
EXPORT ER subsystem1_start(void)
{
    ER er = E_OK;

    g_s1_tid_telem = app_start_task(telemetry_task, S1_PRI_TELEM, 0,
                                    "telemetry_task");
    if (g_s1_tid_telem < E_OK)
    {
        er = g_s1_tid_telem;
    }

#if S1_IF05_COMPAT
    if (er == E_OK)
    {
        tid_led = app_start_task(if05_stub_task, S1_PRI_LED, 0,
                                 "if05_stub");
        if (tid_led < E_OK)
        {
            er = tid_led;
        }
    }
#endif

    return er;
}