/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "../test_report.h"
#define S1_TEST_START_DELAY_MS  (5000)  /* time to open the serial port */

#if APP_TEST

EXPORT void subsystem1_test(void)
{
    T_RTSK rtsk;
    INT    i;
    INT    waiting = 0;

    /* Give the tester time to open the USB serial port */
    (void)tk_dly_tsk(S1_TEST_START_DELAY_MS);

    TEST_INIT();

    /* S1-U01: telemetry task was created */
    TEST_CHECK("S1-U01", g_s1_tid_telem > 0, "telemetry task created");

    /* S1-U02: task sleeps between ticks (not busy-looping) */
    for (i = 0; i < 10; i++)
    {
        (void)tk_dly_tsk(97);   /* not a multiple of the tick */
        if ((tk_ref_tsk(g_s1_tid_telem, &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0))
        {
            waiting++;
        }
    }
    TEST_CHECK("S1-U02", waiting >= 9, "telemetry task sleeps between ticks");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */