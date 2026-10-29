/*
 *	s3_drive_demo.c
 *	Subsystem 3 -- DEBUG ONLY: drive the robot over barcodes so the
 *	decoder can be tested on a moving robot instead of pushing it by hand.
 *
 *	Uses ONLY subsystem 2's public interface (TEAM.md section 5):
 *	  IF-10  q_motion_cmd  + s2_cmd_pack()   -- DRIVE / STOP commands
 *	  IF-16  s2_get_status()                 -- current epoch and state
 *	exactly as the Vehicle Controller (S5) will.  No file of subsystem 2
 *	is changed.
 *
 *	Two modes, chosen by DEMO_CONTINUOUS:
 *
 *	  DEMO_CONTINUOUS 1  (barcode decode test)
 *	    1. wait DEMO_START_DELAY_MS so the robot can be put down
 *	    2. drive FORWARD in a straight line at DEMO_SPEED_MMPS and never
 *	       stop or reverse.  Every decoded barcode is printed by
 *	       barcode_decode.c ("BARCODE: ch=..") and counted here
 *	       ("DEMO: barcode #n read"); the robot keeps going.
 *	    Stop it by picking it up / pressing RESET.  If a wheel is blocked
 *	    (e.g. it hits a wall) subsystem 2 latches STALLED and the motors
 *	    stay off until RESET.
 *
 *	  DEMO_CONTINUOUS 0  (Commit 4 Part B video)
 *	    drive FORWARD until one barcode is decoded, run on DEMO_RUN_ON_MS,
 *	    stop; then (DEMO_REVERSE) drive BACKWARD over it the same way.
 *
 *	Enabled by S3_BC_DRIVE_DEMO in s3_private.h.  When it is 0 (as it
 *	MUST be when committing) this file compiles to a function that does
 *	nothing.
 */

#include "s3_private.h"
#include "../subsystem2/subsystem2.h"      /* IF-10 / IF-16 */

#define DEMO_CONTINUOUS       1      /* 1 = forward forever, report each decode */

#define DEMO_PRI              20     /* below every S3 task and Motion Control */
#define DEMO_START_DELAY_MS   5000   /* time to put the robot down            */
#define DEMO_SPEED_MMPS       100    /* 1-2 mm narrow bars: 100 mm/s is ideal  */
#define DEMO_SEND_MS          50     /* < S2_CMD_TIMEOUT_MS (100 ms) watchdog */
#define DEMO_HEARTBEAT_MS     1000   /* continuous mode: status line period   */

/* Used only when DEMO_CONTINUOUS is 0 */
#define DEMO_MAX_RUN_MS       5000   /* safety limit per pass (~50 cm)        */
#define DEMO_RUN_ON_MS        400    /* keep going after a decode (~4 cm)     */
#define DEMO_PAUSE_MS         2000   /* stop between forward and reverse      */
#define DEMO_REVERSE          1      /* 1 = also drive back over the barcode  */

/* Diagnostics (robot not driving straight):
   DEMO_TRACE 1     print both wheel speeds (IF-16) every 100 ms, and run the
                    spin-by-hand encoder check during the start delay.
                    Off here so the BARCODE lines are easy to see.
   DEMO_STEER_MMPS  steering sent with DRIVE (+ = turn right).  0 = subsystem 2
                    holds a straight line on its encoders.  Any non-zero value
                    switches that hold OFF, so both wheels just run at the
                    commanded speed (+/- this value). */
#define DEMO_TRACE            0
#define DEMO_STEER_MMPS       0

#if S3_BC_DRIVE_DEMO && !APP_TEST

LOCAL UW demo_now_ms(void)
{
    SYSTIM t;
    tk_get_tim(&t);
    return t.lo;
}

LOCAL void demo_send(UW word)
{
    (void)tk_snd_mbf(q_motion_cmd, &word, (INT)sizeof(word), TMO_POL);
}

#if DEMO_CONTINUOUS

/* Drive forward forever.  The DRIVE command is re-sent every DEMO_SEND_MS
   to keep subsystem 2's command watchdog fed; decoding is not touched. */
LOCAL void demo_run_forever(UINT epoch)
{
    UW   start     = demo_now_ms();
    UW   last_beat = start;
    UW   seen      = barcode_decode_count();
    UW   count, now, st;
    UINT state, last_state = 0xFFFFu;

    tm_printf((UB *)"DEMO: driving FORWARD at %d mm/s, steering %d -- "
                    "continuous, will not stop on a barcode\n",
              DEMO_SPEED_MMPS, DEMO_STEER_MMPS);

    while (1) {
        /* steering 0 = Motion Control holds a straight line on the encoders */
        demo_send(s2_cmd_pack(S2_MODE_DRIVE, DEMO_SPEED_MMPS, DEMO_STEER_MMPS, epoch));

        now   = demo_now_ms();
        count = barcode_decode_count();
        if (count != seen) {
            /* the letter itself is printed by barcode_decode.c just before */
            tm_printf((UB *)"DEMO: barcode #%u read at t=%u ms -- still driving\n",
                      count, now - start);
            seen = count;
        }

        st    = s2_get_status();
        state = s2_status_state(st);
        if (state != last_state) {
            /* e.g. STALLED: motors latched off until RESET */
            tm_printf((UB *)"DEMO: motion state %u\n", state);
            last_state = state;
        }

        if ((now - last_beat) >= DEMO_HEARTBEAT_MS) {
            tm_printf((UB *)"DEMO: t=%5u ms  L=%4d  R=%4d mm/s  barcodes=%u\n",
                      now - start, s2_status_speed_l(st), s2_status_speed_r(st),
                      count);
            last_beat = now;
        }

        tk_dly_tsk(DEMO_SEND_MS);
    }
}

#else  /* !DEMO_CONTINUOUS */

/* Drive at 'speed' (signed mm/s) until one more letter is decoded
   (plus the run-on distance) or the time limit, then stop. */
LOCAL void demo_pass(INT speed, UINT epoch)
{
    UW   start   = demo_now_ms();
    UW   count0  = barcode_decode_count();
    UW   seen_at = 0, now;
    bool decoded = false;
#if DEMO_TRACE
    UW   n = 0, st;
#endif

    tm_printf((UB *)"DEMO: driving %s at %d mm/s, steering %d\n",
              (speed > 0) ? "FORWARD" : "BACKWARD", (speed > 0) ? speed : -speed,
              DEMO_STEER_MMPS);

    while (1) {
        demo_send(s2_cmd_pack(S2_MODE_DRIVE, speed, DEMO_STEER_MMPS, epoch));

        now = demo_now_ms();
#if DEMO_TRACE
        if ((++n % 2U) == 0U) {                 /* every 100 ms */
            st = s2_get_status();
            tm_printf((UB *)"DEMO: t=%4u ms  L=%4d  R=%4d mm/s  state=%u\n",
                      now - start, s2_status_speed_l(st), s2_status_speed_r(st),
                      s2_status_state(st));
        }
#endif
        if (!decoded && barcode_decode_count() != count0) {
            decoded = true;
            seen_at = now;
        }
        if (decoded && (now - seen_at) >= DEMO_RUN_ON_MS) break;
        if ((now - start) >= DEMO_MAX_RUN_MS)             break;

        tk_dly_tsk(DEMO_SEND_MS);
    }

    demo_send(s2_cmd_pack(S2_MODE_STOP, 0, 0, epoch));
    tm_printf((UB *)"DEMO: stopped (%s)\n",
              decoded ? "barcode decoded" : "time limit, no barcode decoded");
}

#endif /* DEMO_CONTINUOUS */

LOCAL void s3_drive_demo_task(INT stacd, void *exinf)
{
    UINT epoch;

    (void)stacd; (void)exinf;

    tm_printf((UB *)"DEMO: robot drives in %d s -- right sensor (GP28) in line "
                    "with the bars\n",
              DEMO_START_DELAY_MS / 1000);
#if DEMO_TRACE
    /* Encoder check: motors are off during the countdown, but Motion
       Control still measures wheel speed.  Spin each wheel BY HAND now:
       its L or R value must rise above 0.  If it stays 0, that wheel's
       encoder is not being read (wiring / connector). */
    tm_printf((UB *)"DEMO: encoder check -- spin each wheel by hand now\n");
    {
        UW t0 = demo_now_ms(), st;
        while ((demo_now_ms() - t0) < DEMO_START_DELAY_MS) {
            st = s2_get_status();
            tm_printf((UB *)"DEMO: by hand  L=%4d  R=%4d mm/s\n",
                      s2_status_speed_l(st), s2_status_speed_r(st));
            tk_dly_tsk(250);
        }
    }
#else
    tk_dly_tsk(DEMO_START_DELAY_MS);
#endif

    /* A newer epoch than anything Motion Control has seen (IF-10) */
    epoch = (s2_status_epoch(s2_get_status()) + 1U) & S2_MASK_4;

#if DEMO_CONTINUOUS
    demo_run_forever(epoch);                   /* never returns */
#else
    demo_pass(DEMO_SPEED_MMPS, epoch);
#if DEMO_REVERSE
    tk_dly_tsk(DEMO_PAUSE_MS);
    demo_pass(-DEMO_SPEED_MMPS, epoch);
#endif
    tm_printf((UB *)"DEMO: done (motion state %u) -- press RESET to run again\n",
              s2_status_state(s2_get_status()));
#endif

    while (1) {
        tk_slp_tsk(TMO_FEVR);
    }
}

EXPORT ER s3_drive_demo_start(void)
{
    ID tid = app_start_task(s3_drive_demo_task, DEMO_PRI, 0, "s3_drive_demo");
    return (tid < E_OK) ? tid : E_OK;
}

#else  /* demo off (or a TEST=1 build): nothing to start */

EXPORT ER s3_drive_demo_start(void)
{
    return E_OK;
}

#endif
