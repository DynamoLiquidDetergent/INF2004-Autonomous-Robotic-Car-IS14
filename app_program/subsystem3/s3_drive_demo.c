/*
 *	s3_drive_demo.c
 *	Subsystem 3 -- DEBUG ONLY: drive the robot by itself so subsystem 3
 *	can be tested on a moving robot instead of pushing it by hand.
 *
 *	Uses ONLY public interfaces (TEAM.md section 5):
 *	  IF-05  q_line_data   (S3)              -- line position (line follow)
 *	  IF-10  q_motion_cmd  + s2_cmd_pack()   -- DRIVE / STOP commands
 *	  IF-16  s2_get_status()                 -- current epoch and state
 *	  IF-17  s2_get_odometry()               -- distance driven
 *	exactly as the Vehicle Controller (S5) will.  No file of subsystem 2
 *	is changed.
 *
 *	Three modes:
 *
 *	  S3_LINE_FOLLOW_DEMO 1  (s3_private.h -- line following)
 *	    1. calibration window (DEMO_CALIB_MS): sweep the robot slowly from
 *	       side to side across the line so the left and centre sensors
 *	       see white and black.  Skip it and the default range is kept.
 *	    2. DEMO_PLACE_MS to put the robot down, centre sensor on the line
 *	    3. follow the line: every new LINE_DATA_MSG (20 ms) gives a zone
 *	       1..7 from the LEFT and CENTRE sensors only (4 = centre on black,
 *	       left on white; GP28 only reads barcodes); the steering is
 *	       proportional to how far off that it is (plus a little of how
 *	       fast that changes), and the
 *	       robot slows down on sharp corrections.  While the line is lost
 *	       it keeps turning towards the side it was last seen on, slowly;
 *	       lost for DEMO_LOST_STOP_MS (end of the track) -> stop.
 *	    Barcodes are still decoded and counted on the way.
 *
 *	  DEMO_CONTINUOUS 1  (barcode decode test, S3-M02 / S3-M03)
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
#include "../subsystem2/subsystem2.h"      /* IF-10 / IF-16 / IF-17 */

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

/* ---------------------------------------------------------------------
 *  Line following (S3_LINE_FOLLOW_DEMO).  Tune on the track.
 *
 *  err = zone - 4: -3..+3, + = the line is towards the LEFT sensor (GP26).
 *  steering (mm/s, + = turn right, IF-10) = -(KP*err + KD*(err - err_prev))
 *  speed    = DEMO_LF_SPEED - DEMO_LF_SLOW_PER_ZONE*|err|, >= DEMO_LF_MIN_SPEED
 *
 *  Robot weaves from side to side   -> lower KP, or raise KD a little
 *  Robot runs off on the outside of a bend -> raise KP, or lower DEMO_LF_SPEED
 *  Robot turns AWAY from the line   -> DEMO_LF_STEER_SIGN -1 (see below)
 * --------------------------------------------------------------------- */

/* +1 = the LEFT sensor (GP26) is on the robot's left as it drives forward.
   -1 = it is on the robot's right.  Bench check: hold the robot up, wheels
   free, centre sensor over white; put black tape under GP26 only.  The
   RIGHT wheel must now turn faster than the left (robot turns towards
   GP26).  If the LEFT wheel is faster, set -1. */
#define DEMO_LF_STEER_SIGN    1

#define DEMO_LF_CALIB_MS      5000   /* calibration window (S3_CALIB_TIMEOUT_MS) */
#define DEMO_LF_PLACE_MS      3000   /* then time to put the robot on the line */
#define DEMO_LF_SPEED         100    /* mm/s when centred (barcodes: 50..150)  */
#define DEMO_LF_MIN_SPEED     40     /* mm/s at the sharpest correction        */
#define DEMO_LF_SLOW_PER_ZONE 20     /* mm/s slower per zone off centre        */
#define DEMO_LF_KP            30     /* mm/s steering per zone off centre      */
#define DEMO_LF_KD            20     /* mm/s steering per zone/cycle of change */
#define DEMO_LF_STEER_MAX     120    /* mm/s steering limit                    */
#define DEMO_LF_SEARCH_SPEED  40     /* mm/s while the line is lost            */
#define DEMO_LF_SEARCH_STEER  100    /* mm/s turn towards where the line was   */
#define DEMO_LF_LOST_STOP_MS  1500   /* lost this long (after S3's 200 ms) -> stop */
#define DEMO_LF_NODATA_MS     200    /* no LINE_DATA_MSG this long -> stop     */
#define DEMO_LF_PRINT_MS      500    /* status line period                     */

/* ---------------------------------------------------------------------
 *  Line-follow controller: pure logic, no kernel calls (host-testable).
 * --------------------------------------------------------------------- */
#if S3_BC_DRIVE_DEMO && !APP_TEST && S3_LINE_FOLLOW_DEMO

typedef struct {
    INT  err_prev;       /* err of the previous cycle                       */
    bool have_prev;      /* err_prev is valid                               */
    UW   lost_ms;        /* how long LINE_LOST has been set                 */
    bool stopped;        /* gave up: line lost for DEMO_LF_LOST_STOP_MS     */
} DEMO_LF;

typedef struct {
    INT  speed;          /* mm/s                                            */
    INT  steer;          /* mm/s, + = turn right (IF-10)                    */
    bool stop;           /* send STOP instead of DRIVE                      */
} DEMO_LF_OUT;

LOCAL INT lf_clamp(INT v, INT lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

LOCAL INT lf_abs(INT v)
{
    return (v < 0) ? -v : v;
}

/* One line cycle.  zone/lost from LINE_DATA_MSG, dt_ms since the last call. */
LOCAL DEMO_LF_OUT lf_step(DEMO_LF *c, uint8_t zone, bool lost, UW dt_ms)
{
    DEMO_LF_OUT o = { 0, 0, false };
    INT         err = (INT)zone - (INT)LINE_ZONE_CENTRE;   /* + = line to the LEFT sensor */
    INT         turn;                                     /* + = turn towards GP26 */

    if (zone == 0u) err = 0;                              /* not a valid zone */

    if (c->stopped) {
        o.stop = true;
        return o;
    }

    if (lost) {
        c->lost_ms += dt_ms;
        if (c->lost_ms >= DEMO_LF_LOST_STOP_MS) {
            c->stopped = true;
            o.stop     = true;
            return o;
        }
        /* S3 repeats the last zone while lost: turn towards that side */
        turn       = (err > 0) ? DEMO_LF_SEARCH_STEER : ((err < 0) ? -DEMO_LF_SEARCH_STEER : 0);
        o.speed    = DEMO_LF_SEARCH_SPEED;
        o.steer    = -DEMO_LF_STEER_SIGN * turn;
        c->have_prev = false;                             /* no D kick on re-acquire */
        return o;
    }

    c->lost_ms = 0u;

    turn = DEMO_LF_KP * err;
    if (c->have_prev) {
        turn += DEMO_LF_KD * (err - c->err_prev);
    }
    c->err_prev  = err;
    c->have_prev = true;

    /* turning towards GP26 (the LEFT sensor) is a left turn when
       DEMO_LF_STEER_SIGN is +1, i.e. negative steering */
    o.steer = lf_clamp(-DEMO_LF_STEER_SIGN * turn, DEMO_LF_STEER_MAX);
    o.speed = DEMO_LF_SPEED - DEMO_LF_SLOW_PER_ZONE * lf_abs(err);
    if (o.speed < DEMO_LF_MIN_SPEED) o.speed = DEMO_LF_MIN_SPEED;
    return o;
}

#endif /* line-follow controller */

#if S3_BC_DRIVE_DEMO && !APP_TEST

/* barcode_decode.c: raw GP28 capture of the first barcode pass */
IMPORT UW barcode_capture(const uint16_t **buf);
#define DEMO_CAP_PER_LINE     25     /* values per "CAP" line             */
#define DEMO_CAP_LINES        4      /* "CAP" lines printed per 50 ms loop */

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

#if S3_LINE_FOLLOW_DEMO

/* Follow the line until it is lost for good.  Never returns. */
LOCAL void demo_follow_line(UINT epoch)
{
    DEMO_LF       c = { 0, false, 0u, false };
    DEMO_LF_OUT   o = { 0, 0, true };
    LINE_DATA_MSG m, newer;
    uint8_t       zone = LINE_ZONE_CENTRE;
    bool          lost = false, said_stop = false;
    UW            start = demo_now_ms();
    UW            last_data = start, last_step = start, last_print = start;
    UW            seen = barcode_decode_count();
    UW            now, count, st;
    UINT          state, last_state = 0xFFFFu;
    INT           rc;

    /* old messages from the countdown are stale: start from fresh ones */
    while (tk_rcv_mbf(q_line_data, &newer, TMO_POL) == (INT)sizeof(newer)) { }

    tm_printf((UB *)"DEMO: following the line at up to %d mm/s (KP %d, KD %d, sign %d)\n",
              DEMO_LF_SPEED, DEMO_LF_KP, DEMO_LF_KD, DEMO_LF_STEER_SIGN);

    while (1) {
        /* wait for the next line state (every 20 ms), but never longer than
           DEMO_SEND_MS so the motion command keeps being sent */
        rc  = tk_rcv_mbf(q_line_data, &m, DEMO_SEND_MS);
        now = demo_now_ms();

        if (rc == (INT)sizeof(m)) {
            /* fell behind: use the newest one */
            while (tk_rcv_mbf(q_line_data, &newer, TMO_POL) == (INT)sizeof(newer)) {
                m = newer;
            }
            zone      = line_data_zone(m);
            lost      = line_data_lost(m);
            o         = lf_step(&c, zone, lost, now - last_step);
            last_step = now;
            last_data = now;
        } else if ((now - last_data) >= DEMO_LF_NODATA_MS) {
            o.stop = true;                     /* line sensor silent: do not drive blind */
        }

        if (o.stop) {
            demo_send(s2_cmd_pack(S2_MODE_STOP, 0, 0, epoch));
            if (!said_stop) {
                tm_printf((UB *)"DEMO: stopped (%s) -- press RESET to run again\n",
                          c.stopped ? "line lost" : "no line data");
                said_stop = true;
            }
        } else {
            demo_send(s2_cmd_pack(S2_MODE_DRIVE, o.speed, o.steer, epoch));
        }

        count = barcode_decode_count();
        if (count != seen) {
            tm_printf((UB *)"DEMO: barcode #%u read at t=%u ms\n", count, now - start);
            seen = count;
        }

        st    = s2_get_status();
        state = s2_status_state(st);
        if (state != last_state) {
            tm_printf((UB *)"DEMO: motion state %u\n", state);   /* 6 = STALLED */
            last_state = state;
        }

        if ((now - last_print) >= DEMO_LF_PRINT_MS) {
            tm_printf((UB *)"DEMO: t=%5u  zone=%u lost=%u  cmd %4d mm/s steer %4d  wheels L=%4d R=%4d  dist=%5u\n",
                      now - start, (unsigned)zone, (unsigned)lost,
                      o.stop ? 0 : o.speed, o.stop ? 0 : o.steer,
                      s2_status_speed_l(st), s2_status_speed_r(st),
                      s2_odo_dist_mm(s2_get_odometry()));
            last_print = now;
        }
    }
}

#elif DEMO_CONTINUOUS

/* Drive forward forever.  The DRIVE command is re-sent every DEMO_SEND_MS
   to keep subsystem 2's command watchdog fed; decoding is not touched. */
LOCAL void demo_run_forever(UINT epoch)
{
    UW   start     = demo_now_ms();
    UW   last_beat = start;
    UW   seen      = barcode_decode_count();
    UW   count, now, st;
    UINT state, last_state = 0xFFFFu;
    const uint16_t *cap = 0;
    UW   cap_n = 0, cap_i = 0, k;
    bool cap_printed = false;

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
            /* dist = average of both wheels (IF-17).  It must grow by about
               DEMO_SPEED_MMPS per second; if it stays near 0 one encoder
               counts backwards (S2_ENC_x_INVERT in subsystem2.h). */
            tm_printf((UB *)"DEMO: t=%5u ms  L=%4d  R=%4d mm/s  dist=%5u mm  barcodes=%u\n",
                      now - start, s2_status_speed_l(st), s2_status_speed_r(st),
                      s2_odo_dist_mm(s2_get_odometry()), count);
            last_beat = now;
        }

        /* Raw GP28 capture: one line per loop, so DRIVE keeps being sent */
        if (!cap_printed) {
            if (cap_n == 0U) {
                cap_n = barcode_capture(&cap);
                if (cap_n != 0U) {
                    tm_printf((UB *)"CAP: start, %u GP28 samples, 2 ms apart\n", cap_n);
                }
            } else {
                UW line;

                for (line = 0U; line < (UW)DEMO_CAP_LINES && !cap_printed; line++) {
                    tm_printf((UB *)"CAP,%u:", cap_i);
                    for (k = 0U; k < (UW)DEMO_CAP_PER_LINE && cap_i < cap_n; k++, cap_i++) {
                        tm_printf((UB *)" %u", (unsigned)cap[cap_i]);
                    }
                    tm_printf((UB *)"\n");
                    if (cap_i >= cap_n) {
                        tm_printf((UB *)"CAP: end\n");
                        cap_printed = true;
                    }
                }
            }
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

#endif /* S3_LINE_FOLLOW_DEMO / DEMO_CONTINUOUS */

LOCAL void s3_drive_demo_task(INT stacd, void *exinf)
{
    UINT epoch;

    (void)stacd; (void)exinf;

#if S3_LINE_FOLLOW_DEMO
    /* 1. calibration: the line sensor task records the range of the left
          and centre sensors while the robot is swept across the line */
    tm_printf((UB *)"DEMO: LINE FOLLOW.  Calibrating for %d s -- sweep the robot slowly "
                    "side to side across the black line now (or leave it: defaults kept)\n",
              DEMO_LF_CALIB_MS / 1000);
    if (subsystem3_run_calibration()) {
        tm_printf((UB *)"DEMO: calibration done\n");
    } else {
        tm_printf((UB *)"DEMO: calibration not done, default range kept\n");
    }

    /* 2. time to put it down */
    tm_printf((UB *)"DEMO: put the robot on the line, centre sensor over it -- "
                    "starting in %d s\n", DEMO_LF_PLACE_MS / 1000);
    tk_dly_tsk(DEMO_LF_PLACE_MS);
#else
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
#endif /* S3_LINE_FOLLOW_DEMO */

    /* A newer epoch than anything Motion Control has seen (IF-10) */
    epoch = (s2_status_epoch(s2_get_status()) + 1U) & S2_MASK_4;

#if S3_LINE_FOLLOW_DEMO
    demo_follow_line(epoch);                   /* never returns */
#elif DEMO_CONTINUOUS
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
