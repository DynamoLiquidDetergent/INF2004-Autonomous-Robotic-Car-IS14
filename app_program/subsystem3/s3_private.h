/*
 *	s3_private.h
 *	Functions shared ONLY between subsystem 3's own files.
 *	Other subsystems include subsystem3.h, never this file.
 */

#ifndef S3_PRIVATE_H
#define S3_PRIVATE_H

#include "subsystem3.h"

/* ---------------------------------------------------------------------
 *  DEBUG SWITCH for the Commit 4 Part B video.
 *  1 = in a NORMAL build (make -j8, not TEST=1) the robot drives forward
 *      over the barcode by itself, stops after a decode, then drives back
 *      over it in reverse.  Also turns on the A,B,C,D test mapping and the
 *      "BARCODE: ch=.. cmd=.." prints in barcode_decode.c.
 *  MUST be 0 when committing.
 * --------------------------------------------------------------------- */
#define S3_BC_DRIVE_DEMO    0 /* S5: the Vehicle Controller drives (S5_PHASE 6+); two senders to Motion Control would fight */

/* ---------------------------------------------------------------------
 *  DEBUG SWITCH: line following (needs S3_BC_DRIVE_DEMO 1).
 *  1 = the drive demo FOLLOWS THE BLACK LINE: it reads q_line_data
 *      (IF-05) and q_barcode_cmd (IF-07) and steers through q_motion_cmd
 *      (IF-10), standing in for the Vehicle Controller (S5) until S5 has
 *      its own.  Also prints the raw sensor values and line state every
 *      200 ms, and every junction (S3_LINE_DEBUG).
 *  0 = the drive demo drives straight (barcode tests S3-M02 / S3-M03).
 *  MUST be 0 once S5's Vehicle Controller reads q_line_data or
 *  q_barcode_cmd: two readers of one message buffer would each get only
 *  some of the messages.
 * --------------------------------------------------------------------- */
#define S3_LINE_FOLLOW_DEMO 1

/* ---------------------------------------------------------------------
 *  Line position thresholds, normalised 0..1000 (1000 = fully over the
 *  line).  Shared with the tests (S3-U09).
 * --------------------------------------------------------------------- */
#define S3_NORM_MAX             1000u    /* normalised scale 0..1000       */
#define S3_ON_LINE_THRESHOLD    500u     /* normalised value = "on line"   */
#define S3_EDGE_FULL            800u     /* centre squarely on the line    */
#define S3_EDGE_HINT            250u     /* sensor starting to see the line */

/* ---------------------------------------------------------------------
 *  Junction detection (s3_junction_step), one call per 20 ms line cycle.
 *
 *  Sensor layout: GP26 and the centre side by side (no gap), GP28 ~4.5 cm
 *  out on the other side, line 3-4 cm wide.  Following the line's edge,
 *  GP26 is normally WHITE, the centre BLACK, GP28 WHITE.
 *
 *  A + crossbar makes GP26 and the centre BOTH fully black (>=
 *  S3_EDGE_FULL) for about its width, and also reaches GP28.
 *  Serial log 2026-11-04: GP26 and centre both above ~2400 raw only at
 *  the two junction crossings (in curves the two together peaked at
 *  ~63-69 %); at the first crossing GP28 read 2564 at the same time.
 *
 *  Junction =
 *    1. a run of cycles with GP26 AND centre both fully black, between
 *       S3_JN_MIN_MM and S3_JN_MAX_MM long (odometry, IF-17):
 *         shorter = the line swept across both by a correction
 *         longer  = the main line itself under both (a curve)
 *    2. AND GP28 dark, with the centre still on the line, at some point
 *       from S3_JN_R_BEFORE_MM before the run to S3_JN_R_AFTER_MM after it
 *       (a crooked approach meets the bar with GP28 a little earlier or
 *       later).  When the line drifts under GP26, GP28 is certainly on
 *       white: the line cannot cover both; and when the line itself
 *       swings under GP28 it has left the centre first.
 *  Flagged when both are true -- just after the sensors leave the bar.
 *  Tune from the "S3: GP26+C full ..." lines S3_LINE_DEBUG prints.
 * --------------------------------------------------------------------- */
#define S3_JN_MIN_MM            8u   /* both full at least this long        */
#define S3_JN_MAX_MM            60u  /* ... and at most this (40 mm bar +
                                        6 mm sensor spot + crooked approach) */
#define S3_JN_R_BEFORE_MM       15   /* GP28 dark this far before the run
                                        (45 mm x tan 20 deg of crooked approach) */
#define S3_JN_R_AFTER_MM        15   /* ... or up to this far after it      */
#define S3_JN_BLIND_MM          10u  /* the zone is held over a both-full run
                                        this far on its own; further only once
                                        GP28 has gone dark too (a drift of the
                                        line under GP26 must not blind the
                                        steering for long)                   */
#define S3_JN_HOLD_CYCLES       5u   /* flag stays up 100 ms                 */
#define S3_JN_REFRACT_CYCLES    25u  /* then 500 ms with no new junction     */

/* s3_junction_step() result bits */
#define S3_JN_RISE              0x1u /* the flag rose this cycle             */
#define S3_JN_FALL              0x2u /* the flag fell this cycle             */
#define S3_JN_HOLDZONE          0x4u /* possible crossbar under the sensors:
                                        do not steer on this cycle's zone    */
#define S3_JN_RUN_END           0x8u /* a both-full run just ended (length
                                        in j->last_mm, for the debug print)  */
#define S3_JN_NO_GP28           0x10u/* a run of bar length ended, but GP28
                                        stayed white: not a junction         */

/* Junction detector state.  Call s3_junction_init() before the first step. */
typedef struct {
    bool     full;          /* a both-full run is in progress                */
    uint16_t start_mm;      /* odometry when it started (wraps: differences) */
    uint16_t len_mm;        /* its length so far                             */
    uint16_t last_mm;       /* length of the last finished run               */
    bool     pend;          /* a bar-length run ended, waiting for GP28      */
    uint16_t pend_lo;       /* GP28 dark at or after this counts ...         */
    uint16_t pend_hi;       /* ... until the odometry passes this            */
    bool     r_seen;        /* GP28 has been dark at least once              */
    uint16_t r_mm;          /* odometry when GP28 was last dark              */
    uint8_t  hold;          /* cycles the flag stays up                      */
    uint8_t  refract;       /* cycles before another junction may be flagged */
    bool     active;        /* the junction flag (IF-05 bit 4)               */
} S3_JN;

/* ---- line_sensor.c ---- */
IMPORT ER   line_sensor_init(void);     /* phase 1: create objects      */
IMPORT ER   line_sensor_start(void);    /* phase 2: start the task      */

/* Line position (zone 1..7, 4 = on target) from the LEFT and CENTRE
   sensors only; GP28 is not used.  Normalised readings 0..1000.  Pure
   function, see line_sensor.c. */
IMPORT uint8_t s3_edge_zone(UW n_left, UW n_centre, uint8_t prev_zone,
                            bool was_on, bool *on_line);

/* Junction detector.  One call per 20 ms line cycle.
     both_full : GP26 and the centre are BOTH >= S3_EDGE_FULL (normalised)
     r_dark    : GP28 dark for most of this cycle's 2 ms samples, AND the
                 centre on the line (else it is the main line under GP28)
     odo_mm    : distance driven (s2_odo_dist_mm(s2_get_odometry()), wraps)
   Returns S3_JN_* bits; the flag itself is j->active.  Pure function. */
IMPORT void s3_junction_init(S3_JN *j);
IMPORT UINT s3_junction_step(S3_JN *j, bool both_full, bool r_dark, uint16_t odo_mm);

/* Classify one GP28 reading as dark (bar / line) or light, with
   hysteresis, using the calibrated GP28 range and sensor polarity. */
IMPORT bool line_sensor_gp28_dark(uint16_t adc, bool was_dark);

/* Stop / restart sending GP28 samples to q_gp28_samples.
   Used by the tests to inject a hand-made barcode. */
IMPORT void line_sensor_pause_gp28(bool pause);

/* Count of GP28 samples successfully put into q_gp28_samples (INT-02). */
IMPORT UW   line_sensor_gp28_sent(void);

/* An ADC value that is certainly dark / certainly light (for tests). */
IMPORT uint16_t line_sensor_gp28_test_level(bool dark);

/* ---- barcode_decode.c ---- */
IMPORT ER   barcode_decode_start(void); /* phase 2: start the task      */

/* true if the mapping is four distinct letters A-Z (IF char map) */
IMPORT bool s3_char_map_valid(const CHAR_MAP_MSG *m);

/* Number of valid letters decoded since boot */
IMPORT UW   barcode_decode_count(void);

/* true while GP28 is inside a possible barcode (from its first dark bar
   until the read ends or is abandoned).  The line follower keeps the
   steering small meanwhile, so GP28's speed over the bars stays steady. */
IMPORT bool barcode_decode_busy(void);

/* ---- s3_drive_demo.c (debug only, see S3_BC_DRIVE_DEMO) ---- */
IMPORT ER   s3_drive_demo_start(void);  /* phase 2; does nothing when off */

#endif /* S3_PRIVATE_H */
