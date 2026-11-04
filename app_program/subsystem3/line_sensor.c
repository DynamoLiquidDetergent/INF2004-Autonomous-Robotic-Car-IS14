/*
 *	line_sensor.c
 *	Subsystem 3 -- Line Sensor Task (owning task of the ADC, TEAM.md section 3).
 *
 *	Every 2 ms  (sub-cycle):  read GP28 (ADC2), send GP28_SAMPLE_MSG to
 *	                          q_gp28_samples (IF-06), drop-newest when full.
 *	                          Also counts how many were dark (junction).
 *	Every 20 ms (line cycle): read GP26 (ADC0) and GP27 (ADC1), compute the
 *	                          line state, send LINE_DATA_MSG to q_line_data
 *	                          (IF-05), drop-oldest when full so the reader
 *	                          always gets fresh data.
 *
 *	Wiring (Maker Pi Pico Grove ports):
 *	  left   GP26 (ADC0): Grove 5, the GP26 wire.  NOT the GP6 wire of
 *	                      Grove 5: GP6 has no ADC.  "Left" is the S3 code
 *	                      name: physically it is on the CAR'S RIGHT.
 *	  centre GP27 (ADC1): Grove 6, the GP27 wire.  The GP26 wire of
 *	                      Grove 6 must stay unconnected (it is the same pin
 *	                      as the left sensor).
 *	  right  GP28 (ADC2): barcode sensor, on the CAR'S LEFT, ~4.5 cm out.
 *	                      Not used for the line position or line-lost;
 *	                      confirms a junction (see below).
 *	  GP26 and the centre sensor sit side by side, no gap; the line is
 *	  3-4 cm wide.
 *
 *	Line tracking uses LEFT + CENTRE and follows the line's left-sensor
 *	edge (centre on black, left on white): see s3_edge_zone().
 *
 *	Junction (2026-11-04): GP26 and the centre side by side are normally
 *	white / black (edge following).  A + crossbar makes BOTH fully black for
 *	about its width; that run's length (odometry, IF-17) tells a crossbar
 *	from a curve.  See s3_junction_step().  During such a run the zone is
 *	reported as 4 (on target), so the crossbar never looks like "the line
 *	moved towards GP26" (which used to steer the car into a turn).  GP28
 *	must also go dark near that run: the crossbar reaches it, but the main
 *	line drifting under GP26 never can (it cannot cover both).
 *
 *	Calibration: subsystem3_run_calibration() (called from ANOTHER task)
 *	asks this task to record the min/max of the LEFT and CENTRE sensors
 *	while the robot is swept across the line.  GP28 keeps its hand-measured
 *	range (S3_GP28_DEFAULT_*) unless S3_CALIB_GP28 is 1, because the
 *	barcode thresholds depend on it.  Only this task ever touches the ADC.
 *
 *	Bypass (IF-09): subsystem3_set_bypass(true) suppresses LINE_LOST.
 *	The Safety Monitor fault bit (IF-08) is added in Commit 5.
 *
 *	APIs used: tk_opn_dev, tk_srea_dev, tk_snd_mbf, tk_rcv_mbf, tk_dly_tsk,
 *	           tk_get_tim, tk_cre_flg, tk_set_flg, tk_clr_flg, tk_wai_flg
 */

#include "s3_private.h"
#include "../subsystem2/subsystem2.h"     /* IF-17 odometry: crossbar length */

/* ---------------------------------------------------------------------
 *  Configuration
 * --------------------------------------------------------------------- */
#define S3_ADC_DEVNAME          "adca"   /* kernel ADC driver, unit a      */

#define S3_CH_LEFT              0        /* GP26, ADC0                     */
#define S3_CH_CENTRE            1        /* GP27, ADC1                     */
#define S3_CH_RIGHT             2        /* GP28, ADC2 (also barcode)      */
#define S3_NUM_SENSORS          3

#define S3_SUBCYCLE_MS          2        /* GP28 sample period             */
#define S3_LINE_CYCLE_MS        20       /* line state period              */

#define S3_LOST_DEBOUNCE_CYCLES 10       /* 10 x 20 ms = 200 ms            */


/* 1 = the ADC reads HIGHER over the dark line than over the white floor.
   Check this on the bench with S3_LINE_DEBUG (raw values are printed);
   set to 0 if your sensors read lower over the line. */
#define S3_LINE_IS_HIGH         1

#define S3_CALIB_MIN_SPREAD     300u     /* min (max-min) ADC counts needed */

/* S3_NORM_MAX, S3_ON_LINE_THRESHOLD, S3_EDGE_FULL, S3_EDGE_HINT and the
   S3_JN_* junction settings are in s3_private.h (the tests use them). */

/* 1 = calibration also replaces the GP28 range.  0 (default) = GP28 keeps
   S3_GP28_DEFAULT_LO/HI: a sweep over the solid line reads higher than the
   barcode bars do, and would raise the bar threshold above the bars. */
#define S3_CALIB_GP28           0

/* 1 = print raw values and the line state every 200 ms (Commit 3 Part B),
   and one "S3: JUNCTION" line every time the junction flag rises.
   On with the line-follow demo (s3_private.h); MUST be 0 when committing. */
#define S3_LINE_DEBUG           (S3_BC_DRIVE_DEMO && S3_LINE_FOLLOW_DEMO)

/* 1 = every second, print the lowest and highest GP28 reading of that
   second, and the raw levels a reading must cross to count as dark / light.
   Catches bars that S3_LINE_DEBUG (every 200 ms) would miss.
   On with the drive demo; MUST be 0 when committing. */
#define S3_GP28_DEBUG           S3_BC_DRIVE_DEMO
#define S3_GP28_DEBUG_MS        1000

/* The 2 ms sub-cycle needs a 1 ms system tick (config.h CNF_TIMER_PERIOD).
   With a 10 ms tick, GP28 is sampled only every 10 ms: fine for line
   tracking, too slow for barcode decoding (Commit 4). */
#if defined(CNF_TIMER_PERIOD) && (CNF_TIMER_PERIOD > S3_SUBCYCLE_MS)
#warning "S3: CNF_TIMER_PERIOD is longer than the 2 ms GP28 sub-cycle; barcode sampling will be too slow"
#endif

/* Calibration handshake bits in flg_s3_calib */
#define CALIB_REQ     0x1u   /* caller -> task: start calibrating   */
#define CALIB_OK      0x2u   /* task -> caller: finished, applied    */
#define CALIB_FAIL    0x4u   /* task -> caller: timed out            */

/* GP28 range used until a calibration replaces it (raw ADC counts).
   Measured 2026-10-30 on the robot with the track barcode print:
     white paper ~150-250, wide bars peak ~1450-2800 (dipping to ~1120
     inside a bar), narrow bars only ~+50..+80 above white.
   The 141 mm barcode has narrower wide bars (7.4-9 mm) than the 12 mm
   test print, which may read a little lower, so the range is set with
   margin: with 60 % / 40 % below, dark above ~840, light again below
   ~610.  Every wide bar of both test prints is caught; the narrow-bar
   bumps (and a partly-seen narrow bar at ~780) are not.
   Re-measure (S3_GP28_DEBUG) if the sensor height or paper changes. */
#define S3_GP28_DEFAULT_LO      150u
#define S3_GP28_DEFAULT_HI      1300u

/* LEFT / CENTRE range used until a calibration replaces it (raw ADC).
   Measured 2026-11-04 on the robot (calibration skipped): white ~150-300,
   the black line ~2900-3100.  The old 150..1300 clipped everything darker
   than 1300 to "fully black", so the centre sensor could not show the
   line edge sliding across it (zones 2/3) -- it read 4 until the line had
   gone.  Calibrating (sweep the robot during the 5 s window) replaces it. */
#define S3_LC_DEFAULT_LO        150u
#define S3_LC_DEFAULT_HI        3000u

/* Dark (bar) or light (space) with hysteresis: becomes dark above 60 %
   of the calibrated GP28 range, light again below 40 %. */
#define S3_GP28_DARK_ON    600u
#define S3_GP28_DARK_OFF   400u

/* ---------------------------------------------------------------------
 *  State
 * --------------------------------------------------------------------- */
LOCAL ID            flg_s3_calib;            /* calibration handshake   */
LOCAL volatile bool s_bypass = false;        /* IF-09                   */
LOCAL volatile bool s_gp28_paused = false;   /* tests inject GP28 data  */
LOCAL volatile UW   s_gp28_sent   = 0;       /* samples put in q_gp28_samples */

/* Calibration range per sensor.  Written only by line_sensor_task. */
LOCAL UW s_lo[S3_NUM_SENSORS] = { S3_LC_DEFAULT_LO, S3_LC_DEFAULT_LO, S3_GP28_DEFAULT_LO };
LOCAL UW s_hi[S3_NUM_SENSORS] = { S3_LC_DEFAULT_HI, S3_LC_DEFAULT_HI, S3_GP28_DEFAULT_HI };

typedef struct {
    uint8_t zone_last;              /* zone sent last cycle                 */
    UW      lost_cnt;               /* consecutive line cycles with no line */
    S3_JN   jn;                     /* junction detector                    */
    bool    holding;                /* zone held at 4 over a crossbar       */
} LINE_TRACK;

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
LOCAL UW now_ms(void)
{
    SYSTIM t;
    tk_get_tim(&t);
    return t.lo;            /* wraps after ~49 days; compare with (W)(a-b) */
}

/* One ADC conversion.  Keeps the previous value if the read fails. */
LOCAL UW adc_read(ID dd, INT ch, UW prev)
{
    UW v   = 0;
    SZ asz = 0;
    ER er  = tk_srea_dev(dd, ch, &v, 1, &asz);

    return (er >= E_OK && asz == 1) ? (v & 0x0FFFu) : prev;
}

/* Map a raw reading to 0..1000 where 1000 = fully over the line */
LOCAL UW normalise(UW raw, UW lo, UW hi)
{
    UW n;

    if (hi <= lo)        n = 0;
    else if (raw <= lo)  n = 0;
    else if (raw >= hi)  n = S3_NORM_MAX;
    else                 n = (raw - lo) * S3_NORM_MAX / (hi - lo);

#if !S3_LINE_IS_HIGH
    n = S3_NORM_MAX - n;
#endif
    return n;
}

#if S3_GP28_DEBUG
/* Raw ADC value at a normalised level 0..1000 of the GP28 range */
LOCAL UW gp28_raw_at(UW norm)
{
    UW lo = s_lo[S3_CH_RIGHT], hi = s_hi[S3_CH_RIGHT];
#if !S3_LINE_IS_HIGH
    norm = S3_NORM_MAX - norm;
#endif
    return lo + (hi - lo) * norm / S3_NORM_MAX;
}
#endif

/* true for the sensors a calibration measures and replaces */
LOCAL bool calib_uses(INT ch)
{
#if S3_CALIB_GP28
    (void)ch;
    return true;
#else
    return ch != S3_CH_RIGHT;
#endif
}

/*
 * s3_edge_zone -- line position from the LEFT and CENTRE sensors only.
 *
 *   GP28 (the right sensor) reads the barcode and takes no part in line
 *   tracking.  With two sensors the robot follows the line's LEFT-SENSOR
 *   EDGE: target = centre on black, left on white (zone 4).
 *
 *   2026-11-04: all seven zones are used.  The analog readings are split
 *   at HINT (250), ON (500) and FULL (800), so the error grows in steps of
 *   one zone instead of jumping from 1 to 3 (zones 2 and 6 never occurred
 *   before).  That lets the controller correct early and gently.
 *
 *     left          centre        zone  meaning
 *     >= ON         <  ON         7     line well towards the left sensor
 *     >= ON         >= ON         6     line under both, towards left
 *     HINT..ON      >= HINT       5     left sensor starting to see it
 *     <  HINT       >= FULL       4     ON TARGET
 *     <  HINT       ON..FULL      3     centre sliding off, away from left
 *     <  HINT       HINT..ON      2     centre half off
 *     both off                    1 or 7, from what was seen just before:
 *                                 - previous zone >= 5, or the left
 *                                   sensor is starting to see the line
 *                                   (>= HINT)                        -> 7
 *                                 - otherwise the line slid off the far
 *                                   side of the centre sensor         -> 1
 *                                 then held while the line stays lost.
 *
 *   Pure function (no ADC, no kernel calls): unit-tested by S3-U09.
 *   prev_zone: zone of the previous cycle.  was_on: the line was seen in
 *   the previous cycle.  *on_line: set to whether L or C sees it now
 *   (centre >= HINT counts: zone 2 is still on the line).
 */
EXPORT uint8_t s3_edge_zone(UW n_left, UW n_centre, uint8_t prev_zone,
                            bool was_on, bool *on_line)
{
    bool l  = (n_left   >= S3_ON_LINE_THRESHOLD);
    bool c  = (n_centre >= S3_ON_LINE_THRESHOLD);
    bool lh = (n_left   >= S3_EDGE_HINT);
    bool ch = (n_centre >= S3_EDGE_HINT);

    *on_line = l || ch;

    if (l)  return c ? 6u : 7u;
    if (ch) {
        if (lh)                       return 5u;
        if (n_centre >= S3_EDGE_FULL) return 4u;
        if (c)                        return 3u;
        return 2u;
    }

    /* neither sensor on the line */
    if (!was_on) {
        return prev_zone;                 /* still lost: keep the guess   */
    }
    if (prev_zone >= 5u || lh) {
        return 7u;                        /* it went past the left sensor */
    }
    return 1u;                            /* it slid off towards GP28     */
}

/*
 * s3_junction_step -- junction (crossbar) detector, one call per line cycle.
 *
 *   1. Measures each run of "GP26 and centre both fully black" in mm
 *      (odometry).  A run that ends with a bar's length (S3_JN_MIN_MM ..
 *      S3_JN_MAX_MM) becomes a candidate.
 *   2. The candidate is a junction if GP28 was dark anywhere from
 *      S3_JN_R_BEFORE_MM before the run until S3_JN_R_AFTER_MM after it.
 *      If the car drives on that far with GP28 white, it is dropped
 *      (S3_JN_NO_GP28): the line drifted under GP26 (then GP28 is always on
 *      white), it was not a crossbar.
 *   The flag then stays up S3_JN_HOLD_CYCLES, so a reader polling every
 *   20 ms cannot miss it, and nothing is flagged for S3_JN_REFRACT_CYCLES:
 *   one junction = one pulse.  A robot standing still never finishes a
 *   run of any length.
 *
 *   HOLDZONE while a run is in progress and could be a bar: the line state
 *   reports "on target" so the bar cannot steer the car.  For the first
 *   S3_JN_BLIND_MM always; after that only if GP28 went dark too (a real
 *   crossbar), so a drift of the line under GP26 is corrected normally.
 *   Past S3_JN_MAX_MM it is a curve, and the zone is let through.
 *
 *   Pure function: unit-tested by S3-U10.
 */
EXPORT void s3_junction_init(S3_JN *j)
{
    j->full     = false;
    j->start_mm = 0u;
    j->len_mm   = 0u;
    j->last_mm  = 0u;
    j->pend     = false;
    j->pend_lo  = 0u;
    j->pend_hi  = 0u;
    j->r_seen   = false;
    j->r_mm     = 0u;
    j->hold     = 0u;
    j->refract  = 0u;
    j->active   = false;
}

EXPORT UINT s3_junction_step(S3_JN *j, bool both_full, bool r_dark, uint16_t odo_mm)
{
    UINT ev  = 0u;
    bool bar = false;

    if (r_dark) {
        j->r_seen = true;
        j->r_mm   = odo_mm;
    }

    if (both_full) {
        if (!j->full) {                     /* a new both-full run */
            j->full     = true;
            j->start_mm = odo_mm;
        }
        j->len_mm = (uint16_t)(odo_mm - j->start_mm);   /* wraps: difference */
    } else if (j->full) {                   /* the run just ended */
        j->full    = false;
        j->last_mm = j->len_mm;
        ev |= S3_JN_RUN_END;
        if (j->last_mm >= S3_JN_MIN_MM && j->last_mm <= S3_JN_MAX_MM) {
            j->pend    = true;
            j->pend_lo = (uint16_t)(j->start_mm - S3_JN_R_BEFORE_MM);
            j->pend_hi = (uint16_t)(odo_mm + S3_JN_R_AFTER_MM);
        }
    }

    if (j->pend) {
        if (j->r_seen && (int16_t)(j->r_mm - j->pend_lo) >= 0) {
            j->pend = false;
            bar     = true;                 /* GP28 saw it too: a crossbar */
        } else if ((int16_t)(odo_mm - j->pend_hi) > 0) {
            j->pend = false;
            ev |= S3_JN_NO_GP28;            /* GP28 stayed white */
        }
    }

    if (j->active) {
        if (j->hold > 0u) j->hold--;
        if (j->hold == 0u) {
            j->active  = false;
            j->refract = S3_JN_REFRACT_CYCLES;
            ev |= S3_JN_FALL;
        }
    } else if (j->refract > 0u) {
        j->refract--;
    } else if (bar) {
        j->active = true;
        j->hold   = S3_JN_HOLD_CYCLES;
        ev |= S3_JN_RISE;
    }

    if (j->full && j->len_mm <= S3_JN_MAX_MM) {
        bool r_near = j->r_seen &&
                      (int16_t)(j->r_mm - (uint16_t)(j->start_mm - S3_JN_R_BEFORE_MM)) >= 0;
        if (r_near || j->len_mm <= S3_JN_BLIND_MM) {
            ev |= S3_JN_HOLDZONE;
        }
    }
    return ev;
}

/*
 * compute_line_state
 *   zone 1..7, 4 = on target (see s3_edge_zone).
 *   High zone = line towards the LEFT sensor (GP26): steer towards it.
 *   Low zone  = line away from the left sensor, towards GP28's side.
 *   r_dark: GP28 was dark for most of the 2 ms samples of this cycle.
 *   odo_mm: distance driven (IF-17), for the crossbar length.
 */
LOCAL LINE_DATA_MSG compute_line_state(const UW raw[S3_NUM_SENSORS], bool r_dark,
                                       uint16_t odo_mm, LINE_TRACK *t)
{
    UW      n[S3_NUM_SENSORS];
    bool    any, lost;
    uint8_t z;
    UINT    ev;
    INT     i;

    for (i = 0; i < S3_NUM_SENSORS; i++) {
        n[i] = normalise(raw[i], s_lo[i], s_hi[i]);
    }

    z  = s3_edge_zone(n[S3_CH_LEFT], n[S3_CH_CENTRE], t->zone_last,
                      t->lost_cnt == 0u, &any);
    /* GP28 counts for the junction only while the centre is on the line:
       at a crossbar the main line is still under the centre when GP28
       meets the bar; when the line itself drifts or swings under GP28,
       it has left the centre first. */
    ev = s3_junction_step(&t->jn, n[S3_CH_LEFT] >= S3_EDGE_FULL &&
                                  n[S3_CH_CENTRE] >= S3_EDGE_FULL,
                          r_dark && n[S3_CH_CENTRE] >= S3_ON_LINE_THRESHOLD, odo_mm);

    if ((ev & S3_JN_HOLDZONE) || t->jn.active) {
        /* Possible crossbar under the sensors: it says nothing about where
           the line we follow is.  Report "on target" (zone 4): the
           controller then neither steers on the bar nor builds up its
           curve memory, and the car crosses straight.  (Holding the
           pre-bar zone instead let a 1-zone offset integrate for the whole
           bar into a ~12 deg turn, host simulation 2026-11-04.) */
        t->holding = true;
        z   = LINE_ZONE_CENTRE;
        any = true;
    } else {
        t->holding = false;
    }

#if S3_LINE_DEBUG
    if ((ev & S3_JN_RUN_END) && t->jn.last_mm >= 3u) {
        /* every both-full run: tune S3_JN_MIN_MM / S3_JN_MAX_MM from these */
        tm_printf((UB *)"S3: GP26+C full %u mm  (bar = %u..%u mm)%s\n",
                  (unsigned)t->jn.last_mm, (unsigned)S3_JN_MIN_MM, (unsigned)S3_JN_MAX_MM,
                  t->jn.pend ? ", checking GP28" : "");
    }
    if (ev & S3_JN_NO_GP28) {
        tm_printf((UB *)"S3: ... GP28 stayed white -- not a junction\n");
    }
    if (ev & S3_JN_RISE) {
        tm_printf((UB *)"S3: JUNCTION  (bar %u mm, GP28 dark too)\n", (unsigned)t->jn.last_mm);
    }
#endif

    t->zone_last = z;

    if (any) {
        t->lost_cnt = 0;
    } else if (t->lost_cnt < S3_LOST_DEBOUNCE_CYCLES) {
        t->lost_cnt++;
    }

    /* Debounced line-lost, suppressed while the bypass is active (IF-09) */
    lost = !s_bypass && (t->lost_cnt >= S3_LOST_DEBOUNCE_CYCLES);

    return line_data_pack(z, t->jn.active, lost);
}

/* IF-05: drop-OLDEST when full, so the reader always gets the newest state */
LOCAL void send_line_data(LINE_DATA_MSG m)
{
    LINE_DATA_MSG old;

    if (tk_snd_mbf(q_line_data, &m, sizeof(m), TMO_POL) == E_TMOUT) {
        (void)tk_rcv_mbf(q_line_data, &old, TMO_POL);
        (void)tk_snd_mbf(q_line_data, &m, sizeof(m), TMO_POL);
    }
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
LOCAL void line_sensor_task(INT stacd, void *exinf)
{
    ID          dd;
    UW          raw[S3_NUM_SENSORS] = { 0u, 0u, 0u };
    uint8_t     slot = 0;
    LINE_TRACK  trk;
    UW          now, next_sub, next_line;
    W           wait;

    /* GP28 dark count over the current line cycle (junction detector) */
    bool        gp28_dark   = false;
    UW          gp28_n      = 0u, gp28_dark_n = 0u;

    /* calibration in progress */
    bool        cal_active = false;
    UW          cal_start  = 0, cal_count = 0;
    UW          cal_min[S3_NUM_SENSORS], cal_max[S3_NUM_SENSORS];
#if S3_LINE_DEBUG
    UW          dbg_cycle = 0;
#endif
#if S3_GP28_DEBUG
    UW          gp_min = 4095u, gp_max = 0u, gp_next = 0u;
#endif
    INT         i;

    (void)stacd; (void)exinf;

    trk.zone_last = LINE_ZONE_CENTRE;
    trk.lost_cnt  = 0u;
    trk.holding   = false;
    s3_junction_init(&trk.jn);

    /* This task is the ONLY user of the ADC (TEAM.md section 3) */
    dd = tk_opn_dev((UB *)S3_ADC_DEVNAME, TD_READ);
    if (dd < E_OK) {
        CHECK(dd, "line_sensor_task: tk_opn_dev adca");
        tk_slp_tsk(TMO_FEVR);           /* nothing useful we can do */
    }

    now       = now_ms();
    next_sub  = now;
    next_line = now;
#if S3_GP28_DEBUG
    gp_next   = now + S3_GP28_DEBUG_MS;
#endif

    while (1) {
        /* ---- 2 ms sub-cycle: GP28 sample for the barcode decoder (IF-06) ---- */
        GP28_SAMPLE_MSG s;

        raw[S3_CH_RIGHT] = adc_read(dd, S3_CH_RIGHT, raw[S3_CH_RIGHT]);
        s    = gp28_pack((uint16_t)raw[S3_CH_RIGHT], slot);
        slot = (uint8_t)((slot + 1u) & 0x0Fu);  /* a gap in slots = dropped sample */
        if (!s_gp28_paused) {
            if (tk_snd_mbf(q_gp28_samples, &s, sizeof(s), TMO_POL) == E_OK) {  /* drop-newest */
                s_gp28_sent++;
            }
        }

        /* same dark/light rule (and hysteresis) as the barcode decoder */
        gp28_dark = line_sensor_gp28_dark((uint16_t)raw[S3_CH_RIGHT], gp28_dark);
        gp28_n++;
        if (gp28_dark) gp28_dark_n++;

#if S3_GP28_DEBUG
        if (raw[S3_CH_RIGHT] < gp_min) gp_min = raw[S3_CH_RIGHT];
        if (raw[S3_CH_RIGHT] > gp_max) gp_max = raw[S3_CH_RIGHT];
        now = now_ms();
        if ((W)(now - gp_next) >= 0) {
            gp_next = now + S3_GP28_DEBUG_MS;
            tm_printf((UB *)"GP28: min=%4u max=%4u   (dark %s %u, light %s %u)\n",
                      gp_min, gp_max,
                      S3_LINE_IS_HIGH ? "above" : "below", gp28_raw_at(S3_GP28_DARK_ON),
                      S3_LINE_IS_HIGH ? "below" : "above", gp28_raw_at(S3_GP28_DARK_OFF));
            gp_min = 4095u;
            gp_max = 0u;
        }
#endif

        /* ---- 20 ms line cycle (IF-05) ---- */
        now = now_ms();
        if ((W)(now - next_line) >= 0) {
            bool r_dark;

            next_line += S3_LINE_CYCLE_MS;
            if ((W)(now - next_line) >= 0) {
                next_line = now + S3_LINE_CYCLE_MS;   /* fell behind: resync */
            }

            raw[S3_CH_LEFT]   = adc_read(dd, S3_CH_LEFT,   raw[S3_CH_LEFT]);
            raw[S3_CH_CENTRE] = adc_read(dd, S3_CH_CENTRE, raw[S3_CH_CENTRE]);

            /* GP28 dark for at least half of this cycle's samples */
            r_dark      = (gp28_n > 0u) && (gp28_dark_n * 2u >= gp28_n);
            gp28_n      = 0u;
            gp28_dark_n = 0u;

            /* -- calibration request from another task? -- */
            if (!cal_active) {
                UINT ptn;
                if (tk_wai_flg(flg_s3_calib, CALIB_REQ, TWF_ORW | TWF_BITCLR,
                               &ptn, TMO_POL) == E_OK) {
                    cal_active = true;
                    cal_start  = now;
                    cal_count  = 0;
                    for (i = 0; i < S3_NUM_SENSORS; i++) {
                        cal_min[i] = 4095u;
                        cal_max[i] = 0u;
                    }
                }
            }

            /* -- calibration in progress: track min/max of the calibrated sensors -- */
            if (cal_active) {
                bool spread_ok = true;

                for (i = 0; i < S3_NUM_SENSORS; i++) {
                    if (raw[i] < cal_min[i]) cal_min[i] = raw[i];
                    if (raw[i] > cal_max[i]) cal_max[i] = raw[i];
                    if (calib_uses(i) && cal_max[i] - cal_min[i] < S3_CALIB_MIN_SPREAD) {
                        spread_ok = false;
                    }
                }
                cal_count++;

                if (cal_count >= S3_CALIB_SAMPLES && spread_ok) {
                    for (i = 0; i < S3_NUM_SENSORS; i++) {
                        if (calib_uses(i)) {
                            s_lo[i] = cal_min[i];
                            s_hi[i] = cal_max[i];
                        }
                    }
                    cal_active = false;
                    tm_printf((UB *)"S3 calib: L %u..%u  C %u..%u  R %u..%u%s\n",
                              s_lo[0], s_hi[0], s_lo[1], s_hi[1], s_lo[2], s_hi[2],
                              S3_CALIB_GP28 ? "" : " (R kept)");
                    tk_set_flg(flg_s3_calib, CALIB_OK);
                } else if ((W)(now - cal_start) >= (W)S3_CALIB_TIMEOUT_MS) {
                    cal_active = false;             /* keep the old range */
                    tm_printf((UB *)"S3 calib: timed out, seen L %u..%u  C %u..%u -- range kept\n",
                              cal_min[0], cal_max[0], cal_min[1], cal_max[1]);
                    tk_set_flg(flg_s3_calib, CALIB_FAIL);
                }
            }

            /* -- line state -- */
            {
                LINE_DATA_MSG m = compute_line_state(raw, r_dark,
                                                     s2_odo_dist_mm(s2_get_odometry()), &trk);
                send_line_data(m);

#if S3_LINE_DEBUG
                if (++dbg_cycle % 10u == 0u) {      /* every 200 ms */
                    tm_printf((UB *)"raw L=%4u C=%4u R=%4u  zone=%u jn=%u lost=%u\n",
                              raw[S3_CH_LEFT], raw[S3_CH_CENTRE], raw[S3_CH_RIGHT],
                              line_data_zone(m), line_data_junction(m),
                              line_data_lost(m));
                }
#endif
            }
        }

        /* ---- wait for the next sub-cycle (absolute deadline, no drift) ---- */
        next_sub += S3_SUBCYCLE_MS;
        now  = now_ms();
        wait = (W)(next_sub - now);
        if (wait <= 0) {                     /* overran: resync, still yield */
            next_sub = now + S3_SUBCYCLE_MS;
            wait     = S3_SUBCYCLE_MS;
        }
        tk_dly_tsk((TMO)wait);
    }
}

/* ---------------------------------------------------------------------
 *  Called from subsystem3.c
 * --------------------------------------------------------------------- */

/* Phase 1 (subsystem3_init): create this file's kernel objects */
EXPORT ER line_sensor_init(void)
{
    T_CFLG cflg = {0};

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    flg_s3_calib = tk_cre_flg(&cflg);
    return (flg_s3_calib < E_OK) ? flg_s3_calib : E_OK;
}

/* Phase 2 (subsystem3_start): start the task */
EXPORT ER line_sensor_start(void)
{
    ID tid = app_start_task(line_sensor_task, S3_LINE_TASK_PRI, 0, "line_sensor_task");
    return (tid < E_OK) ? tid : E_OK;
}

/* ---------------------------------------------------------------------
 *  Public functions (subsystem3.h)
 * --------------------------------------------------------------------- */

/* Blocks the CALLING task for up to S3_CALIB_TIMEOUT_MS while the robot is
   swept across the line.  Never call it from line_sensor_task itself. */
EXPORT bool subsystem3_run_calibration(void)
{
    UINT ptn = 0;
    ER   er;

    if (flg_s3_calib <= 0) return false;

    /* clear old results (tk_clr_flg keeps only the bits set in the mask) */
    tk_clr_flg(flg_s3_calib, ~(UINT)(CALIB_OK | CALIB_FAIL));
    tk_set_flg(flg_s3_calib, CALIB_REQ);

    er = tk_wai_flg(flg_s3_calib, CALIB_OK | CALIB_FAIL, TWF_ORW | TWF_BITCLR,
                    &ptn, (TMO)(S3_CALIB_TIMEOUT_MS + 500u));

    return (er == E_OK) && ((ptn & CALIB_OK) != 0u);
}

/* IF-09: while active, LINE_LOST is suppressed (robot intentionally off line) */
EXPORT void subsystem3_set_bypass(bool active)
{
    s_bypass = active;
}

/* ---------------------------------------------------------------------
 *  Shared with barcode_decode.c and the tests (s3_private.h)
 * --------------------------------------------------------------------- */

EXPORT bool line_sensor_gp28_dark(uint16_t adc, bool was_dark)
{
    UW n = normalise(adc, s_lo[S3_CH_RIGHT], s_hi[S3_CH_RIGHT]);
    return was_dark ? (n >= S3_GP28_DARK_OFF) : (n >= S3_GP28_DARK_ON);
}

/* Number of GP28 samples successfully put into q_gp28_samples so far */
EXPORT UW line_sensor_gp28_sent(void)
{
    return s_gp28_sent;
}

EXPORT void line_sensor_pause_gp28(bool pause)
{
    s_gp28_paused = pause;
}

EXPORT uint16_t line_sensor_gp28_test_level(bool dark)
{
#if S3_LINE_IS_HIGH
    return dark ? 4095u : 0u;
#else
    return dark ? 0u : 4095u;
#endif
}
