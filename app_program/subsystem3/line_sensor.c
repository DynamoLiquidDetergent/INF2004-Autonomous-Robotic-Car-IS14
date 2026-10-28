/*
 *	line_sensor.c
 *	Subsystem 3 -- Line Sensor Task (owning task of the ADC, TEAM.md section 3).
 *
 *	Every 2 ms  (sub-cycle):  read GP28 (ADC2), send GP28_SAMPLE_MSG to
 *	                          q_gp28_samples (IF-06), drop-newest when full.
 *	Every 20 ms (line cycle): read GP26 (ADC0) and GP27 (ADC1), compute the
 *	                          line state, send LINE_DATA_MSG to q_line_data
 *	                          (IF-05), drop-oldest when full so the reader
 *	                          always gets fresh data.
 *
 *	Calibration: subsystem3_run_calibration() (called from ANOTHER task)
 *	asks this task to record the min/max of each sensor while the robot is
 *	swept across the line.  Only this task ever touches the ADC.
 *
 *	Bypass (IF-09): subsystem3_set_bypass(true) suppresses LINE_LOST.
 *	The Safety Monitor fault bit (IF-08) is added in Commit 5.
 *
 *	APIs used: tk_opn_dev, tk_srea_dev, tk_snd_mbf, tk_rcv_mbf, tk_dly_tsk,
 *	           tk_get_tim, tk_cre_flg, tk_set_flg, tk_clr_flg, tk_wai_flg
 */

#include "subsystem3.h"

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
#define S3_JN_DEBOUNCE_CYCLES   2        /* 2 x 20 ms = 40 ms              */

/* 1 = the ADC reads HIGHER over the dark line than over the white floor.
   Check this on the bench with S3_LINE_DEBUG (raw values are printed);
   set to 0 if your sensors read lower over the line. */
#define S3_LINE_IS_HIGH         1

#define S3_NORM_MAX             1000u    /* normalised scale 0..1000       */
#define S3_ON_LINE_THRESHOLD    500u     /* normalised value = "on line"   */
#define S3_CALIB_MIN_SPREAD     300u     /* min (max-min) ADC counts needed */

/* 1 = print raw values and the line state every 200 ms (Commit 3 Part B).
   MUST be 0 when committing. */
#define S3_LINE_DEBUG           0

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

/* ---------------------------------------------------------------------
 *  State
 * --------------------------------------------------------------------- */
LOCAL ID            flg_s3_calib;            /* calibration handshake   */
LOCAL volatile bool s_bypass = false;        /* IF-09                   */

/* Calibration range per sensor.  Written only by line_sensor_task. */
LOCAL UW s_lo[S3_NUM_SENSORS] = { 0u,    0u,    0u    };
LOCAL UW s_hi[S3_NUM_SENSORS] = { 4095u, 4095u, 4095u };

typedef struct {
    uint8_t zone_last;   /* last zone seen with the line under a sensor */
    UW      lost_cnt;    /* consecutive line cycles with no line         */
    UW      jn_cnt;      /* consecutive line cycles with a junction      */
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

/*
 * compute_line_state
 *   zone 1..7, 4 = line under the centre sensor.
 *   Low zone  = line under the RIGHT sensor (robot has drifted left).
 *   High zone = line under the LEFT sensor  (robot has drifted right).
 *   (Direction to confirm with the S5 owner -- IF-05 only fixes 4 = centre.)
 *   While the line is lost the last valid zone is repeated.
 */
LOCAL LINE_DATA_MSG compute_line_state(const UW raw[S3_NUM_SENSORS], LINE_TRACK *t)
{
    UW   n[S3_NUM_SENSORS];
    bool on[S3_NUM_SENSORS];
    bool any, jn_now, jn, lost;
    INT  i;

    for (i = 0; i < S3_NUM_SENSORS; i++) {
        n[i]  = normalise(raw[i], s_lo[i], s_hi[i]);
        on[i] = (n[i] >= S3_ON_LINE_THRESHOLD);
    }
    any = on[S3_CH_LEFT] || on[S3_CH_CENTRE] || on[S3_CH_RIGHT];

    if (any) {
        /* Weighted position: left = 2, centre = 1, right = 0  -> 0..2.
           zone = 1 + round(3 * pos)  -> 1..7, centre = 4. */
        UW den  = n[S3_CH_LEFT] + n[S3_CH_CENTRE] + n[S3_CH_RIGHT];
        UW num  = 2u * n[S3_CH_LEFT] + n[S3_CH_CENTRE];
        UW zone = 1u + (6u * num + den) / (2u * den);

        if (zone > 7u) zone = 7u;
        t->zone_last = (uint8_t)zone;
        t->lost_cnt  = 0;
    } else if (t->lost_cnt < S3_LOST_DEBOUNCE_CYCLES) {
        t->lost_cnt++;
    }

    /* Junction: two neighbouring sensors on the line together */
    jn_now = (on[S3_CH_LEFT] && on[S3_CH_CENTRE]) ||
             (on[S3_CH_CENTRE] && on[S3_CH_RIGHT]);
    if (!jn_now)                              t->jn_cnt = 0;
    else if (t->jn_cnt < S3_JN_DEBOUNCE_CYCLES) t->jn_cnt++;
    jn = (t->jn_cnt >= S3_JN_DEBOUNCE_CYCLES);

    /* Debounced line-lost, suppressed while the bypass is active (IF-09) */
    lost = !s_bypass && (t->lost_cnt >= S3_LOST_DEBOUNCE_CYCLES);

    return line_data_pack(t->zone_last, jn, lost);
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
    LINE_TRACK  trk  = { LINE_ZONE_CENTRE, 0u, 0u };
    UW          now, next_sub, next_line;
    W           wait;

    /* calibration in progress */
    bool        cal_active = false;
    UW          cal_start  = 0, cal_count = 0;
    UW          cal_min[S3_NUM_SENSORS], cal_max[S3_NUM_SENSORS];
#if S3_LINE_DEBUG
    UW          dbg_cycle = 0;
#endif
    INT         i;

    (void)stacd; (void)exinf;

    /* This task is the ONLY user of the ADC (TEAM.md section 3) */
    dd = tk_opn_dev((UB *)S3_ADC_DEVNAME, TD_READ);
    if (dd < E_OK) {
        CHECK(dd, "line_sensor_task: tk_opn_dev adca");
        tk_slp_tsk(TMO_FEVR);           /* nothing useful we can do */
    }

    now       = now_ms();
    next_sub  = now;
    next_line = now;

    while (1) {
        /* ---- 2 ms sub-cycle: GP28 sample for the barcode decoder (IF-06) ---- */
        GP28_SAMPLE_MSG s;

        raw[S3_CH_RIGHT] = adc_read(dd, S3_CH_RIGHT, raw[S3_CH_RIGHT]);
        s    = gp28_pack((uint16_t)raw[S3_CH_RIGHT], slot);
        slot = (uint8_t)((slot + 1u) & 0x0Fu);  /* a gap in slots = dropped sample */
        (void)tk_snd_mbf(q_gp28_samples, &s, sizeof(s), TMO_POL);  /* drop-newest */

        /* ---- 20 ms line cycle (IF-05) ---- */
        now = now_ms();
        if ((W)(now - next_line) >= 0) {
            next_line += S3_LINE_CYCLE_MS;
            if ((W)(now - next_line) >= 0) {
                next_line = now + S3_LINE_CYCLE_MS;   /* fell behind: resync */
            }

            raw[S3_CH_LEFT]   = adc_read(dd, S3_CH_LEFT,   raw[S3_CH_LEFT]);
            raw[S3_CH_CENTRE] = adc_read(dd, S3_CH_CENTRE, raw[S3_CH_CENTRE]);

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

            /* -- calibration in progress: track min/max of every sensor -- */
            if (cal_active) {
                bool spread_ok = true;

                for (i = 0; i < S3_NUM_SENSORS; i++) {
                    if (raw[i] < cal_min[i]) cal_min[i] = raw[i];
                    if (raw[i] > cal_max[i]) cal_max[i] = raw[i];
                    if (cal_max[i] - cal_min[i] < S3_CALIB_MIN_SPREAD) spread_ok = false;
                }
                cal_count++;

                if (cal_count >= S3_CALIB_SAMPLES && spread_ok) {
                    for (i = 0; i < S3_NUM_SENSORS; i++) {
                        s_lo[i] = cal_min[i];
                        s_hi[i] = cal_max[i];
                    }
                    cal_active = false;
                    tm_printf((UB *)"S3 calib: L %u..%u  C %u..%u  R %u..%u\n",
                              s_lo[0], s_hi[0], s_lo[1], s_hi[1], s_lo[2], s_hi[2]);
                    tk_set_flg(flg_s3_calib, CALIB_OK);
                } else if ((W)(now - cal_start) >= (W)S3_CALIB_TIMEOUT_MS) {
                    cal_active = false;             /* keep the old range */
                    tk_set_flg(flg_s3_calib, CALIB_FAIL);
                }
            }

            /* -- line state -- */
            {
                LINE_DATA_MSG m = compute_line_state(raw, &trk);
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
