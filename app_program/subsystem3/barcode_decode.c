/*
 *	barcode_decode.c
 *	Subsystem 3 -- Barcode Decode Task.
 *
 *	Reads GP28 samples (IF-06), turns them into bar/space element widths,
 *	and when 29 elements (start '*', gap, data, gap, stop '*') are
 *	collected, decodes them with code39_decode_symbol() in either scan
 *	direction.  A valid letter A-Z is
 *	  - sent to q_decoded_char  (IF-07, the letter), and
 *	  - mapped through the current CHAR_MAP_MSG to a BARCODE_CMD and sent
 *	    to q_barcode_cmd       (IF-07, the movement command).
 *	The mapping arrives on mbx_char_map (from S1 / MQTT).  Invalid
 *	mappings are dropped silently and the previous one stays in force.
 *
 *	A read is abandoned when:
 *	  - a slot index is skipped (a GP28 sample was dropped),
 *	  - a light run reaches BC_QUIET_SAMPLES before 29 elements,
 *	  - more than BC_MAX_WINDOW_SAMPLES samples pass before 29 elements.
 *
 *	A second decoder (code39_wide.c, S3_BC_WIDE_DECODER) runs on the same
 *	samples.  It finds the letter from the positions of the 6 WIDE bars,
 *	for prints where GP28 cannot threshold the narrow bars (the track
 *	barcode: the sensor spot is wider than a narrow bar).  Look-alike
 *	letters (same wide bars) are told apart by the BLURRED shape their
 *	narrow bars give the reading (see code39_wide.h).  A read that is not
 *	clear-cut is reported "unsure" and NOT sent.  Once a mapping arrives,
 *	only its 4 letters are considered (only those can be on the track).  Whichever decoder reads the barcode first reports it; the
 *	same letter again within BC_DEDUP_SAMPLES is ignored, so one barcode =
 *	one report.
 *
 *	APIs used: tk_rcv_mbf, tk_snd_mbf
 */

#include "s3_private.h"
#include "code39.h"
#include "code39_wide.h"

/* ---------------------------------------------------------------------
 *  Configuration
 * --------------------------------------------------------------------- */
#define BC_ELEMS              29u   /* * gap X gap *                     */

/* Timing for the 141 mm track barcode ("*X*": narrow ~3.0-3.7 mm,
   wide ~7.4-9 mm), in 2 ms samples, valid from 50 to 150 mm/s.

   A light run this long ends a read.  It must be longer than the widest
   SPACE (9 mm = 90 samples at 50 mm/s) and shorter than the white margin
   around the barcode.  100 samples = 20 mm at 100 mm/s. */
#define BC_QUIET_SAMPLES      100u

/* Longest a whole symbol may take.  Overrides the 800 in subsystem3.h.
   141 mm at 50 mm/s = 1410 samples; 1600 leaves ~10 % margin. */
#undef  BC_MAX_WINDOW_SAMPLES
#define BC_MAX_WINDOW_SAMPLES 1600u

#define BC_MAP_POLL_MS        20    /* check mbx_char_map at least this often */

/* 1 = also run the wide-bar decoder (code39_wide.c).  Needed for the
   track barcode print, whose narrow bars GP28 cannot see. */
#define S3_BC_WIDE_DECODER    1

/* The same letter reported again within this many samples is the same
   barcode (both decoders finish it within a few mm of each other):
   ignore it.  500 samples = 1 s = 10 cm at 100 mm/s, so two barcodes with
   the SAME letter must be more than ~10 cm apart. */
#define BC_DEDUP_SAMPLES      500u

/* 1 = start with mapping A,B,C,D (forward, U-turn, left, right) so a
   live scan works before S1 sends one (Commit 4 Part B).  MUST be 0
   when committing. */
#define S3_BC_TEST_MAP        S3_BC_DRIVE_DEMO   /* on with the drive demo */

/* 1 = print every decode attempt (Commit 4 Part B).  MUST be 0 when
   committing. */
#define S3_BC_DEBUG           S3_BC_DRIVE_DEMO   /* on with the drive demo */

/* With S3_BC_DEBUG: a read abandoned after at least this many elements is
   reported, with the element widths.  Fewer = a speck, the floor edge, or
   the track print (only its 6 wide bars show, giving 11 elements -- the
   wide-bar decoder reads those), not worth printing. */
#define BC_DEBUG_MIN_ELEMS    12u

/* 1 = record the raw GP28 readings of the first barcode pass (from the
   first reading above BC_CAP_TRIGGER, BC_CAP_LEN samples = 3 s) so
   the drive demo can print them ("CAP" lines) for tuning.
   Set to S3_BC_DRIVE_DEMO to switch it on with the drive demo. */
#define S3_BC_CAPTURE         0
#define BC_CAP_LEN            1500u   /* 3 s: the 141 mm barcode at >= 50 mm/s */
#define BC_CAP_TRIGGER        400u    /* raw ADC: clearly above white      */
#define BC_CAP_PRE            25u     /* samples kept from before trigger  */

/* ---------------------------------------------------------------------
 *  State
 * --------------------------------------------------------------------- */
LOCAL CHAR_MAP_MSG s_char_map;
LOCAL bool         s_map_valid = false;
LOCAL volatile UW  s_decodes   = 0;      /* valid letters decoded */

EXPORT UW barcode_decode_count(void)
{
    return s_decodes;
}

/* ---------------------------------------------------------------------
 *  Raw capture (debug, S3_BC_CAPTURE)
 * --------------------------------------------------------------------- */
#if S3_BC_CAPTURE
LOCAL uint16_t      s_cap[BC_CAP_LEN];
LOCAL uint16_t      s_pre[BC_CAP_PRE];        /* ring of recent samples */
LOCAL UW            s_pre_i   = 0;
LOCAL UW            s_cap_n   = 0;
LOCAL volatile bool s_cap_on  = false;
LOCAL volatile bool s_cap_done = false;

LOCAL void capture_feed(uint16_t adc)
{
    UW i;

    if (s_cap_done) return;
    if (!s_cap_on) {
        if (adc >= BC_CAP_TRIGGER && s_pre_i >= BC_CAP_PRE) {
            for (i = 0; i < BC_CAP_PRE; i++) {        /* oldest first */
                s_cap[i] = s_pre[(s_pre_i + i) % BC_CAP_PRE];
            }
            s_cap_n  = BC_CAP_PRE;
            s_cap_on = true;
        } else {
            s_pre[s_pre_i % BC_CAP_PRE] = adc;
            s_pre_i++;
            return;
        }
    }
    s_cap[s_cap_n++] = adc;
    if (s_cap_n >= BC_CAP_LEN) {
        s_cap_done = true;
    }
}
#endif

/* Number of captured samples once the capture is complete, else 0.
   *buf is set to the samples (2 ms apart). */
EXPORT UW barcode_capture(const uint16_t **buf)
{
#if S3_BC_CAPTURE
    if (!s_cap_done) return 0;
    *buf = s_cap;
    return s_cap_n;
#else
    (void)buf;
    return 0;
#endif
}

typedef struct {
    bool     have_slot;          /* last_slot is valid                  */
    uint8_t  last_slot;
    bool     have_level;         /* dark / run are valid                */
    bool     dark;               /* level of the current run            */
    uint16_t run;                /* samples in the current run          */
    bool     collecting;         /* inside a symbol                     */
    uint16_t n;                  /* elements collected                  */
    uint16_t window;             /* samples since the symbol started    */
    BcElem   elems[BC_ELEMS];
} BC_STATE;

/* ---------------------------------------------------------------------
 *  Character mapping
 * --------------------------------------------------------------------- */
EXPORT bool s3_char_map_valid(const CHAR_MAP_MSG *m)
{
    return (m->ch_forward >= 'A' && m->ch_forward <= 'Z') &&
           (m->ch_uturn   >= 'A' && m->ch_uturn   <= 'Z') &&
           (m->ch_left    >= 'A' && m->ch_left    <= 'Z') &&
           (m->ch_right   >= 'A' && m->ch_right   <= 'Z') &&
           (m->ch_forward != m->ch_uturn)  &&
           (m->ch_forward != m->ch_left)   &&
           (m->ch_forward != m->ch_right)  &&
           (m->ch_uturn   != m->ch_left)   &&
           (m->ch_uturn   != m->ch_right)  &&
           (m->ch_left    != m->ch_right);
}

#if S3_BC_WIDE_DECODER
LOCAL void wide_use_map(const CHAR_MAP_MSG *m);   /* below */
#endif

/* Take the newest mapping, if any.  Invalid ones are dropped silently.
   Only the 4 mapped letters can appear on the track, so the wide-bar
   decoder is told to consider just those 4. */
LOCAL void poll_char_map(void)
{
    CHAR_MAP_MSG m;

    while (tk_rcv_mbf(mbx_char_map, &m, TMO_POL) == (INT)sizeof(m)) {
        if (s3_char_map_valid(&m)) {
            s_char_map  = m;
            s_map_valid = true;
#if S3_BC_WIDE_DECODER
            wide_use_map(&m);
#endif
        }
    }
}

/* Letter -> movement command, using the current mapping */
LOCAL BARCODE_CMD resolve(uint8_t ch)
{
    if (!s_map_valid)                return BARCODE_CMD_NONE;
    if (ch == s_char_map.ch_forward) return BARCODE_CMD_FORWARD;
    if (ch == s_char_map.ch_uturn)   return BARCODE_CMD_UTURN;
    if (ch == s_char_map.ch_left)    return BARCODE_CMD_TURN_LEFT;
    if (ch == s_char_map.ch_right)   return BARCODE_CMD_TURN_RIGHT;
    return BARCODE_CMD_NONE;
}

/* Send a 1-byte message, dropping the OLDEST one if the buffer is full */
LOCAL void send_latest_u8(ID mbf, uint8_t v)
{
    uint8_t old;

    if (tk_snd_mbf(mbf, &v, sizeof(v), TMO_POL) == E_TMOUT) {
        (void)tk_rcv_mbf(mbf, &old, TMO_POL);
        (void)tk_snd_mbf(mbf, &v, sizeof(v), TMO_POL);
    }
}

/* ---------------------------------------------------------------------
 *  Reporting (both decoders)
 * --------------------------------------------------------------------- */
LOCAL UW      s_samples   = 0;      /* GP28 samples processed: the clock  */
LOCAL uint8_t s_last_ch   = 0;      /* last letter reported ...           */
LOCAL UW      s_last_at   = 0;      /* ... and when (s_samples)           */

/* Send a decoded letter and its command (IF-07), once per barcode.
   Returns false if it was a duplicate and nothing was sent. */
LOCAL bool report_letter(uint8_t ch, BARCODE_CMD *cmd)
{
    *cmd = BARCODE_CMD_NONE;
    if (s_last_ch == ch && (s_samples - s_last_at) < BC_DEDUP_SAMPLES) {
        return false;                       /* same barcode again */
    }
    s_last_ch = ch;
    s_last_at = s_samples;

    send_latest_u8(q_decoded_char, ch);
    s_decodes++;
    *cmd = resolve(ch);
    if (*cmd != BARCODE_CMD_NONE) {
        send_latest_u8(q_barcode_cmd, *cmd);
    }
    return true;
}

/* ---------------------------------------------------------------------
 *  Wide-bar decoder (code39_wide.c)
 * --------------------------------------------------------------------- */
#if S3_BC_WIDE_DECODER
LOCAL C39W_STATE s_wb;              /* 4 KB: static, not on the task stack */
LOCAL bool       s_wb_dark = false;

/* Only the mapped letters can be on the track: the wide-bar decoder then
   never mistakes one of them for a look-alike that is not there. */
LOCAL void wide_use_map(const CHAR_MAP_MSG *m)
{
    c39w_set_letters(&s_wb, (1ul << (m->ch_forward - 'A')) |
                            (1ul << (m->ch_uturn   - 'A')) |
                            (1ul << (m->ch_left    - 'A')) |
                            (1ul << (m->ch_right   - 'A')));
#if S3_BC_DEBUG
    tm_printf((UB *)"BARCODE: mapping %c %c %c %c -- only these letters are read\n",
              (char)m->ch_forward, (char)m->ch_uturn,
              (char)m->ch_left, (char)m->ch_right);
#endif
}

LOCAL void wide_sample(uint16_t adc)
{
    C39W_RESULT r;
    BARCODE_CMD cmd;
    bool        sent;

    s_wb_dark = line_sensor_gp28_dark(adc, s_wb_dark);
    if (!c39w_feed(&s_wb, adc, s_wb_dark, &r)) {
        return;
    }

    if (r.ch != C39W_NONE && !r.uncertain) {
        sent = report_letter(r.ch, &cmd);
#if S3_BC_DEBUG
        if (sent) {
            tm_printf((UB *)"BARCODE: ch=%c cmd=%u   (wide bars, %s, fit %u.%02u, ratio %u.%02u",
                      (char)r.ch, (unsigned)cmd, r.reversed ? "reverse" : "forward",
                      r.err_x100 / 100u, r.err_x100 % 100u,
                      r.ratio_x100 / 100u, r.ratio_x100 % 100u);
            if (r.ntied > 1u && r.ch2 != C39W_NONE) {
                /* blurred narrow-bar shape: error of the winner vs the
                   runner-up (ADC rms), how clearly it won, narrow-bar lift */
                tm_printf((UB *)", beat %c: error %u.%u vs %u.%u, evidence %u, narrow +%d",
                          (char)r.ch2, r.rms_x10 / 10u, r.rms_x10 % 10u,
                          r.rms2_x10 / 10u, r.rms2_x10 % 10u,
                          (unsigned)r.evidence, (int)r.narrow);
            }
            tm_printf((UB *)")\n");
        }
#else
        (void)sent;
#endif
    }
#if S3_BC_DEBUG
    else if (r.ch != C39W_NONE && r.ch2 != C39W_NONE) {
        tm_printf((UB *)"BARCODE: unsure, %c or %c (error %u.%u vs %u.%u, evidence %u, narrow %d, %u samples) -- not sent\n",
                  (char)r.ch, (char)r.ch2, r.rms_x10 / 10u, r.rms_x10 % 10u,
                  r.rms2_x10 / 10u, r.rms2_x10 % 10u, (unsigned)r.evidence,
                  (int)r.narrow, (unsigned)r.nsamp);
    } else if (r.ch != C39W_NONE) {
        tm_printf((UB *)"BARCODE: unsure, %c (narrow-bar shape not usable) -- not sent\n",
                  (char)r.ch);
    } else {
        tm_printf((UB *)"BARCODE: 6 wide bars, no letter fits (error %u.%02u)\n",
                  r.err_x100 / 100u, r.err_x100 % 100u);
    }
#endif
}
#endif /* S3_BC_WIDE_DECODER */

/* ---------------------------------------------------------------------
 *  Element collection
 * --------------------------------------------------------------------- */
#if S3_BC_DEBUG
/* Element widths in samples (x 2 ms), bars and spaces alternating,
   starting with a bar */
LOCAL void print_widths(const BC_STATE *st)
{
    uint16_t i;

    tm_printf((UB *)"BARCODE:   widths:");
    for (i = 0; i < st->n; i++) {
        tm_printf((UB *)" %u", (unsigned)st->elems[i].w);
    }
    tm_printf((UB *)"\n");
}
#endif

/* why = reason for the debug print, or NULL after a completed decode */
LOCAL void abandon(BC_STATE *st, const char *why)
{
#if S3_BC_DEBUG
    if (why != NULL && st->collecting && st->n >= BC_DEBUG_MIN_ELEMS) {
        tm_printf((UB *)"BARCODE: gave up after %u of %u elements (%s)\n",
                  (unsigned)st->n, (unsigned)BC_ELEMS, why);
        print_widths(st);
    }
#else
    (void)why;
#endif
    st->collecting = false;
    st->n          = 0;
    st->window     = 0;
}

LOCAL void decode_and_send(BC_STATE *st)
{
    uint8_t     ch;
    BARCODE_CMD cmd = BARCODE_CMD_NONE;
    bool        sent = false;

    ch = code39_decode_symbol(st->elems, BC_MIN_NARROW_SAMPLES, BC_WIDE_RATIO_X10);

    if (ch >= 'A' && ch <= 'Z') {
        sent = report_letter(ch, &cmd);
    }

#if S3_BC_DEBUG
    if (ch >= 'A' && ch <= 'Z') {
        if (sent) {
            tm_printf((UB *)"BARCODE: ch=%c cmd=%u   (all 29 elements)\n",
                      (char)ch, (unsigned)cmd);
        }
    } else {
        tm_printf((UB *)"BARCODE: 29 elements, no valid symbol\n");
        print_widths(st);
    }
#else
    (void)sent;
#endif
}

/* Feed one GP28 sample into the element state machine */
LOCAL void process_sample(BC_STATE *st, GP28_SAMPLE_MSG s)
{
    uint8_t slot = gp28_slot(s);
    bool    dark;

    /* A skipped slot means a sample was dropped: widths are no longer
       trustworthy, so start again from scratch. */
    if (st->have_slot && slot != (uint8_t)((st->last_slot + 1u) & 0x0Fu)) {
        abandon(st, "GP28 sample dropped");
        st->have_level = false;
    }
    st->last_slot = slot;
    st->have_slot = true;

    dark = line_sensor_gp28_dark(gp28_adc(s), st->have_level ? st->dark : false);

    if (!st->have_level) {
        st->have_level = true;
        st->dark       = dark;
        st->run        = 1;
        return;
    }

    if (st->collecting) {
        st->window++;
        if (st->window > BC_MAX_WINDOW_SAMPLES) {     /* far too long */
            abandon(st, "took too long, BC_MAX_WINDOW_SAMPLES");
        }
    }

    if (dark == st->dark) {                           /* same run */
        if (st->run < 0xFFFFu) st->run++;
        if (st->collecting && !dark && st->run >= BC_QUIET_SAMPLES) {
            abandon(st, "long white gap, BC_QUIET_SAMPLES");  /* barcode ended early */
        }
        return;
    }

    /* Level changed: the run that just ended is one element */
    if (st->collecting) {
        st->elems[st->n].w = st->run;
        st->n++;
        if (st->n == BC_ELEMS) {                      /* stop marker's last bar */
            decode_and_send(st);
            abandon(st, NULL);
        }
    } else if (dark) {
        /* light -> dark while idle: first bar of a possible symbol */
        st->collecting = true;
        st->n          = 0;
        st->window     = 0;
    }

    st->dark = dark;
    st->run  = 1;
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
LOCAL void barcode_decode_task(INT stacd, void *exinf)
{
    BC_STATE        st = {0};
    GP28_SAMPLE_MSG s;
    INT             rc;

    (void)stacd; (void)exinf;

#if S3_BC_TEST_MAP
    s_char_map  = (CHAR_MAP_MSG){ 'A', 'Z', 'C', 'D' };
    s_map_valid = true;
#if S3_BC_WIDE_DECODER
    wide_use_map(&s_char_map);
#endif
#endif

    while (1) {
        /* Wait for a sample, but wake at least every BC_MAP_POLL_MS so a
           new mapping is picked up even when no samples arrive. */
        rc = tk_rcv_mbf(q_gp28_samples, &s, BC_MAP_POLL_MS);

        poll_char_map();

        if (rc == (INT)sizeof(s)) {
            s_samples++;
#if S3_BC_CAPTURE
            capture_feed(gp28_adc(s));
#endif
#if S3_BC_WIDE_DECODER
            wide_sample(gp28_adc(s));
#endif
            process_sample(&st, s);
        }
    }
}

/* Phase 2 (subsystem3_start): start the task */
EXPORT ER barcode_decode_start(void)
{
    ID tid = app_start_task(barcode_decode_task, S3_BC_TASK_PRI, 0, "barcode_decode_task");
    return (tid < E_OK) ? tid : E_OK;
}
