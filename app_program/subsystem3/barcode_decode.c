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
 *	APIs used: tk_rcv_mbf, tk_snd_mbf
 */

#include "s3_private.h"
#include "code39.h"

/* ---------------------------------------------------------------------
 *  Configuration
 * --------------------------------------------------------------------- */
#define BC_ELEMS              29u   /* * gap X gap *                     */
#define BC_QUIET_SAMPLES      40u   /* light run this long ends a read   */
#define BC_MAP_POLL_MS        20    /* check mbx_char_map at least this often */

/* 1 = start with mapping A,B,C,D (forward, U-turn, left, right) so a
   live scan works before S1 sends one (Commit 4 Part B).  MUST be 0
   when committing. */
#define S3_BC_TEST_MAP        1 //was 0

/* 1 = print every decode attempt (Commit 4 Part B).  MUST be 0 when
   committing. */
#define S3_BC_DEBUG           1 //was 0

/* ---------------------------------------------------------------------
 *  State
 * --------------------------------------------------------------------- */
LOCAL CHAR_MAP_MSG s_char_map;
LOCAL bool         s_map_valid = false;

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

/* Take the newest mapping, if any.  Invalid ones are dropped silently. */
LOCAL void poll_char_map(void)
{
    CHAR_MAP_MSG m;

    while (tk_rcv_mbf(mbx_char_map, &m, TMO_POL) == (INT)sizeof(m)) {
        if (s3_char_map_valid(&m)) {
            s_char_map  = m;
            s_map_valid = true;
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
 *  Element collection
 * --------------------------------------------------------------------- */
LOCAL void abandon(BC_STATE *st)
{
    st->collecting = false;
    st->n          = 0;
    st->window     = 0;
}

LOCAL void decode_and_send(BC_STATE *st)
{
    uint8_t     ch;
    BARCODE_CMD cmd = BARCODE_CMD_NONE;

    ch = code39_decode_symbol(st->elems, BC_MIN_NARROW_SAMPLES, BC_WIDE_RATIO_X10);

    if (ch >= 'A' && ch <= 'Z') {
        send_latest_u8(q_decoded_char, ch);
        cmd = resolve(ch);
        if (cmd != BARCODE_CMD_NONE) {
            send_latest_u8(q_barcode_cmd, cmd);
        }
    }

#if S3_BC_DEBUG
    if (ch >= 'A' && ch <= 'Z') {
        tm_printf((UB *)"BARCODE: ch=%c cmd=%u\n", (char)ch, (unsigned)cmd);
    } else {
        tm_printf((UB *)"BARCODE: 29 elements, no valid symbol\n");
    }
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
        abandon(st);
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
            abandon(st);
        }
    }

    if (dark == st->dark) {                           /* same run */
        if (st->run < 0xFFFFu) st->run++;
        if (st->collecting && !dark && st->run >= BC_QUIET_SAMPLES) {
            abandon(st);                              /* barcode ended early */
        }
        return;
    }

    /* Level changed: the run that just ended is one element */
    if (st->collecting) {
        st->elems[st->n].w = st->run;
        st->n++;
        if (st->n == BC_ELEMS) {                      /* stop marker's last bar */
            decode_and_send(st);
            abandon(st);
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
    s_char_map  = (CHAR_MAP_MSG){ 'A', 'B', 'C', 'D' };
    s_map_valid = true;
#endif

    while (1) {
        /* Wait for a sample, but wake at least every BC_MAP_POLL_MS so a
           new mapping is picked up even when no samples arrive. */
        rc = tk_rcv_mbf(q_gp28_samples, &s, BC_MAP_POLL_MS);

        poll_char_map();

        if (rc == (INT)sizeof(s)) {
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
