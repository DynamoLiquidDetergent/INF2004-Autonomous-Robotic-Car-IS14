/*
 * subsystem1.c
 * Subsystem 1 (Network Gatekeeper): creates objects, starts tasks.
 */

#include "subsystem1.h"
#include "telemetry.h"
#include "cmd_parse.h"
#include "s1_topics.h"
#include "telem_layout.h"
#include "telem_pack.h"

#if defined(TM_MQTT) && (TM_MQTT == 1)
#include "mqtt_utk.h"
#endif

/* Message buffer depths (messages).  The kernel stores each message as a
   4-byte header plus the message rounded up to 4 bytes. */
#define S1_MBF_SLOT(msz)     (4U + ((((UW)(msz)) + 3U) & ~3UL))
#define S1_TX_DEPTH          (8U)   /* ~2.5 s of output at 3 msg/s     */
#define S1_RX_DEPTH          (4U)   /* commands/maps arrive rarely     */
#define S1_STATUS_DEPTH      (2U)   /* S1 only needs the newest        */

#define S1_FAKE_PERIOD_MS    (500)    /* IF-13 period                  */
#define S1_FAKE_PHASE_SENDS  (20U)    /* 10 s READY, then 10 s state 2 */

EXPORT ID g_s1_tid_telem = 0;
EXPORT ID mbf_telem_status = 0;
EXPORT ID mbf_mqtt_tx = 0;
EXPORT ID mbf_mqtt_rx = 0;

/* IF-13 layout (see subsystem1.h): shift and width of each field */
#define S1_ST_TIME_SH        (0U)
#define S1_ST_TIME_W         (16U)
#define S1_ST_STATE_SH       (16U)
#define S1_ST_STATE_W        (4U)
#define S1_ST_FAULT_SH       (20U)
#define S1_ST_FAULT_W        (4U)
#define S1_ST_LETTER_SH      (24U)
#define S1_ST_LETTER_W       (5U)
#define S1_ST_ACTION_SH      (29U)
#define S1_ST_ACTION_W       (2U)
#define S1_ST_OBST_SH        (31U)
#define S1_ST_OBST_W         (1U)
#define S1_ST_DIST_SH        (0U)     /* w1 */
#define S1_ST_DIST_W         (9U)
#define S1_ST_SIDE_SH        (9U)
#define S1_ST_SIDE_W         (2U)
#define S1_ST_WIDTH_SH       (11U)
#define S1_ST_WIDTH_W        (6U)
#define S1_ST_BARC_SH        (17U)
#define S1_ST_BARC_W         (4U)

/* Largest value a field of width w holds */
#define S1_FIELD_MAX(w)      ((1UL << (w)) - 1UL)

static ID s1_create_mbf(INT maxmsz, UW depth);
static UW s1_put(UW value, UW width, UW shift);
static UW s1_get(UW word, UW width, UW shift);
#if S1_FAKE_S5 && !APP_TEST
static void fake_s5_task(INT stacd, void *exinf);
#endif

#if S1_TX_SINK && !APP_TEST && !(defined(TM_MQTT) && (TM_MQTT == 1))
#define S1_USE_TX_SINK       (1)
static void tx_sink_task(INT stacd, void *exinf);
#else
#define S1_USE_TX_SINK       (0)
#endif

/* IF-18 map word.  volatile: S5's task may read it from the other core. */
static volatile UW s_nav_map = 0U;

/* The bit layouts in subsystem1.h and cmd_parse.h must agree */
typedef char s1_check_map_valid[(S1_NAV_MAP_VALID == CMD_MAP_VALID) ? 1 : -1];
typedef char s1_check_map_moves[((S1_MOVE_FORWARD == CMD_MOVE_FORWARD) &&
                                 (S1_MOVE_UTURN == CMD_MOVE_UTURN) &&
                                 (S1_MOVE_LEFT == CMD_MOVE_LEFT) &&
                                 (S1_MOVE_RIGHT == CMD_MOVE_RIGHT)) ? 1 : -1];

#if S1_IF05_COMPAT
EXPORT ID tid_led = 0;

/* Stub for S4's IF-05: sleeps forever, safe to suspend/resume.
   DELETE when S4 removes its IF-05 code. */
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

/* Create kernel objects: the IF-13 and IF-15 message buffers. */
EXPORT ER subsystem1_init(void)
{
    mbf_telem_status = s1_create_mbf((INT)sizeof(s1_status_msg_t),
                                     S1_STATUS_DEPTH);
    if (mbf_telem_status < E_OK)
    {
        return mbf_telem_status;
    }

    mbf_mqtt_tx = s1_create_mbf((INT)S1_TX_MSG_MAX, S1_TX_DEPTH);
    if (mbf_mqtt_tx < E_OK)
    {
        return mbf_mqtt_tx;
    }

    mbf_mqtt_rx = s1_create_mbf((INT)S1_RX_MSG_MAX, S1_RX_DEPTH);
    if (mbf_mqtt_rx < E_OK)
    {
        return mbf_mqtt_rx;
    }

#if defined(TM_MQTT) && (TM_MQTT == 1)
    /* Both buffers exist now: let mqtt_utk.c use them (IF-15) */
    mqtt_utk_set_buffers((int32_t)mbf_mqtt_tx, (int32_t)mbf_mqtt_rx);
#endif

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

#if S1_FAKE_S5 && !APP_TEST
    if (er == E_OK)
    {
        ID tid_fake = app_start_task(fake_s5_task, S1_PRI_TELEM, 0,
                                     "fake_s5");
        if (tid_fake < E_OK)
        {
            er = tid_fake;
        }
    }
#endif

#if S1_USE_TX_SINK
    if (er == E_OK)
    {
        ID tid_sink = app_start_task(tx_sink_task, S1_PRI_TELEM, 0,
                                     "tx_sink");
        if (tid_sink < E_OK)
        {
            er = tid_sink;
        }
    }
#endif

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

/* IF-18: read the map word (one 32-bit load). */
EXPORT UW s1_get_nav_map(void)
{
    return s_nav_map;
}

/* IF-18: movement for a letter, from one snapshot of the map word. */
EXPORT BOOL s1_nav_lookup(UB letter, UB *p_move)
{
    uint8_t move = 0U;
    BOOL    b_found = FALSE;

    if ((p_move != NULL) && cmd_map_lookup(s_nav_map, letter, &move))
    {
        *p_move = move;
        b_found = TRUE;
    }
    return b_found;
}

/* IF-18: store an accepted map (one 32-bit store). */
EXPORT void s1_set_nav_map(UW map_word)
{
    s_nav_map = map_word;
}

/*
 * IF-13: pack a status into its 8-byte message.
 */
EXPORT void s1_status_pack(const s1_status_t *p_st, s1_status_msg_t *p_msg)
{
    if ((p_st == NULL) || (p_msg == NULL))
    {
        return;
    }

    p_msg->w0 = s1_put(p_st->mission_time_ds, S1_ST_TIME_W, S1_ST_TIME_SH) |
                s1_put(p_st->mission_state, S1_ST_STATE_W, S1_ST_STATE_SH) |
                s1_put(p_st->fault_mask, S1_ST_FAULT_W, S1_ST_FAULT_SH) |
                s1_put(p_st->letter, S1_ST_LETTER_W, S1_ST_LETTER_SH) |
                s1_put(p_st->last_action, S1_ST_ACTION_W, S1_ST_ACTION_SH) |
                s1_put((p_st->b_obstacle != 0U) ? 1U : 0U,
                       S1_ST_OBST_W, S1_ST_OBST_SH);
    p_msg->w1 = s1_put(p_st->obst_dist_cm, S1_ST_DIST_W, S1_ST_DIST_SH) |
                s1_put(p_st->obst_side, S1_ST_SIDE_W, S1_ST_SIDE_SH) |
                s1_put(p_st->obst_width_cm, S1_ST_WIDTH_W, S1_ST_WIDTH_SH) |
                s1_put(p_st->barcodes, S1_ST_BARC_W, S1_ST_BARC_SH);
}

/*
 * IF-13: unpack an 8-byte message into a status.
 */
EXPORT void s1_status_unpack(const s1_status_msg_t *p_msg, s1_status_t *p_st)
{
    if ((p_st == NULL) || (p_msg == NULL))
    {
        return;
    }

    p_st->mission_time_ds = (uint16_t)s1_get(p_msg->w0, S1_ST_TIME_W,
                                             S1_ST_TIME_SH);
    p_st->mission_state   = (uint8_t)s1_get(p_msg->w0, S1_ST_STATE_W,
                                            S1_ST_STATE_SH);
    p_st->fault_mask      = (uint8_t)s1_get(p_msg->w0, S1_ST_FAULT_W,
                                            S1_ST_FAULT_SH);
    p_st->letter          = (uint8_t)s1_get(p_msg->w0, S1_ST_LETTER_W,
                                            S1_ST_LETTER_SH);
    p_st->last_action     = (uint8_t)s1_get(p_msg->w0, S1_ST_ACTION_W,
                                            S1_ST_ACTION_SH);
    p_st->b_obstacle      = (uint8_t)s1_get(p_msg->w0, S1_ST_OBST_W,
                                            S1_ST_OBST_SH);
    p_st->obst_dist_cm    = (uint16_t)s1_get(p_msg->w1, S1_ST_DIST_W,
                                             S1_ST_DIST_SH);
    p_st->obst_side       = (uint8_t)s1_get(p_msg->w1, S1_ST_SIDE_W,
                                            S1_ST_SIDE_SH);
    p_st->obst_width_cm   = (uint8_t)s1_get(p_msg->w1, S1_ST_WIDTH_W,
                                            S1_ST_WIDTH_SH);
    p_st->barcodes        = (uint8_t)s1_get(p_msg->w1, S1_ST_BARC_W,
                                            S1_ST_BARC_SH);
}

/*
 * IF-13 (S5 calls this): pack one status and send it with TMO_POL.
 */
EXPORT ER s1_status_send(const s1_status_t *p_st)
{
    s1_status_msg_t msg;

    if (p_st == NULL)
    {
        return E_PAR;
    }
    s1_status_pack(p_st, &msg);
    return tk_snd_mbf(mbf_telem_status, &msg, (INT)sizeof(msg), TMO_POL);
}

/*
 * Value cut to the field's largest value, then moved into place.
 */
static UW s1_put(UW value, UW width, UW shift)
{
    UW max = S1_FIELD_MAX(width);

    return ((value > max) ? max : value) << shift;
}

static UW s1_get(UW word, UW width, UW shift)
{
    return (word >> shift) & S1_FIELD_MAX(width);
}

/*
 * Create one message buffer holding 'depth' messages of up to 'maxmsz'
 * bytes.  Returns its ID, or a negative error code.
 */
static ID s1_create_mbf(INT maxmsz, UW depth)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = maxmsz;
    cmbf.bufsz  = (SZ)(depth * S1_MBF_SLOT(maxmsz));
    return tk_cre_mbf(&cmbf);
}

#if S1_FAKE_S5 && !APP_TEST
/*
 * TEMPORARY stand-in for S5's IF-13 sender (normal builds only).
 * Every 500 ms: mission time counts up, the letter steps A..Z, and the
 * state alternates READY / 2 every 10 s so STATE_CHANGE events appear.
 */
static void fake_s5_task(INT stacd, void *exinf)
{
    s1_status_t st = {0};
    UW          sends = 0U;

    (void)stacd;
    (void)exinf;

    st.obst_dist_cm  = (uint16_t)TELEM_OBST_DIST_NONE;
    st.obst_width_cm = (uint8_t)TELEM_OBST_WIDTH_NONE;

    for (;;)
    {
        st.mission_time_ds = (uint16_t)(sends * 5U);
        st.mission_state   = (((sends / S1_FAKE_PHASE_SENDS) % 2U) == 0U) ?
                             (uint8_t)S1_MSTATE_READY : 2U;
        st.letter          = (uint8_t)(sends % 26U);
        st.barcodes        = (uint8_t)((sends / 10U) % 16U);

        (void)s1_status_send(&st);
        sends++;
        (void)tk_dly_tsk(S1_FAKE_PERIOD_MS);
    }
}
#endif

#if S1_USE_TX_SINK
/*
 * Debug stand-in for mqtt_utk.c (builds without MQTT=1): take each
 * message from mbf_mqtt_tx and print it, e.g.
 * "s1: pub telem t=12345 R 41 D2 04 ...".
 * R = replayed from the outage buffer.  Hex built by hand: tm_printf may
 * not support %02X.
 */
static void tx_sink_task(INT stacd, void *exinf)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t           msg[S1_TX_MSG_MAX];
    char              line[(S1_TX_MSG_MAX * 3U) + 1U];
    const char       *p_name;
    SYSTIM            now;
    INT               len;
    INT               i;
    uint8_t           pos;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        len = tk_rcv_mbf(mbf_mqtt_tx, msg, TMO_FEVR);
        if (len < 2)
        {
            continue;
        }

        pos = 0U;
        for (i = 1; i < len; i++)
        {
            line[pos++] = ' ';
            line[pos++] = hex[(msg[i] >> 4) & 0x0FU];
            line[pos++] = hex[msg[i] & 0x0FU];
        }
        line[pos] = '\0';

        p_name = (msg[0] == S1_TOPIC_TELEM) ? "telem" :
                 (msg[0] == S1_TOPIC_HB)    ? "hb   " : "evt  ";

        (void)tk_get_tim(&now);
        tm_printf((UB *)"s1: pub %s t=%u %s%s\n", p_name, now.lo,
                  telem_is_replayed(&msg[1]) ? "R" : "-", line);
    }
}
#endif