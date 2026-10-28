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

static ID s1_create_mbf(INT maxmsz, UW depth);
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
    s1_status_msg_t msg = {0};
    UW              sends = 0U;

    (void)stacd;
    (void)exinf;

    msg.obst_dist_cm  = (uint16_t)TELEM_OBST_DIST_NONE;
    msg.obst_width_cm = (uint8_t)TELEM_OBST_WIDTH_NONE;

    for (;;)
    {
        msg.mission_time_ds = (uint16_t)(sends * 5U);
        msg.mission_state   = (((sends / S1_FAKE_PHASE_SENDS) % 2U) == 0U) ?
                              (uint8_t)S1_MSTATE_READY : 2U;
        msg.letter          = (uint8_t)(sends % 26U);
        msg.barcodes        = (uint8_t)((sends / 10U) % 16U);

        (void)tk_snd_mbf(mbf_telem_status, &msg, (INT)sizeof(msg), TMO_POL);
        sends++;
        (void)tk_dly_tsk(S1_FAKE_PERIOD_MS);
    }
}
#endif

#if S1_USE_TX_SINK
/*
 * TEMPORARY stand-in for mqtt_utk.c: take each message from mbf_mqtt_tx
 * and print it, e.g. "s1: pub telem t=12345 R 41 D2 04 ...".
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