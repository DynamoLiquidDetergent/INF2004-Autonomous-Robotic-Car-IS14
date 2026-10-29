/*
 * telemetry.c
 * Subsystem 1: telemetry task.  The only S1 code that calls the kernel
 * on a schedule.
 *
 * Every 100 ms tick (drift-free: deadlines come from tk_get_tim()):
 *   1. read the link state (WiFi joined with an IP, and MQTT
 *      connected in MQTT=1 builds)
 *   2. take the newest IF-13 status from S5 (TMO_POL); a new mission
 *      state sends a STATE_CHANGE event
 *   3. handle commands and maps waiting in mbf_mqtt_rx (TMO_POL),
 *      answering each with a CMD_ACK or MAP_ACK event
 *   4. every 5th tick (500 ms): pack a telemetry frame.  Link up: send it.
 *      Link down, or TX full: keep it in the outage buffer (20 frames,
 *      drop-oldest)
 *   5. every 10th tick (1 s): send a heartbeat (link up only, never kept)
 *   6. link up and frames waiting: replay 1 frame, marked "replayed"
 * Everything goes to mbf_mqtt_tx with TMO_POL.  A heartbeat or event that
 * does not fit is dropped and counted.  The task never waits on anything
 * except its own tick (NFR1).
 */

#include <stdint.h>
#include "subsystem1.h"
#include "telemetry.h"
#include "s1_topics.h"
#include "cmd_parse.h"
#include "telem_pack.h"
#include "outage_buf.h"

#if defined(TM_NET_LWIP) && (TM_NET_LWIP == 1)
#include "cyw43_utk.h"
#endif

#if defined(TM_MQTT) && (TM_MQTT == 1)
#include "mqtt_utk.h"
#endif

/* Event data layouts (telem_layout.h lists the fields) */
#define S1_EVT_STATE_MASK       (0x0FU) /* STATE_CHANGE: 4-bit states    */
#define S1_EVT_OLD_STATE_SHIFT  (4U)    /* new bits 0-3, old bits 4-7    */
#define S1_EVT_CMD_OK           (0x04U) /* CMD_ACK: cmd bits 0-1, ok b2  */
#define S1_EVT_MAP_OK           (0x01U) /* MAP_ACK: ok bit 0             */
#define S1_EVT_MAP_REASON_SHIFT (1U)    /*          reason bits 1-2      */

/* Kernel message-buffer cost of one message: 4-byte header + data
   rounded up to 4 bytes (kernel/tkernel/messagebuf.h). */
#define S1_MBF_HEADER_SZ        (4U)
#define S1_MBF_ROUND_MASK       (3U)

#define S1_COUNT_MAX            (0xFFFFU)

/* Telemetry task state: one copy, used only by this task */
typedef struct
{
    s1_status_msg_t status;     /* newest IF-13 status                 */
    cmd_parser_t    parser;     /* commands + maps, with reject count  */
    outage_buf_t    outage;     /* telemetry kept while the link is down */
    uint32_t        tick;
    uint16_t        tx_drops;   /* hb/events lost; debugger only       */
    uint8_t         hb_seq;
    bool            b_link_up;
} s1_telem_ctx_t;

static s1_telem_ctx_t s_ctx;

#if APP_TEST
static volatile INT s_force_link = TELEM_LINK_AUTO;
#endif

static bool s1_read_link(void);
static void s1_update_link(s1_telem_ctx_t *p_ctx);
static void s1_take_status(s1_telem_ctx_t *p_ctx);
static void s1_handle_rx(s1_telem_ctx_t *p_ctx);
static void s1_send_telem(s1_telem_ctx_t *p_ctx);
static void s1_send_hb(s1_telem_ctx_t *p_ctx);
static void s1_replay(s1_telem_ctx_t *p_ctx);
static void s1_send_event(s1_telem_ctx_t *p_ctx, uint8_t type, uint16_t data);
static bool s1_tx_put(uint8_t topic, const uint8_t *p_frame, uint8_t len);
static bool s1_tx_has_room(uint8_t len);
static void s1_count_drop(s1_telem_ctx_t *p_ctx);

EXPORT void telemetry_task(INT stacd, void *exinf)
{
    SYSTIM   now;
    uint32_t next_ms;
    int32_t  wait_ms;

    (void)stacd;
    (void)exinf;

    /* Nothing known yet: every field starts at its "no data" value */
    s_ctx.status.mission_state = (uint8_t)S1_MSTATE_NONE;
    s_ctx.status.letter        = (uint8_t)TELEM_LETTER_NONE;
    s_ctx.status.obst_dist_cm  = (uint16_t)TELEM_OBST_DIST_NONE;
    s_ctx.status.obst_width_cm = (uint8_t)TELEM_OBST_WIDTH_NONE;
    cmd_parse_init(&s_ctx.parser);
    outage_buf_init(&s_ctx.outage);
    s_ctx.b_link_up = s1_read_link();

    (void)tk_get_tim(&now);
    next_ms = now.lo + TELEM_TICK_MS;

    for (;;)
    {
        (void)tk_get_tim(&now);
        wait_ms = (int32_t)(next_ms - now.lo);   /* wrap-safe */

        if (wait_ms > 0)
        {
            (void)tk_dly_tsk((RELTIM)wait_ms);
        }
        else if (wait_ms < -(int32_t)TELEM_TICK_MS)
        {
            next_ms = now.lo;                    /* far behind: resync */
        }
        else
        {
            /* slightly late: run now, keep the schedule */
        }

        next_ms += TELEM_TICK_MS;
        s_ctx.tick++;

        s1_update_link(&s_ctx);
        s1_take_status(&s_ctx);
        s1_handle_rx(&s_ctx);

        if ((s_ctx.tick % TELEM_FRAME_TICKS) == 0U)
        {
            s1_send_telem(&s_ctx);
        }

        if ((s_ctx.tick % TELEM_HB_TICKS) == 0U)
        {
            s1_send_hb(&s_ctx);
        }

        s1_replay(&s_ctx);
    }
}

#if APP_TEST
/*
 * Tests only: make the task see the link as down or up, or go back to
 * the real link (TELEM_LINK_AUTO).  One aligned store, read every tick.
 */
EXPORT void telem_test_force_link(INT mode)
{
    s_force_link = mode;
}
#endif

/*
 * True if telemetry can go out.  With the network stack built in
 * (NET=lwip) that means WiFi joined with an IP, and with MQTT=1 also
 * connected to the broker.  Without the network stack there is no link
 * to lose, so frames always go straight to mbf_mqtt_tx.
 */
static bool s1_read_link(void)
{
    bool b_up = true;

#if defined(TM_NET_LWIP) && (TM_NET_LWIP == 1)
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    b_up = (st.link_status == TELEM_WIFI_LINK_UP);
#endif

#if defined(TM_MQTT) && (TM_MQTT == 1)
    b_up = b_up && (mqtt_utk_connected() != 0);
#endif

#if APP_TEST
    if (s_force_link == TELEM_LINK_DOWN)
    {
        b_up = false;
    }
    else if (s_force_link == TELEM_LINK_UP)
    {
        b_up = true;
    }
    else
    {
        /* TELEM_LINK_AUTO: keep the real state */
    }
#endif

    return b_up;
}

/*
 * Note link changes on the console.  WiFi loss is status, not a fault:
 * it only decides where telemetry goes.
 */
static void s1_update_link(s1_telem_ctx_t *p_ctx)
{
    bool b_up = s1_read_link();

    if (b_up != p_ctx->b_link_up)
    {
        p_ctx->b_link_up = b_up;
        if (b_up)
        {
            tm_printf((UB *)"s1: link up, %d frame(s) to replay\n",
                      (INT)outage_buf_count(&p_ctx->outage));
        }
        else
        {
            tm_printf((UB *)"s1: link down, buffering telemetry\n");
        }
    }
}

/*
 * IF-13: keep only the newest status S5 sent since the last tick.
 */
static void s1_take_status(s1_telem_ctx_t *p_ctx)
{
    s1_status_msg_t msg;
    uint8_t         old_state = p_ctx->status.mission_state;
    bool            b_got     = false;

    while (tk_rcv_mbf(mbf_telem_status, &msg, TMO_POL) ==
           (INT)sizeof(s1_status_msg_t))
    {
        p_ctx->status = msg;
        b_got = true;
    }

    if (b_got && (p_ctx->status.mission_state != old_state))
    {
        s1_send_event(p_ctx, TELEM_EVT_STATE_CHANGE,
                      (uint16_t)((p_ctx->status.mission_state &
                                  S1_EVT_STATE_MASK) |
                                 ((old_state & S1_EVT_STATE_MASK) <<
                                  S1_EVT_OLD_STATE_SHIFT)));
    }
}

/*
 * IF-15 RX: check each waiting command or map and acknowledge it.
 * At most TELEM_RX_PER_TICK per tick, so a flood cannot hold up telemetry.
 */
static void s1_handle_rx(s1_telem_ctx_t *p_ctx)
{
    uint8_t      msg[S1_RX_MSG_MAX];
    INT          len;
    uint8_t      n;
    cmd_code_t   cmd;
    cmd_result_t result;
    uint32_t     map_word;
    uint16_t     data;
    bool         b_ready;

    for (n = 0U; n < TELEM_RX_PER_TICK; n++)
    {
        len = tk_rcv_mbf(mbf_mqtt_rx, msg, TMO_POL);
        if (len < 1)
        {
            break;                               /* nothing waiting */
        }

        if (msg[0] == S1_TOPIC_CMD)
        {
            result = cmd_parse(&p_ctx->parser, &msg[1],
                               (uint32_t)(len - 1), &cmd);
            /* PAUSE/RESUME/ABORT go to S5 over IF-14 in a later phase */
            data = (uint16_t)cmd;
            if (result == CMD_OK)
            {
                data = (uint16_t)(data | S1_EVT_CMD_OK);
            }
            s1_send_event(p_ctx, TELEM_EVT_CMD_ACK, data);
        }
        else if (msg[0] == S1_TOPIC_MAP)
        {
            b_ready = (p_ctx->status.mission_state == S1_MSTATE_READY);
            result  = cmd_parse_map(&p_ctx->parser, &msg[1],
                                    (uint32_t)(len - 1), b_ready, &map_word);
            if (result == CMD_OK)
            {
                s1_set_nav_map((UW)map_word);
                data = (uint16_t)S1_EVT_MAP_OK;
            }
            else
            {
                data = (uint16_t)(cmd_map_reason(result) <<
                                  S1_EVT_MAP_REASON_SHIFT);
            }
            s1_send_event(p_ctx, TELEM_EVT_MAP_ACK, data);
        }
        else
        {
            /* unknown topic ID: ignore (mqtt_utk.c only subscribes to two) */
        }
    }
}

/*
 * Pack one telemetry frame.  Link up: send it now.  Link down, or TX
 * full: keep it for replay.  Fields with no source yet go out as their
 * "no data" value.
 */
static void s1_send_telem(s1_telem_ctx_t *p_ctx)
{
    telem_status_t t;
    uint8_t        frame[TELEM_FRAME_LEN];
    uint8_t        len;

    t.time_ds       = p_ctx->status.mission_time_ds;
    t.mission_state = p_ctx->status.mission_state;
    t.fault_mask    = p_ctx->status.fault_mask;
    t.speed_left    = (int16_t)TELEM_SPEED_NONE;     /* S2: IF-16 later */
    t.speed_right   = (int16_t)TELEM_SPEED_NONE;
    t.motion_state  = 0U;
    t.dist_mm       = 0U;                            /* S2 odometry later */
    t.line_zone     = (uint8_t)TELEM_ZONE_NONE;      /* S3 later */
    t.b_junction    = false;
    t.b_line_lost   = false;
    t.letter        = p_ctx->status.letter;
    t.last_action   = p_ctx->status.last_action;
    t.b_map_ok      = ((s1_get_nav_map() & S1_NAV_MAP_VALID) != 0UL);
    t.barcodes      = p_ctx->status.barcodes;
    t.hump_mm       = (uint8_t)TELEM_HUMP_NONE;      /* S4 later */
    t.b_obstacle    = (p_ctx->status.b_obstacle != 0U);
    t.obst_dist_cm  = p_ctx->status.obst_dist_cm;
    t.obst_side     = p_ctx->status.obst_side;
    t.obst_width_cm = p_ctx->status.obst_width_cm;

    len = telem_pack_status(&t, frame, (uint8_t)sizeof(frame));
    if (len == 0U)
    {
        return;
    }

    /* Keep the order: while older frames wait, new ones queue behind them */
    if (!p_ctx->b_link_up || (outage_buf_count(&p_ctx->outage) > 0U) ||
        !s1_tx_put(S1_TOPIC_TELEM, frame, len))
    {
        (void)outage_buf_push(&p_ctx->outage, frame, len);
    }
}

/*
 * Pack and send one heartbeat.  Only while the link is up, and never
 * kept: a missing heartbeat is how the PC sees the link was down.
 */
static void s1_send_hb(s1_telem_ctx_t *p_ctx)
{
    telem_hb_t hb;
    uint8_t    frame[TELEM_HB_FRAME_LEN];
    uint8_t    len;
    uint16_t   rejected = cmd_parse_rejected(&p_ctx->parser);

    if (!p_ctx->b_link_up)
    {
        return;
    }

    hb.seq           = p_ctx->hb_seq;
    hb.mission_state = p_ctx->status.mission_state;
    hb.outage_count  = outage_buf_count(&p_ctx->outage);
#if defined(TM_NET_LWIP) && (TM_NET_LWIP == 1)
    hb.b_wifi_up     = true;                  /* hb only sent with link up */
#else
    hb.b_wifi_up     = false;                 /* no network stack built */
#endif
#if defined(TM_MQTT) && (TM_MQTT == 1)
    hb.b_mqtt_up     = (mqtt_utk_connected() != 0);
#else
    hb.b_mqtt_up     = false;                 /* no MQTT client built   */
#endif
    hb.b_map_ok      = ((s1_get_nav_map() & S1_NAV_MAP_VALID) != 0UL);
    hb.rejected      = (rejected > 255U) ? 255U : (uint8_t)rejected;

    len = telem_pack_hb(&hb, frame, (uint8_t)sizeof(frame));
    if ((len == 0U) || !s1_tx_put(S1_TOPIC_HB, frame, len))
    {
        s1_count_drop(p_ctx);
    }
    p_ctx->hb_seq++;
}

/*
 * Replay at most TELEM_REPLAY_PER_TICK kept frames, oldest first, each
 * marked "replayed".  A frame leaves the outage buffer only once TX has
 * room for it, so nothing is lost on the way.
 */
static void s1_replay(s1_telem_ctx_t *p_ctx)
{
    uint8_t frame[OUTAGE_BUF_FRAME_MAX];
    uint8_t len = 0U;
    uint8_t n;

    for (n = 0U; n < TELEM_REPLAY_PER_TICK; n++)
    {
        if (!p_ctx->b_link_up || (outage_buf_count(&p_ctx->outage) == 0U) ||
            !s1_tx_has_room(TELEM_FRAME_LEN))
        {
            break;
        }

        if (outage_buf_pop(&p_ctx->outage, frame, (uint8_t)sizeof(frame),
                           &len))
        {
            telem_mark_replayed(frame);
            (void)s1_tx_put(S1_TOPIC_TELEM, frame, len);
        }
    }
}

/*
 * Send an event while the link is up; otherwise it is dropped and
 * counted (telemetry already carries the state it reports).
 */
static void s1_send_event(s1_telem_ctx_t *p_ctx, uint8_t type, uint16_t data)
{
    uint8_t frame[TELEM_EVT_FRAME_LEN];
    uint8_t len;

    len = telem_pack_event(type, data, frame, (uint8_t)sizeof(frame));
    if ((len == 0U) || !p_ctx->b_link_up ||
        !s1_tx_put(S1_TOPIC_EVT, frame, len))
    {
        s1_count_drop(p_ctx);
    }
}

/*
 * Put [topic ID][frame] into mbf_mqtt_tx without waiting.
 * False if it does not fit.
 */
static bool s1_tx_put(uint8_t topic, const uint8_t *p_frame, uint8_t len)
{
    uint8_t msg[S1_TX_MSG_MAX];
    uint8_t i;

    if ((len == 0U) || (len > (S1_TX_MSG_MAX - 1U)))
    {
        return false;
    }

    msg[0] = topic;
    for (i = 0U; i < len; i++)
    {
        msg[i + 1U] = p_frame[i];
    }

    return (tk_snd_mbf(mbf_mqtt_tx, msg, (INT)(len + 1U), TMO_POL) == E_OK);
}

/*
 * True if mbf_mqtt_tx has room for a frame of 'len' bytes (+ topic ID).
 * Only this task writes to it, so the room cannot disappear before use.
 */
static bool s1_tx_has_room(uint8_t len)
{
    T_RMBF r;
    UW     need = S1_MBF_HEADER_SZ +
                  (((UW)len + 1U + S1_MBF_ROUND_MASK) & ~(UW)S1_MBF_ROUND_MASK);

    return (tk_ref_mbf(mbf_mqtt_tx, &r) == E_OK) && ((UW)r.frbufsz >= need);
}

static void s1_count_drop(s1_telem_ctx_t *p_ctx)
{
    if (p_ctx->tx_drops < S1_COUNT_MAX)
    {
        p_ctx->tx_drops++;
    }
}