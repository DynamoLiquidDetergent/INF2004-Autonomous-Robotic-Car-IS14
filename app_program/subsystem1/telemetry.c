/*
 * telemetry.c
 * Subsystem 1: telemetry task.  The only S1 code that calls the kernel
 * on a schedule.
 *
 * Every 100 ms tick (drift-free: deadlines come from tk_get_tim()):
 *   1. take the newest IF-13 status from S5 (TMO_POL); a new mission
 *      state sends a STATE_CHANGE event
 *   2. handle commands and maps waiting in mbf_mqtt_rx (TMO_POL),
 *      answering each with a CMD_ACK or MAP_ACK event
 *   3. every 5th tick (500 ms): pack and send a telemetry frame
 *   4. every 10th tick (1 s):   pack and send a heartbeat
 * Frames go to mbf_mqtt_tx with TMO_POL; if it is full the frame is
 * dropped & counted (the outage buffer takes over in the next phase).
 * The task never waits on anything except its own tick.
 */

#include <stdint.h>
#include "subsystem1.h"
#include "telemetry.h"
#include "s1_topics.h"
#include "cmd_parse.h"
#include "telem_pack.h"

/* Event data layouts (telem_layout.h lists the fields) */
#define S1_EVT_CMD_OK_SHIFT     (2U)   /* CMD_ACK: command bits 0-1, ok bit 2 */
#define S1_EVT_MAP_REASON_SHIFT (1U)   /* MAP_ACK: ok bit 0, reason bits 1-2  */
#define S1_EVT_OLD_STATE_SHIFT  (4U)   /* STATE_CHANGE: new 0-3, old 4-7      */

/* Telemetry task state: one copy, used only by this task */
typedef struct
{
    s1_status_msg_t status;     /* newest IF-13 status                 */
    cmd_parser_t    parser;     /* commands + maps, with reject count  */
    uint32_t        tick;
    uint16_t        tx_drops;   /* frames lost because TX was full     */
    uint8_t         hb_seq;
} s1_telem_ctx_t;

static s1_telem_ctx_t s_ctx;

static void s1_take_status(s1_telem_ctx_t *p_ctx);
static void s1_handle_rx(s1_telem_ctx_t *p_ctx);
static void s1_send_telem(s1_telem_ctx_t *p_ctx);
static void s1_send_hb(s1_telem_ctx_t *p_ctx);
static void s1_send_event(s1_telem_ctx_t *p_ctx, uint8_t type, uint16_t data);
static void s1_send_frame(s1_telem_ctx_t *p_ctx, uint8_t topic,
                          const uint8_t *p_frame, uint8_t len);
static void s1_print_hex(uint8_t topic, const uint8_t *p_frame, uint8_t len);

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
                      (uint16_t)((p_ctx->status.mission_state & 0x0FU) |
                                 ((old_state & 0x0FU) << S1_EVT_OLD_STATE_SHIFT)));
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

    for (n = 0U; n < TELEM_RX_PER_TICK; n++)
    {
        len = tk_rcv_mbf(mbf_mqtt_rx, msg, TMO_POL);
        if (len < 1)
        {
            break;                               /* nothing waiting */
        }

        if (msg[0] == S1_TOPIC_CMD)
        {
            result = cmd_parse(&p_ctx->parser, &msg[1], (uint32_t)(len - 1),
                               &cmd);
            /* PAUSE/RESUME/ABORT go to S5 over IF-14 in a later phase */
            s1_send_event(p_ctx, TELEM_EVT_CMD_ACK,
                          (uint16_t)((uint16_t)cmd |
                                     ((result == CMD_OK) ? (1U << S1_EVT_CMD_OK_SHIFT) : 0U)));
        }
        else if (msg[0] == S1_TOPIC_MAP)
        {
            result = cmd_parse_map(&p_ctx->parser, &msg[1], (uint32_t)(len - 1),
                                   (p_ctx->status.mission_state == S1_MSTATE_READY),
                                   &map_word);
            if (result == CMD_OK)
            {
                s1_set_nav_map((UW)map_word);
                s1_send_event(p_ctx, TELEM_EVT_MAP_ACK, 1U);
            }
            else
            {
                s1_send_event(p_ctx, TELEM_EVT_MAP_ACK,
                              (uint16_t)(cmd_map_reason(result) << S1_EVT_MAP_REASON_SHIFT));
            }
        }
        else
        {
            /* unknown topic ID: ignore (mqtt_utk.c only subscribes to two) */
        }
    }
}

/*
 * Pack and send one telemetry frame.  Fields with no source yet go out
 * as their "no data" value.
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
    s1_send_frame(p_ctx, S1_TOPIC_TELEM, frame, len);
}

/*
 * Pack and send one heartbeat.
 */
static void s1_send_hb(s1_telem_ctx_t *p_ctx)
{
    telem_hb_t hb;
    uint8_t    frame[TELEM_HB_FRAME_LEN];
    uint8_t    len;
    uint16_t   rejected = cmd_parse_rejected(&p_ctx->parser);

    hb.seq           = p_ctx->hb_seq;
    hb.mission_state = p_ctx->status.mission_state;
    hb.outage_count  = 0U;                    /* outage buffer: next phase */
    hb.b_wifi_up     = false;                 /* link state: next phase    */
    hb.b_mqtt_up     = false;
    hb.b_map_ok      = ((s1_get_nav_map() & S1_NAV_MAP_VALID) != 0UL);
    hb.rejected      = (rejected > 255U) ? 255U : (uint8_t)rejected;

    len = telem_pack_hb(&hb, frame, (uint8_t)sizeof(frame));
    s1_send_frame(p_ctx, S1_TOPIC_HB, frame, len);
    p_ctx->hb_seq++;
}

static void s1_send_event(s1_telem_ctx_t *p_ctx, uint8_t type, uint16_t data)
{
    uint8_t frame[TELEM_EVT_FRAME_LEN];
    uint8_t len;

    len = telem_pack_event(type, data, frame, (uint8_t)sizeof(frame));
    s1_send_frame(p_ctx, S1_TOPIC_EVT, frame, len);
}

/*
 * Put [topic ID][frame] into mbf_mqtt_tx without waiting.
 */
static void s1_send_frame(s1_telem_ctx_t *p_ctx, uint8_t topic,
                          const uint8_t *p_frame, uint8_t len)
{
    uint8_t msg[S1_TX_MSG_MAX];
    uint8_t i;

    if ((len == 0U) || (len > (S1_TX_MSG_MAX - 1U)))
    {
        return;                                  /* nothing valid to send */
    }

    msg[0] = topic;
    for (i = 0U; i < len; i++)
    {
        msg[i + 1U] = p_frame[i];
    }

    if (tk_snd_mbf(mbf_mqtt_tx, msg, (INT)(len + 1U), TMO_POL) != E_OK)
    {
        if (p_ctx->tx_drops < 0xFFFFU)
        {
            p_ctx->tx_drops++;
        }
    }

#if TELEM_DEBUG_HEX
    s1_print_hex(topic, p_frame, len);
#endif
}

#if TELEM_DEBUG_HEX
/*
 * Print one frame, e.g. "s1: tx telem t=12345 01 D2 04 ...".
 * Built by hand: tm_printf may not support %02X.
 */
static void s1_print_hex(uint8_t topic, const uint8_t *p_frame, uint8_t len)
{
    static const char  hex[] = "0123456789ABCDEF";
    char               line[(S1_TX_MSG_MAX * 3U) + 1U];
    const char        *p_name;
    SYSTIM             now;
    uint8_t            i;
    uint8_t            pos = 0U;

    for (i = 0U; (i < len) && (i < (S1_TX_MSG_MAX - 1U)); i++)
    {
        line[pos++] = ' ';
        line[pos++] = hex[(p_frame[i] >> 4) & 0x0FU];
        line[pos++] = hex[p_frame[i] & 0x0FU];
    }
    line[pos] = '\0';

    p_name = (topic == S1_TOPIC_TELEM) ? "telem" :
             (topic == S1_TOPIC_HB)    ? "hb   " : "evt  ";

    (void)tk_get_tim(&now);
    tm_printf((UB *)"s1: tx %s t=%u%s\n", p_name, now.lo, line);
}
#else
static void s1_print_hex(uint8_t topic, const uint8_t *p_frame, uint8_t len)
{
    (void)topic;
    (void)p_frame;
    (void)len;
}
#endif