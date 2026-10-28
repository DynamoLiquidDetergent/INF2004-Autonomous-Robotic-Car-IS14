/*
 * subsystem1_test.c
 * On-target tests for subsystem 1.  Built only with: make TEST=1
 * Test logs (console output) are saved to testing/subsystem1/logs/.
 *
 * TEST_INIT() declares the pass/fail counters as locals, so every
 * TEST_CHECK stays in subsystem1_test().  The helpers below only work
 * out a result and return it.
 *
 * Does not include <string.h>: it clashes with the kernel's size_t.
 */

#include "subsystem1.h"
#include "outage_buf.h"
#include "cmd_parse.h"
#include "telem_pack.h"
#include "s1_topics.h"
#include "telemetry.h"
#include "../test_report.h"

#if APP_TEST

#define S1_TEST_START_DELAY_MS  (5000)  /* time to open the serial port */
#define S1_TEST_FRAME_LEN       (2U)    /* bytes in each test frame     */

/* Test buffer: static so its ~680 bytes are not on the test task stack,
   and separate from anything the telemetry task uses. */
static outage_buf_t s_test_buf;

/* Test parser, separate from the one the telemetry task will use */
static cmd_parser_t s_test_parser;

/* What the outage tests saw of a replay (S1-U29, S1-U30) */
typedef struct
{
    uint8_t  n;          /* replayed outage frames seen               */
    uint16_t first;      /* mission time of the first one             */
    uint16_t last;       /* mission time of the last one              */
    UW       min_gap;    /* shortest time between two of them, ms     */
    UW       span;       /* first to last arrival, ms                 */
    bool     b_order;    /* all marked replayed, mission time rising  */
} s1_replay_result_t;

static INT  s1_count_waiting(void);
static bool s1_outage_empty(void);
static bool s1_outage_fill(void);
static bool s1_outage_overflow(void);
static bool s1_outage_bad_input(void);
static bool s1_push_n(uint8_t first, uint8_t n);
static bool s1_pop_expect(uint8_t first, uint8_t n);
static bool s1_cmd_valid(void);
static bool s1_cmd_bad_length(void);
static bool s1_cmd_bad_code(void);
static bool s1_cmd_expect(uint8_t code, cmd_result_t want,
                          cmd_code_t want_cmd);
static bool s1_pack_golden(void);
static bool s1_pack_limits(void);
static bool s1_pack_clamp(void);
static bool s1_pack_hb(void);
static bool s1_pack_event(void);
static bool s1_pack_header(void);
static bool s1_map_valid(void);
static bool s1_map_bad_length(void);
static bool s1_map_bad_letter(void);
static bool s1_map_duplicate_state(void);
static bool s1_map_store(void);
static bool s1_map_expect(const uint8_t *p_frame, bool b_ready,
                          cmd_result_t want);
static bool s1_task_objects(void);
static bool s1_task_timing(void);
static bool s1_task_status_in(void);
static bool s1_task_cmd_in(void);
static bool s1_task_map_in(void);
static void s1_flush_tx(void);
static bool s1_drain_backlog(void);
static bool s1_wait_tx(uint8_t topic, uint8_t evt_type, INT timeout_ms,
                       uint8_t *p_frame, uint8_t *p_len, uint16_t *p_data);
static bool s1_wait_event(uint8_t evt_type, uint16_t want_data);
static bool s1_send_status(uint8_t state, uint8_t letter);
static bool s1_send_status_t(uint8_t state, uint8_t letter,
                             uint16_t time_ds);
static bool s1_outage_replay(void);
static bool s1_outage_overflow_live(void);
static bool s1_hold_link_down(uint16_t first_time, uint8_t sends);
static bool s1_collect_replay(INT window_ms, uint16_t max_time,
                              s1_replay_result_t *p_res);
static void s1_print_replay(const char *p_id, const s1_replay_result_t *p_res);
static bool s1_send_rx(uint8_t topic, const uint8_t *p_payload, uint8_t len);
static UW   s1_now_ms(void);
static bool s1_bytes_equal(const uint8_t *p_a, const uint8_t *p_b,
                           uint8_t len);
static bool s1_status_equal(const telem_status_t *p_a,
                            const telem_status_t *p_b);
static void s1_status_fill(telem_status_t *p_s, bool b_max);

EXPORT void subsystem1_test(void)
{
    /* Give the tester time to open the USB serial port */
    (void)tk_dly_tsk(S1_TEST_START_DELAY_MS);

    TEST_INIT();

    /* Telemetry task */
    TEST_CHECK("S1-U01", g_s1_tid_telem > 0,
               "telemetry task created");
    TEST_CHECK("S1-U02", s1_count_waiting() >= 9,
               "telemetry task sleeps between ticks");

    /* Outage buffer (IF-15: 20 frames, drop-oldest) */
    TEST_CHECK("S1-U04", s1_outage_empty(),
               "empty outage buffer: pop fails, count 0");
    TEST_CHECK("S1-U05", s1_outage_fill(),
               "20 pushes: count 20, 0 dropped, popped 1..20 in order");
    TEST_CHECK("S1-U06", s1_outage_overflow(),
               "25 pushes: oldest 5 dropped, popped 6..25 in order");
    TEST_CHECK("S1-U07", s1_outage_bad_input(),
               "bad frames refused; too-small pop keeps the frame");

    /* Command parser (FR7, FR8: only PAUSE / RESUME / ABORT) */
    TEST_CHECK("S1-U08", s1_cmd_valid(),
               "PAUSE, RESUME, ABORT accepted and decoded");
    TEST_CHECK("S1-U09", s1_cmd_bad_length(),
               "NULL and wrong-length frames rejected");
    TEST_CHECK("S1-U10", s1_cmd_bad_code(),
               "codes other than 1..3 (incl. 0, 4, 255, ASCII) rejected");

    /* Binary packing (Change Requirement 1) */
    TEST_CHECK("S1-U11", s1_pack_golden(),
               "telemetry, heartbeat, event match independently computed bytes");
    TEST_CHECK("S1-U12", s1_pack_limits(),
               "telemetry 15 B; every field round-trips at min and max");
    TEST_CHECK("S1-U13", s1_pack_clamp(),
               "out-of-range values clamp; no-data values kept");
    TEST_CHECK("S1-U14", s1_pack_hb(),
               "heartbeat 4 B round-trip at min and max");
    TEST_CHECK("S1-U15", s1_pack_event(),
               "event 3 B round-trip; bad type/data refused");
    TEST_CHECK("S1-U16", s1_pack_header(),
               "header version/type/replayed; NULL and small buffer refused");

    /* Mapping message (prof requirement: letter-to-movement over MQTT) */
    TEST_CHECK("S1-U18", s1_map_valid(),
               "valid map accepted; K/D/A/Z map to F/U/L/R, B not in map");
    TEST_CHECK("S1-U19", s1_map_bad_length(),
               "NULL and wrong-length map frames rejected");
    TEST_CHECK("S1-U20", s1_map_bad_letter(),
               "letter past Z or spare bits set rejected");
    TEST_CHECK("S1-U21", s1_map_duplicate_state(),
               "duplicate letters or not READY rejected; ack reasons right");
    TEST_CHECK("S1-U22", s1_map_store(),
               "IF-18: no map at boot; stored word read back and looked up");

    /* Task wiring: message buffers, timing, IF-13 in, IF-15 RX/TX */
    TEST_CHECK("S1-U23", s1_task_objects(),
               "IF-13 and IF-15 message buffers created, sizes right");
    TEST_CHECK("S1-U24", s1_task_timing(),
               "telemetry every 500 ms, heartbeat every 1 s (+/-10 ms), all valid");
    TEST_CHECK("S1-U25", s1_task_status_in(),
               "IF-13 status reaches telemetry; STATE_CHANGE event sent");
    TEST_CHECK("S1-U26", s1_task_cmd_in(),
               "commands via RX buffer: valid ACKed, junk rejected");
    TEST_CHECK("S1-U27", s1_task_map_in(),
               "map via RX buffer: stored when READY, refused otherwise");

    /* Link loss: outage buffering and replay (IF-15, NFR4) */
    TEST_CHECK("S1-U29", s1_outage_replay(),
               "link down 3 s: nothing sent, frames kept; link up: replayed");
    TEST_CHECK("S1-U30", s1_outage_overflow_live(),
               "link down 12 s: 20 newest kept, oldest dropped, 1 per tick");

    TEST_SUMMARY("subsystem1");
}

/*
 * S1-U02 helper: sample the telemetry task 10 times over ~1 s and count
 * how often it is waiting (asleep) rather than running.
 */
static INT s1_count_waiting(void)
{
    T_RTSK rtsk;
    INT    i;
    INT    waiting = 0;

    for (i = 0; i < 10; i++)
    {
        (void)tk_dly_tsk(97);   /* not a multiple of the tick */
        if ((tk_ref_tsk(g_s1_tid_telem, &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0))
        {
            waiting++;
        }
    }
    return waiting;
}

/*
 * S1-U04: an empty buffer has nothing to pop.
 */
static bool s1_outage_empty(void)
{
    uint8_t out[OUTAGE_BUF_FRAME_MAX];
    uint8_t len = 0U;

    outage_buf_init(&s_test_buf);
    return (outage_buf_count(&s_test_buf) == 0U) &&
           !outage_buf_pop(&s_test_buf, out, (uint8_t)sizeof(out), &len);
}

/*
 * S1-U05: 20 pushes fit exactly, nothing dropped, FIFO order.
 */
static bool s1_outage_fill(void)
{
    outage_buf_init(&s_test_buf);
    return s1_push_n(1U, 20U) &&
           (outage_buf_count(&s_test_buf) == 20U) &&
           (outage_buf_dropped(&s_test_buf) == 0U) &&
           s1_pop_expect(1U, 20U) &&
           (outage_buf_count(&s_test_buf) == 0U);
}

/*
 * S1-U06: 25 pushes keep the newest 20 (6..25) and count 5 dropped.
 */
static bool s1_outage_overflow(void)
{
    uint8_t out[OUTAGE_BUF_FRAME_MAX];
    uint8_t len = 0U;

    outage_buf_init(&s_test_buf);
    return s1_push_n(1U, 25U) &&
           (outage_buf_count(&s_test_buf) == 20U) &&
           (outage_buf_dropped(&s_test_buf) == 5U) &&
           s1_pop_expect(6U, 20U) &&
           !outage_buf_pop(&s_test_buf, out, (uint8_t)sizeof(out), &len);
}

/*
 * S1-U07: bad input is refused and changes nothing; a pop into a buffer
 * that is too small fails and leaves the frame in place.
 */
static bool s1_outage_bad_input(void)
{
    uint8_t out[OUTAGE_BUF_FRAME_MAX];
    uint8_t small[1];
    uint8_t big[OUTAGE_BUF_FRAME_MAX + 1U];
    uint8_t len = 0U;
    uint8_t i;

    for (i = 0U; i < (uint8_t)sizeof(big); i++)
    {
        big[i] = 0xAAU;
    }

    outage_buf_init(&s_test_buf);
    return !outage_buf_push(&s_test_buf, NULL, 1U) &&
           !outage_buf_push(&s_test_buf, big, 0U) &&
           !outage_buf_push(&s_test_buf, big, (uint8_t)sizeof(big)) &&
           (outage_buf_count(&s_test_buf) == 0U) &&
           outage_buf_push(&s_test_buf, big, (uint8_t)OUTAGE_BUF_FRAME_MAX) &&
           !outage_buf_pop(&s_test_buf, small, (uint8_t)sizeof(small), &len) &&
           (outage_buf_count(&s_test_buf) == 1U) &&
           outage_buf_pop(&s_test_buf, out, (uint8_t)sizeof(out), &len) &&
           (len == OUTAGE_BUF_FRAME_MAX);
}

/*
 * Push frames first, first+1, ... (n frames).  Frame k holds { k, ~k }.
 * False if any push fails.
 */
static bool s1_push_n(uint8_t first, uint8_t n)
{
    uint8_t frame[S1_TEST_FRAME_LEN];
    uint8_t i;

    for (i = 0U; i < n; i++)
    {
        frame[0] = (uint8_t)(first + i);
        frame[1] = (uint8_t)~frame[0];
        if (!outage_buf_push(&s_test_buf, frame, S1_TEST_FRAME_LEN))
        {
            return false;
        }
    }
    return true;
}

/*
 * Pop n frames and check they are first, first+1, ... with the right
 * length and content.
 */
static bool s1_pop_expect(uint8_t first, uint8_t n)
{
    uint8_t out[OUTAGE_BUF_FRAME_MAX];
    uint8_t len = 0U;
    uint8_t expect;
    uint8_t i;

    for (i = 0U; i < n; i++)
    {
        expect = (uint8_t)(first + i);
        if (!outage_buf_pop(&s_test_buf, out, (uint8_t)sizeof(out), &len) ||
            (len != S1_TEST_FRAME_LEN) ||
            (out[0] != expect) ||
            (out[1] != (uint8_t)~expect))
        {
            return false;
        }
    }
    return true;
}

/*
 * Parse the 1-byte frame { code } with s_test_parser and check the
 * result and the command that comes out.
 */
static bool s1_cmd_expect(uint8_t code, cmd_result_t want,
                          cmd_code_t want_cmd)
{
    uint8_t    frame[CMD_FRAME_LEN];
    cmd_code_t cmd = CMD_PAUSE;   /* not NONE, so a reject must clear it */

    frame[0] = code;
    return (cmd_parse(&s_test_parser, frame, CMD_FRAME_LEN, &cmd) == want) &&
           (cmd == want_cmd);
}

/*
 * S1-U08: each allowed command is accepted and decoded; nothing rejected.
 */
static bool s1_cmd_valid(void)
{
    cmd_parse_init(&s_test_parser);
    return s1_cmd_expect(1U, CMD_OK, CMD_PAUSE) &&
           s1_cmd_expect(2U, CMD_OK, CMD_RESUME) &&
           s1_cmd_expect(3U, CMD_OK, CMD_ABORT) &&
           (cmd_parse_rejected(&s_test_parser) == 0U);
}

/*
 * S1-U09: NULL, empty and too-long frames are rejected and counted,
 * and a valid frame afterwards is still accepted.
 */
static bool s1_cmd_bad_length(void)
{
    uint8_t    frame[2] = { 1U, 1U };
    cmd_code_t cmd = CMD_PAUSE;
    bool       b_ok;

    cmd_parse_init(&s_test_parser);
    b_ok = (cmd_parse(&s_test_parser, NULL, CMD_FRAME_LEN, &cmd) == CMD_ERR_NULL) &&
           (cmd == CMD_NONE) &&
           (cmd_parse(&s_test_parser, frame, 0U, &cmd) == CMD_ERR_LEN) &&
           (cmd_parse(&s_test_parser, frame, 2U, &cmd) == CMD_ERR_LEN) &&
           (cmd == CMD_NONE) &&
           (cmd_parse_rejected(&s_test_parser) == 3U);

    return b_ok && s1_cmd_expect(1U, CMD_OK, CMD_PAUSE);
}

/*
 * S1-U10: every value other than 1..3 is rejected: 0 (NONE), 4 (just
 * past ABORT), 255, and the ASCII text 'P' someone might type.
 */
static bool s1_cmd_bad_code(void)
{
    cmd_parse_init(&s_test_parser);
    return s1_cmd_expect(0U,          CMD_ERR_CODE, CMD_NONE) &&
           s1_cmd_expect(4U,          CMD_ERR_CODE, CMD_NONE) &&
           s1_cmd_expect(255U,        CMD_ERR_CODE, CMD_NONE) &&
           s1_cmd_expect((uint8_t)'P', CMD_ERR_CODE, CMD_NONE) &&
           (cmd_parse_rejected(&s_test_parser) == 4U) &&
           s1_cmd_expect(3U, CMD_OK, CMD_ABORT);
}

/* ---- Packing tests -------------------------------------------------- */

/* Expected bytes, worked out independently of telem_pack.c (by a
   separate script) from the inputs in s1_pack_golden(). */
static const uint8_t s_golden_telem[TELEM_FRAME_LEN] =
{
    0x01U, 0xD2U, 0x04U, 0x53U, 0xC8U, 0xA8U, 0xADU, 0x70U,
    0x08U, 0xA5U, 0x3CU, 0x2AU, 0xAFU, 0xF4U, 0x00U
};
static const uint8_t s_golden_hb[TELEM_HB_FRAME_LEN] =
{
    0x09U, 0xC8U, 0x43U, 0x9BU
};
static const uint8_t s_golden_evt[TELEM_EVT_FRAME_LEN] =
{
    0x11U, 0x26U, 0x01U
};

/*
 * S1-U11: fixed inputs give exactly the expected bytes.  This catches
 * wrong field order, overlap and bit order, which a round-trip alone
 * can miss.
 */
static bool s1_pack_golden(void)
{
    telem_status_t s;
    telem_hb_t     hb;
    uint8_t        out[TELEM_FRAME_LEN];

    s.time_ds       = 1234U;   /* 123.4 s */
    s.mission_state = 3U;
    s.fault_mask    = 5U;
    s.speed_left    = 200;
    s.speed_right   = -150;
    s.motion_state  = 2U;
    s.dist_mm       = 4321U;
    s.line_zone     = 2U;
    s.b_junction    = true;
    s.b_line_lost   = false;
    s.letter        = 10U;     /* 'K' */
    s.last_action   = 2U;      /* LEFT */
    s.b_map_ok      = true;
    s.barcodes      = 3U;
    s.hump_mm       = 42U;
    s.b_obstacle    = true;
    s.obst_dist_cm  = 87U;
    s.obst_side     = 1U;
    s.obst_width_cm = 15U;

    if ((telem_pack_status(&s, out, (uint8_t)sizeof(out)) != TELEM_FRAME_LEN) ||
        !s1_bytes_equal(out, s_golden_telem, TELEM_FRAME_LEN))
    {
        return false;
    }

    hb.seq           = 200U;
    hb.mission_state = 3U;
    hb.outage_count  = 20U;
    hb.b_wifi_up     = true;
    hb.b_mqtt_up     = false;
    hb.b_map_ok      = true;
    hb.rejected      = 9U;

    if ((telem_pack_hb(&hb, out, (uint8_t)sizeof(out)) != TELEM_HB_FRAME_LEN) ||
        !s1_bytes_equal(out, s_golden_hb, TELEM_HB_FRAME_LEN))
    {
        return false;
    }

    /* STATE_CHANGE: new state 4 (bits 0-3), old state 2 (bits 4-7) */
    return (telem_pack_event(TELEM_EVT_STATE_CHANGE, (uint16_t)(4U | (2U << 4)),
                             out, (uint8_t)sizeof(out)) == TELEM_EVT_FRAME_LEN) &&
           s1_bytes_equal(out, s_golden_evt, TELEM_EVT_FRAME_LEN);
}

/*
 * S1-U12: the frame is TELEM_FRAME_LEN (15) bytes, and every field comes
 * back unchanged at its minimum and at its largest real value.
 */
static bool s1_pack_limits(void)
{
    telem_status_t in;
    telem_status_t back;
    uint8_t        out[TELEM_FRAME_LEN];

    s1_status_fill(&in, false);
    if ((telem_pack_status(&in, out, (uint8_t)sizeof(out)) != 15U) ||
        !telem_unpack_status(out, TELEM_FRAME_LEN, &back) ||
        !s1_status_equal(&in, &back))
    {
        return false;
    }

    s1_status_fill(&in, true);
    return (telem_pack_status(&in, out, (uint8_t)sizeof(out)) == 15U) &&
           telem_unpack_status(out, TELEM_FRAME_LEN, &back) &&
           s1_status_equal(&in, &back);
}

/*
 * S1-U13: values past a field's range are clamped to its largest real
 * value (never to the no-data value), a letter past 'Z' becomes NONE,
 * and every no-data value survives the round trip.
 */
static bool s1_pack_clamp(void)
{
    telem_status_t in;
    telem_status_t back;
    uint8_t        out[TELEM_FRAME_LEN];
    bool           b_ok;

    s1_status_fill(&in, false);
    in.speed_left   = 600;
    in.speed_right  = -600;
    in.hump_mm      = 255U;          /* is NONE: must stay NONE */
    in.obst_dist_cm = 700U;
    in.letter       = 27U;           /* not a letter */
    in.barcodes     = 40U;
    in.line_zone    = 9U;

    b_ok = (telem_pack_status(&in, out, (uint8_t)sizeof(out)) == TELEM_FRAME_LEN) &&
           telem_unpack_status(out, TELEM_FRAME_LEN, &back) &&
           (back.speed_left   == TELEM_SPEED_MAX) &&
           (back.speed_right  == TELEM_SPEED_MIN) &&
           (back.hump_mm      == TELEM_HUMP_NONE) &&
           (back.obst_dist_cm == TELEM_OBST_DIST_MAX) &&
           (back.letter       == TELEM_LETTER_NONE) &&
           (back.barcodes     == TELEM_BARCODES_MAX) &&
           (back.line_zone    == TELEM_ZONE_MAX);
    if (!b_ok)
    {
        return false;
    }

    s1_status_fill(&in, false);
    in.mission_state = TELEM_MSTATE_NONE;
    in.speed_left    = TELEM_SPEED_NONE;
    in.speed_right   = TELEM_SPEED_NONE;
    in.line_zone     = TELEM_ZONE_NONE;
    in.letter        = TELEM_LETTER_NONE;
    in.hump_mm       = TELEM_HUMP_NONE;
    in.obst_dist_cm  = TELEM_OBST_DIST_NONE;
    in.obst_width_cm = TELEM_OBST_WIDTH_NONE;

    return (telem_pack_status(&in, out, (uint8_t)sizeof(out)) == TELEM_FRAME_LEN) &&
           telem_unpack_status(out, TELEM_FRAME_LEN, &back) &&
           s1_status_equal(&in, &back);
}

/*
 * S1-U14: heartbeat is 4 bytes and round-trips at min and max.
 */
static bool s1_pack_hb(void)
{
    telem_hb_t in;
    telem_hb_t back;
    uint8_t    out[TELEM_HB_FRAME_LEN];
    bool       b_ok;

    in.seq           = 0U;
    in.mission_state = 0U;
    in.outage_count  = 0U;
    in.b_wifi_up     = false;
    in.b_mqtt_up     = false;
    in.b_map_ok      = false;
    in.rejected      = 0U;

    b_ok = (telem_pack_hb(&in, out, (uint8_t)sizeof(out)) == 4U) &&
           telem_unpack_hb(out, TELEM_HB_FRAME_LEN, &back) &&
           (back.seq == 0U) && (back.outage_count == 0U) &&
           !back.b_wifi_up && !back.b_mqtt_up && !back.b_map_ok &&
           (back.rejected == 0U);
    if (!b_ok)
    {
        return false;
    }

    in.seq           = 255U;
    in.mission_state = TELEM_MSTATE_NONE;
    in.outage_count  = OUTAGE_BUF_DEPTH;
    in.b_wifi_up     = true;
    in.b_mqtt_up     = true;
    in.b_map_ok      = true;
    in.rejected      = 200U;   /* stops at 15 */

    return (telem_pack_hb(&in, out, (uint8_t)sizeof(out)) == 4U) &&
           telem_unpack_hb(out, TELEM_HB_FRAME_LEN, &back) &&
           (back.seq == 255U) && (back.mission_state == TELEM_MSTATE_NONE) &&
           (back.outage_count == OUTAGE_BUF_DEPTH) &&
           back.b_wifi_up && back.b_mqtt_up && back.b_map_ok &&
           (back.rejected == TELEM_HB_REJECTED_MAX);
}

/*
 * S1-U15: an event is 3 bytes, type and data come back unchanged at
 * their limits, and a bad type or oversized data is refused.
 */
static bool s1_pack_event(void)
{
    uint8_t  out[TELEM_EVT_FRAME_LEN];
    uint8_t  type = 0U;
    uint16_t data = 0U;

    return (telem_pack_event(TELEM_EVT_TYPE_MAX, TELEM_EVT_DATA_MAX,
                             out, (uint8_t)sizeof(out)) == 3U) &&
           telem_unpack_event(out, TELEM_EVT_FRAME_LEN, &type, &data) &&
           (type == TELEM_EVT_TYPE_MAX) && (data == TELEM_EVT_DATA_MAX) &&
           (telem_pack_event(TELEM_EVT_MAP_ACK, 0U,
                             out, (uint8_t)sizeof(out)) == 3U) &&
           telem_unpack_event(out, TELEM_EVT_FRAME_LEN, &type, &data) &&
           (type == TELEM_EVT_MAP_ACK) && (data == 0U) &&
           (telem_pack_event(8U, 0U, out, (uint8_t)sizeof(out)) == 0U) &&
           (telem_pack_event(0U, (uint16_t)(TELEM_EVT_DATA_MAX + 1U),
                             out, (uint8_t)sizeof(out)) == 0U);
}

/*
 * S1-U16: header carries version and type; the replayed bit is set
 * without changing anything else; NULL and too-small buffers are
 * refused and nothing is written; unpack refuses the wrong type.
 */
static bool s1_pack_header(void)
{
    telem_status_t s;
    telem_status_t back;
    telem_hb_t     hb_back;
    uint8_t        out[TELEM_FRAME_LEN];
    uint8_t        small[TELEM_FRAME_LEN - 1U];
    uint8_t        before;
    uint8_t        i;

    for (i = 0U; i < (uint8_t)sizeof(small); i++)
    {
        small[i] = 0xAAU;
    }

    s1_status_fill(&s, true);
    if (telem_pack_status(&s, out, (uint8_t)sizeof(out)) != TELEM_FRAME_LEN)
    {
        return false;
    }
    before = out[0];
    telem_mark_replayed(out);

    return (telem_frame_version(out) == TELEM_VERSION) &&
           (telem_frame_type(out) == TELEM_TYPE_TELEM) &&
           telem_is_replayed(out) &&
           (out[0] == (uint8_t)(before | TELEM_HDR_REPLAYED)) &&
           telem_unpack_status(out, TELEM_FRAME_LEN, &back) &&   /* still valid */
           !telem_unpack_hb(out, TELEM_HB_FRAME_LEN, &hb_back) &&   /* wrong type */
           (telem_pack_status(NULL, out, (uint8_t)sizeof(out)) == 0U) &&
           (telem_pack_status(&s, NULL, TELEM_FRAME_LEN) == 0U) &&
           (telem_pack_status(&s, small, (uint8_t)sizeof(small)) == 0U) &&
           (small[0] == 0xAAU) &&
           (telem_pack_hb(NULL, out, (uint8_t)sizeof(out)) == 0U);
}

/*
 * Fill a status with every field at 0 / minimum, or at its largest real
 * value.
 */
static void s1_status_fill(telem_status_t *p_s, bool b_max)
{
    p_s->time_ds       = b_max ? 0xFFFFU : 0U;
    p_s->mission_state = b_max ? TELEM_MSTATE_MAX : 0U;
    p_s->fault_mask    = b_max ? TELEM_FAULT_MAX : 0U;
    p_s->speed_left    = b_max ? (int16_t)TELEM_SPEED_MAX : (int16_t)TELEM_SPEED_MIN;
    p_s->speed_right   = b_max ? (int16_t)TELEM_SPEED_MIN : (int16_t)TELEM_SPEED_MAX;
    p_s->motion_state  = b_max ? TELEM_MOTION_MAX : 0U;
    p_s->dist_mm       = b_max ? 0xFFFFU : 0U;
    p_s->line_zone     = b_max ? TELEM_ZONE_MAX : 0U;
    p_s->b_junction    = b_max;
    p_s->b_line_lost   = b_max;
    p_s->letter        = b_max ? TELEM_LETTER_MAX : 0U;
    p_s->last_action   = b_max ? TELEM_ACTION_MAX : 0U;
    p_s->b_map_ok      = b_max;
    p_s->barcodes      = b_max ? TELEM_BARCODES_MAX : 0U;
    p_s->hump_mm       = b_max ? TELEM_HUMP_MAX : 0U;
    p_s->b_obstacle    = b_max;
    p_s->obst_dist_cm  = b_max ? TELEM_OBST_DIST_MAX : 0U;
    p_s->obst_side     = b_max ? TELEM_OBST_SIDE_MAX : 0U;
    p_s->obst_width_cm = b_max ? TELEM_OBST_WIDTH_MAX : 0U;
}

static bool s1_status_equal(const telem_status_t *p_a,
                            const telem_status_t *p_b)
{
    return (p_a->time_ds       == p_b->time_ds) &&
           (p_a->mission_state == p_b->mission_state) &&
           (p_a->fault_mask    == p_b->fault_mask) &&
           (p_a->speed_left    == p_b->speed_left) &&
           (p_a->speed_right   == p_b->speed_right) &&
           (p_a->motion_state  == p_b->motion_state) &&
           (p_a->dist_mm       == p_b->dist_mm) &&
           (p_a->line_zone     == p_b->line_zone) &&
           (p_a->b_junction    == p_b->b_junction) &&
           (p_a->b_line_lost   == p_b->b_line_lost) &&
           (p_a->letter        == p_b->letter) &&
           (p_a->last_action   == p_b->last_action) &&
           (p_a->b_map_ok      == p_b->b_map_ok) &&
           (p_a->barcodes      == p_b->barcodes) &&
           (p_a->hump_mm       == p_b->hump_mm) &&
           (p_a->b_obstacle    == p_b->b_obstacle) &&
           (p_a->obst_dist_cm  == p_b->obst_dist_cm) &&
           (p_a->obst_side     == p_b->obst_side) &&
           (p_a->obst_width_cm == p_b->obst_width_cm);
}

static bool s1_bytes_equal(const uint8_t *p_a, const uint8_t *p_b,
                           uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++)
    {
        if (p_a[i] != p_b[i])
        {
            return false;
        }
    }
    return true;
}

/* ---- Mapping tests -------------------------------------------------- */

/* F = K (10), U-TURN = D (3), LEFT = A (0), RIGHT = Z (25).
   24-bit value 0x0C806A, byte 0 first (worked out separately). */
static const uint8_t s_map_good[CMD_MAP_FRAME_LEN] = { 0x6AU, 0x80U, 0x0CU };

/*
 * Parse a map frame with s_test_parser (READY or not) and check the
 * result.  On CMD_OK also checks the word holds the frame plus the valid
 * bit; on error, that the word is 0.
 */
static bool s1_map_expect(const uint8_t *p_frame, bool b_ready,
                          cmd_result_t want)
{
    uint32_t word = 0xFFFFFFFFUL;
    uint32_t raw;

    if (cmd_parse_map(&s_test_parser, p_frame, CMD_MAP_FRAME_LEN,
                      b_ready, &word) != want)
    {
        return false;
    }
    if (want != CMD_OK)
    {
        return (word == 0UL);
    }
    raw = (uint32_t)p_frame[0] | ((uint32_t)p_frame[1] << 8) |
          ((uint32_t)p_frame[2] << 16);
    return (word == (raw | CMD_MAP_VALID));
}

/*
 * S1-U18: a valid map is accepted, and each mapped letter gives its
 * movement; a letter not in the map gives nothing.
 */
static bool s1_map_valid(void)
{
    uint32_t word = 0UL;
    uint8_t  move = 0xFFU;

    cmd_parse_init(&s_test_parser);
    if ((cmd_parse_map(&s_test_parser, s_map_good, CMD_MAP_FRAME_LEN,
                       true, &word) != CMD_OK) ||
        (word != (0x000C806AUL | CMD_MAP_VALID)) ||
        (cmd_parse_rejected(&s_test_parser) != 0U))
    {
        return false;
    }

    return cmd_map_lookup(word, 10U, &move) && (move == CMD_MOVE_FORWARD) &&
           cmd_map_lookup(word, 3U, &move)  && (move == CMD_MOVE_UTURN) &&
           cmd_map_lookup(word, 0U, &move)  && (move == CMD_MOVE_LEFT) &&
           cmd_map_lookup(word, 25U, &move) && (move == CMD_MOVE_RIGHT) &&
           !cmd_map_lookup(word, 1U, &move) &&        /* B: not mapped   */
           !cmd_map_lookup(0UL, 10U, &move);          /* no valid map    */
}

/*
 * S1-U19: NULL, 0, 2 and 4-byte frames are rejected and counted.
 */
static bool s1_map_bad_length(void)
{
    uint8_t  frame[4] = { 0x6AU, 0x80U, 0x0CU, 0x00U };
    uint32_t word = 1UL;

    cmd_parse_init(&s_test_parser);
    return (cmd_parse_map(&s_test_parser, NULL, 3U, true, &word) == CMD_ERR_NULL) &&
           (word == 0UL) &&
           (cmd_parse_map(&s_test_parser, frame, 0U, true, &word) == CMD_ERR_LEN) &&
           (cmd_parse_map(&s_test_parser, frame, 2U, true, &word) == CMD_ERR_LEN) &&
           (cmd_parse_map(&s_test_parser, frame, 4U, true, &word) == CMD_ERR_LEN) &&
           (cmd_parse_rejected(&s_test_parser) == 4U);
}

/*
 * S1-U20: a letter code of 26 or 31 in any position, or a spare bit
 * set, is rejected.
 */
static bool s1_map_bad_letter(void)
{
    /* 26 in FORWARD: bits 0-4 = 26, others valid (D, A, Z as above) */
    const uint8_t fwd_26[CMD_MAP_FRAME_LEN]   = { 0x7AU, 0x80U, 0x0CU };
    /* 31 in RIGHT: bits 15-19 = 31 */
    const uint8_t right_31[CMD_MAP_FRAME_LEN] = { 0x6AU, 0x80U, 0x0FU };
    /* good map with bit 23 set */
    const uint8_t spare[CMD_MAP_FRAME_LEN]    = { 0x6AU, 0x80U, 0x8CU };

    cmd_parse_init(&s_test_parser);
    return s1_map_expect(fwd_26,   true, CMD_ERR_LETTER) &&
           s1_map_expect(right_31, true, CMD_ERR_LETTER) &&
           s1_map_expect(spare,    true, CMD_ERR_RESERVED) &&
           (cmd_parse_rejected(&s_test_parser) == 3U) &&
           s1_map_expect(s_map_good, true, CMD_OK);
}

/*
 * S1-U21: the same letter twice is rejected (the map would be
 * ambiguous); a valid map outside READY is rejected; each rejection maps
 * to the right MAP_ACK reason.
 */
static bool s1_map_duplicate_state(void)
{
    /* FORWARD = RIGHT = Z: bits 0-4 = 25 */
    const uint8_t dup[CMD_MAP_FRAME_LEN] = { 0x79U, 0x80U, 0x0CU };

    cmd_parse_init(&s_test_parser);
    return s1_map_expect(dup, true, CMD_ERR_DUPLICATE) &&
           s1_map_expect(s_map_good, false, CMD_ERR_STATE) &&
           (cmd_parse_rejected(&s_test_parser) == 2U) &&
           (cmd_map_reason(CMD_ERR_LEN)       == CMD_MAP_REASON_FORMAT) &&
           (cmd_map_reason(CMD_ERR_RESERVED)  == CMD_MAP_REASON_FORMAT) &&
           (cmd_map_reason(CMD_ERR_LETTER)    == CMD_MAP_REASON_LETTER) &&
           (cmd_map_reason(CMD_ERR_DUPLICATE) == CMD_MAP_REASON_DUPLICATE) &&
           (cmd_map_reason(CMD_ERR_STATE)     == CMD_MAP_REASON_STATE);
}

/*
 * S1-U22: IF-18 has no valid map at boot, so S5 can refuse to start; a
 * stored word reads back unchanged and s1_nav_lookup uses it.  The store
 * is cleared again afterwards so the test leaves no map behind.
 */
static bool s1_map_store(void)
{
    UB   move = 0xFFU;
    bool b_ok;

    b_ok = ((s1_get_nav_map() & S1_NAV_MAP_VALID) == 0UL) &&
           !s1_nav_lookup(10U, &move);
    if (!b_ok)
    {
        return false;
    }

    s1_set_nav_map((UW)(0x000C806AUL | CMD_MAP_VALID));
    b_ok = (s1_get_nav_map() == (UW)(0x000C806AUL | CMD_MAP_VALID)) &&
           s1_nav_lookup(25U, &move) && (move == S1_MOVE_RIGHT);

    s1_set_nav_map(0U);
    return b_ok;
}

/* ---- Task wiring tests ---------------------------------------------- */

#define S1_TEST_TIMING_MS     (2100)  /* capture window for S1-U24      */
#define S1_TEST_TOL_MS        (10U)   /* allowed timing error           */
#define S1_TEST_ACK_MS        (300)   /* max wait for an ACK event      */
#define S1_TEST_TELEM_MS      (700)   /* max wait for a telemetry frame */
#define S1_TEST_MAX_TIMES     (8U)
#define S1_TEST_BACKLOG_MS    (5000)  /* max wait for the backlog to clear */

/*
 * S1-U23: the three message buffers exist with the right message sizes.
 */
static bool s1_task_objects(void)
{
    T_RMBF r;

    return (tk_ref_mbf(mbf_telem_status, &r) == E_OK) &&
           (r.maxmsz == (INT)sizeof(s1_status_msg_t)) &&
           (tk_ref_mbf(mbf_mqtt_tx, &r) == E_OK) &&
           (r.maxmsz == (INT)S1_TX_MSG_MAX) &&
           (tk_ref_mbf(mbf_mqtt_rx, &r) == E_OK) &&
           (r.maxmsz == (INT)S1_RX_MSG_MAX);
}

/*
 * S1-U24: for about 2 s, take everything the task puts in mbf_mqtt_tx
 * (standing in for mqtt_utk.c).  Telemetry frames must be 500 ms apart
 * and heartbeats 1000 ms apart, within S1_TEST_TOL_MS, and every frame
 * must unpack.  The test runs at priority 1, so it wakes the moment a
 * frame is sent and the arrival time is the send time.  Any backlog from
 * before the test (TX full, frames queued for replay) is cleared first.
 */
static bool s1_task_timing(void)
{
    UW             t_telem[S1_TEST_MAX_TIMES];
    UW             t_hb[S1_TEST_MAX_TIMES];
    uint8_t        n_telem = 0U;
    uint8_t        n_hb = 0U;
    uint8_t        msg[S1_TX_MSG_MAX];
    telem_status_t st;
    telem_hb_t     hb;
    UW             end_ms;
    INT            left;
    INT            len;
    uint8_t        i;

    /* Nothing drains mbf_mqtt_tx in a test build, so by now it has been
       full and frames have queued in the outage buffer.  Let that backlog
       replay first, then time the live frames. */
    if (!s1_drain_backlog())
    {
        return false;
    }
    end_ms = s1_now_ms() + (UW)S1_TEST_TIMING_MS;

    for (;;)
    {
        left = (INT)(end_ms - s1_now_ms());
        if (left <= 0)
        {
            break;
        }
        len = tk_rcv_mbf(mbf_mqtt_tx, msg, (TMO)left);
        if (len < 2)
        {
            continue;                       /* timeout: loop ends above */
        }

        if (msg[0] == S1_TOPIC_TELEM)
        {
            if (!telem_unpack_status(&msg[1], (uint8_t)(len - 1), &st))
            {
                return false;
            }
            if (n_telem < S1_TEST_MAX_TIMES)
            {
                t_telem[n_telem++] = s1_now_ms();
            }
        }
        else if (msg[0] == S1_TOPIC_HB)
        {
            if (!telem_unpack_hb(&msg[1], (uint8_t)(len - 1), &hb))
            {
                return false;
            }
            if (n_hb < S1_TEST_MAX_TIMES)
            {
                t_hb[n_hb++] = s1_now_ms();
            }
        }
        else
        {
            /* events: not timed here */
        }
    }

    if ((n_telem < 4U) || (n_hb < 2U))
    {
        return false;
    }
    for (i = 1U; i < n_telem; i++)
    {
        UW gap = t_telem[i] - t_telem[i - 1U];
        if ((gap < (500U - S1_TEST_TOL_MS)) || (gap > (500U + S1_TEST_TOL_MS)))
        {
            return false;
        }
    }
    for (i = 1U; i < n_hb; i++)
    {
        UW gap = t_hb[i] - t_hb[i - 1U];
        if ((gap < (1000U - S1_TEST_TOL_MS)) || (gap > (1000U + S1_TEST_TOL_MS)))
        {
            return false;
        }
    }
    return true;
}

/*
 * S1-U25: an IF-13 status sent the way S5 will send it shows up in the
 * next telemetry frame, and the new mission state sends a STATE_CHANGE
 * event (new = READY, old = 15 "not known", as nothing was sent before).
 */
static bool s1_task_status_in(void)
{
    uint8_t        frame[S1_TX_MSG_MAX];
    uint8_t        len = 0U;
    telem_status_t st;

    s1_flush_tx();
    if (!s1_send_status((uint8_t)S1_MSTATE_READY, 16U))   /* 'Q' */
    {
        return false;
    }

    /* STATE_CHANGE data: new state bits 0-3, old state bits 4-7 */
    if (!s1_wait_event(TELEM_EVT_STATE_CHANGE,
                       (uint16_t)(S1_MSTATE_READY | (S1_MSTATE_NONE << 4))))
    {
        return false;
    }

    return s1_wait_tx(S1_TOPIC_TELEM, 0xFFU, S1_TEST_TELEM_MS,
                      frame, &len, NULL) &&
           telem_unpack_status(frame, len, &st) &&
           (st.mission_state == S1_MSTATE_READY) &&
           (st.letter == 16U) &&
           (st.time_ds == 1234U) &&
           (st.fault_mask == 2U) &&
           (st.barcodes == 3U);
}

/*
 * S1-U26: commands put in mbf_mqtt_rx (as mqtt_utk.c will) are checked
 * and answered with CMD_ACK: data = command (bits 0-1) + accepted (bit 2).
 */
static bool s1_task_cmd_in(void)
{
    const uint8_t pause[1]   = { 0x01U };
    const uint8_t junk[1]    = { 0x07U };
    const uint8_t two[2]     = { 0x01U, 0x01U };

    s1_flush_tx();
    return s1_send_rx(S1_TOPIC_CMD, pause, 1U) &&
           s1_wait_event(TELEM_EVT_CMD_ACK, 0x05U) &&     /* PAUSE, ok  */
           s1_send_rx(S1_TOPIC_CMD, junk, 1U) &&
           s1_wait_event(TELEM_EVT_CMD_ACK, 0x00U) &&     /* rejected   */
           s1_send_rx(S1_TOPIC_CMD, two, 2U) &&
           s1_wait_event(TELEM_EVT_CMD_ACK, 0x00U);       /* bad length */
}

/*
 * S1-U27: a map put in mbf_mqtt_rx while READY is stored in IF-18 and
 * acknowledged (MAP_ACK data 1); after the state leaves READY the same
 * map is refused with reason STATE (data = 3 << 1).  Clears the map after.
 */
static bool s1_task_map_in(void)
{
    bool b_ok;

    s1_flush_tx();
    b_ok = s1_send_rx(S1_TOPIC_MAP, s_map_good, CMD_MAP_FRAME_LEN) &&
           s1_wait_event(TELEM_EVT_MAP_ACK, 0x01U) &&
           (s1_get_nav_map() == (UW)(0x000C806AUL | S1_NAV_MAP_VALID)) &&
           s1_send_status(2U, 16U) &&
           s1_wait_event(TELEM_EVT_STATE_CHANGE,
                         (uint16_t)(2U | (S1_MSTATE_READY << 4))) &&
           s1_send_rx(S1_TOPIC_MAP, s_map_good, CMD_MAP_FRAME_LEN) &&
           s1_wait_event(TELEM_EVT_MAP_ACK,
                         (uint16_t)(CMD_MAP_REASON_STATE << 1));

    s1_set_nav_map(0U);
    return b_ok;
}

/*
 * Empty mbf_mqtt_tx, so a test only sees frames sent after it starts.
 */
static void s1_flush_tx(void)
{
    uint8_t msg[S1_TX_MSG_MAX];

    while (tk_rcv_mbf(mbf_mqtt_tx, msg, TMO_POL) > 0)
    {
        /* discard */
    }
}

/*
 * Empty mbf_mqtt_tx (old frames queued before it filled), then take
 * everything until a live (not replayed) telemetry frame arrives: from
 * then on the outage buffer is empty and frames go out on their own
 * schedule.  False if that takes longer than S1_TEST_BACKLOG_MS (a full
 * backlog of 20 replays in about 2 s).
 */
static bool s1_drain_backlog(void)
{
    uint8_t msg[S1_TX_MSG_MAX];
    UW      end_ms;
    INT     left;
    INT     len;

    s1_flush_tx();
    end_ms = s1_now_ms() + (UW)S1_TEST_BACKLOG_MS;

    for (;;)
    {
        left = (INT)(end_ms - s1_now_ms());
        if (left <= 0)
        {
            return false;
        }
        len = tk_rcv_mbf(mbf_mqtt_tx, msg, (TMO)left);
        if ((len >= 2) && (msg[0] == S1_TOPIC_TELEM) &&
            !telem_is_replayed(&msg[1]))
        {
            return true;
        }
    }
}

/*
 * Take frames from mbf_mqtt_tx until one on 'topic' arrives (for events,
 * of type 'evt_type' unless it is 0xFF), or timeout_ms passes.  Copies
 * the frame out; for events also returns its data.
 */
static bool s1_wait_tx(uint8_t topic, uint8_t evt_type, INT timeout_ms,
                       uint8_t *p_frame, uint8_t *p_len, uint16_t *p_data)
{
    uint8_t  msg[S1_TX_MSG_MAX];
    UW       end_ms = s1_now_ms() + (UW)timeout_ms;
    INT      left;
    INT      len;
    uint8_t  type;
    uint16_t data;
    uint8_t  i;

    for (;;)
    {
        left = (INT)(end_ms - s1_now_ms());
        if (left <= 0)
        {
            return false;
        }
        len = tk_rcv_mbf(mbf_mqtt_tx, msg, (TMO)left);
        if ((len < 2) || (msg[0] != topic))
        {
            continue;
        }
        if ((topic == S1_TOPIC_EVT) && (evt_type != 0xFFU))
        {
            if (!telem_unpack_event(&msg[1], (uint8_t)(len - 1), &type, &data) ||
                (type != evt_type))
            {
                continue;
            }
            if (p_data != NULL)
            {
                *p_data = data;
            }
        }
        for (i = 0U; i < (uint8_t)(len - 1); i++)
        {
            p_frame[i] = msg[i + 1];
        }
        *p_len = (uint8_t)(len - 1);
        return true;
    }
}

/*
 * Wait up to S1_TEST_ACK_MS for an event of evt_type and check its data.
 */
static bool s1_wait_event(uint8_t evt_type, uint16_t want_data)
{
    uint8_t  frame[S1_TX_MSG_MAX];
    uint8_t  len = 0U;
    uint16_t data = 0xFFFFU;

    return s1_wait_tx(S1_TOPIC_EVT, evt_type, S1_TEST_ACK_MS,
                      frame, &len, &data) &&
           (data == want_data);
}

/*
 * Send one IF-13 status, as S5 will: fixed test values plus the given
 * state and letter.
 */
static bool s1_send_status(uint8_t state, uint8_t letter)
{
    return s1_send_status_t(state, letter, 1234U);
}

/*
 * Send one IF-13 status, as S5 will, with the given mission time.
 */
static bool s1_send_status_t(uint8_t state, uint8_t letter,
                             uint16_t time_ds)
{
    s1_status_msg_t msg = {0};

    msg.mission_time_ds = time_ds;
    msg.mission_state   = state;
    msg.fault_mask      = 2U;
    msg.letter          = letter;
    msg.barcodes        = 3U;
    msg.obst_dist_cm    = (uint16_t)TELEM_OBST_DIST_NONE;
    msg.obst_width_cm   = (uint8_t)TELEM_OBST_WIDTH_NONE;

    return tk_snd_mbf(mbf_telem_status, &msg, (INT)sizeof(msg), TMO_POL) ==
           E_OK;
}

/*
 * Put [topic][payload] into mbf_mqtt_rx, as mqtt_utk.c will.
 */
static bool s1_send_rx(uint8_t topic, const uint8_t *p_payload, uint8_t len)
{
    uint8_t msg[S1_RX_MSG_MAX];
    uint8_t i;

    if (len > (S1_RX_MSG_MAX - 1U))
    {
        return false;
    }
    msg[0] = topic;
    for (i = 0U; i < len; i++)
    {
        msg[i + 1U] = p_payload[i];
    }
    return tk_snd_mbf(mbf_mqtt_rx, msg, (INT)(len + 1U), TMO_POL) == E_OK;
}

static UW s1_now_ms(void)
{
    SYSTIM now;

    (void)tk_get_tim(&now);
    return now.lo;
}

/* ---- Outage tests --------------------------------------------------- */

#define S1_TEST_STATUS_MS     (500)   /* fake S5 period in the tests      */
#define S1_TEST_SETTLE_MS     (150)   /* > 1 tick: task has seen a change */
#define S1_TEST_REPLAY_GAP_MS (50U)   /* 2 in one 100 ms tick = too close */
#define S1_TEST_SPAN_TOL_MS   (30U)   /* tick jitter allowed over a span  */

/*
 * S1-U29: with the link forced down for 3 s, no telemetry or heartbeat
 * reaches mbf_mqtt_tx.  When it comes back, the kept frames come out
 * marked "replayed", oldest first (mission time never goes backwards),
 * at most 1 per 100 ms tick (no two closer than 50 ms; n frames span at
 * least (n - 1) x 100 ms, less 30 ms for tick jitter).  The test plays
 * S5, sending mission time 1, 2, 3 ... every 500 ms, so each kept frame
 * can be told apart.
 */
static bool s1_outage_replay(void)
{
    s1_replay_result_t r;
    bool               b_ok;

    b_ok = s1_hold_link_down(1U, 6U);

    telem_test_force_link(TELEM_LINK_UP);
    (void)s1_send_status_t((uint8_t)S1_MSTATE_READY, 16U, 500U);

    b_ok = s1_collect_replay(1500, 6U, &r) && b_ok;
    s1_print_replay("S1-U29", &r);

    telem_test_force_link(TELEM_LINK_AUTO);
    return b_ok && r.b_order &&
           (r.n >= 5U) && (r.n <= 7U) &&
           (r.first <= 2U) && (r.last >= 5U) &&
           (r.min_gap >= S1_TEST_REPLAY_GAP_MS) &&
           (r.span + S1_TEST_SPAN_TOL_MS >= ((UW)r.n - 1U) * TELEM_TICK_MS);
}

/*
 * S1-U30: with the link down for ~12 s (23-24 frames, mission time
 * 100..123), only the newest 20 are kept: exactly 20 come back, in order,
 * at most 1 per tick.  At least 3 of the oldest were dropped, and each
 * mission time covers at most 2 frames, so the first kept one is >= 102.
 * New frames made during the replay queue behind it.
 */
static bool s1_outage_overflow_live(void)
{
    s1_replay_result_t r;
    bool               b_ok;

    b_ok = s1_hold_link_down(100U, 24U);

    telem_test_force_link(TELEM_LINK_UP);
    (void)s1_send_status_t((uint8_t)S1_MSTATE_READY, 16U, 500U);

    b_ok = s1_collect_replay(3500, 123U, &r) && b_ok;
    s1_print_replay("S1-U30", &r);

    telem_test_force_link(TELEM_LINK_AUTO);
    return b_ok && r.b_order &&
           (r.n == (uint8_t)OUTAGE_BUF_DEPTH) &&
           (r.first >= 102U) && (r.last >= 122U) &&
           (r.min_gap >= S1_TEST_REPLAY_GAP_MS) &&
           (r.span + S1_TEST_SPAN_TOL_MS >= ((UW)r.n - 1U) * TELEM_TICK_MS);
}

/*
 * Send mission time first_time, then force the link down and keep
 * sending first_time + 1, + 2 ... (sends in all) every 500 ms, playing
 * S5.  False if any telemetry or heartbeat reaches mbf_mqtt_tx while
 * the link is down.
 */
static bool s1_hold_link_down(uint16_t first_time, uint8_t sends)
{
    uint8_t msg[S1_TX_MSG_MAX];
    UW      end_ms;
    INT     left;
    INT     len;
    uint8_t k;

    (void)s1_send_status_t((uint8_t)S1_MSTATE_READY, 16U, first_time);
    (void)tk_dly_tsk(S1_TEST_SETTLE_MS);
    telem_test_force_link(TELEM_LINK_DOWN);
    (void)tk_dly_tsk(S1_TEST_SETTLE_MS);
    s1_flush_tx();

    for (k = 1U; k < sends; k++)
    {
        (void)s1_send_status_t((uint8_t)S1_MSTATE_READY, 16U,
                               (uint16_t)(first_time + k));
        end_ms = s1_now_ms() + (UW)S1_TEST_STATUS_MS;

        for (;;)
        {
            left = (INT)(end_ms - s1_now_ms());
            if (left <= 0)
            {
                break;
            }
            len = tk_rcv_mbf(mbf_mqtt_tx, msg, (TMO)left);
            if ((len > 0) &&
                ((msg[0] == S1_TOPIC_TELEM) || (msg[0] == S1_TOPIC_HB)))
            {
                return false;            /* sent while the link was down */
            }
        }
    }
    return true;
}

/*
 * For window_ms, take everything from mbf_mqtt_tx and measure the
 * replayed outage frames (mission time <= max_time): how many, first and
 * last mission time, shortest gap and total span between arrivals, and
 * whether every one was marked replayed with mission time never going
 * backwards.  False if none arrived.
 */
static bool s1_collect_replay(INT window_ms, uint16_t max_time,
                              s1_replay_result_t *p_res)
{
    uint8_t        msg[S1_TX_MSG_MAX];
    telem_status_t st;
    UW             end_ms = s1_now_ms() + (UW)window_ms;
    UW             first_ms = 0U;
    UW             last_ms = 0U;
    UW             now_ms;
    INT            left;
    INT            len;

    p_res->n       = 0U;
    p_res->first   = 0U;
    p_res->last    = 0U;
    p_res->min_gap = 0xFFFFFFFFUL;
    p_res->span    = 0U;
    p_res->b_order = true;

    for (;;)
    {
        left = (INT)(end_ms - s1_now_ms());
        if (left <= 0)
        {
            break;
        }
        len = tk_rcv_mbf(mbf_mqtt_tx, msg, (TMO)left);
        if ((len < 2) || (msg[0] != S1_TOPIC_TELEM) ||
            !telem_unpack_status(&msg[1], (uint8_t)(len - 1), &st) ||
            (st.time_ds > max_time))
        {
            continue;                    /* heartbeat, event or new frame */
        }

        now_ms = s1_now_ms();
        if (!telem_is_replayed(&msg[1]))
        {
            p_res->b_order = false;
        }
        if (p_res->n == 0U)
        {
            p_res->first = st.time_ds;
            first_ms     = now_ms;
        }
        else
        {
            if (st.time_ds < p_res->last)
            {
                p_res->b_order = false;
            }
            if ((now_ms - last_ms) < p_res->min_gap)
            {
                p_res->min_gap = now_ms - last_ms;
            }
        }
        p_res->last = st.time_ds;
        last_ms     = now_ms;
        p_res->span = last_ms - first_ms;
        if (p_res->n < 0xFFU)
        {
            p_res->n++;
        }
    }

    if (p_res->n < 2U)
    {
        p_res->min_gap = 0U;
    }
    return (p_res->n > 0U);
}

/*
 * Print what a replay test measured, so a PASS or FAIL can be checked.
 */
static void s1_print_replay(const char *p_id, const s1_replay_result_t *p_res)
{
    tm_printf((UB *)"[INFO] %s replayed n=%d first=%d last=%d "
              "min_gap=%d ms span=%d ms order=%s\n",
              p_id, (INT)p_res->n, (INT)p_res->first, (INT)p_res->last,
              (INT)p_res->min_gap, (INT)p_res->span,
              p_res->b_order ? "ok" : "BROKEN");
}

#endif /* APP_TEST */