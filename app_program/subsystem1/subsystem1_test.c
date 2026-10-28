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
#include "../test_report.h"

#if APP_TEST

#define S1_TEST_START_DELAY_MS  (5000)  /* time to open the serial port */
#define S1_TEST_FRAME_LEN       (2U)    /* bytes in each test frame     */

/* Test buffer: static so its ~680 bytes are not on the test task stack,
   and separate from anything the telemetry task uses. */
static outage_buf_t s_test_buf;

/* Test parser, separate from the one the telemetry task will use */
static cmd_parser_t s_test_parser;

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

#endif /* APP_TEST */