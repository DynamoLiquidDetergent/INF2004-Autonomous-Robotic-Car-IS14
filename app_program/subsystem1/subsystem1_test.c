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

#endif /* APP_TEST */