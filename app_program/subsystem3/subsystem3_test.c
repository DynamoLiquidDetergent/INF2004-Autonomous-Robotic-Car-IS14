/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and
 *	integration results in testing/integration/TEST_LOG.md.
 */

#include "s3_private.h"
#include "code39.h"
#include "../test_report.h"

#if APP_TEST

/* INT-01 / INT-02 need line_sensor_task (Commit 3 onwards) */
#define S3_RUN_INT_TESTS   1

/* TEST_INIT() normally declares the pass/fail counters as locals of one
   function.  The checks here are split across helper functions, so the
   counters live at file scope instead (expands to: static INT test_pass_ = 0, ...). */
static TEST_INIT();

// queue creation is tested by checking that the queue IDs are non-zero after init
static void test_queues_created(void)
{
    TEST_CHECK("S3-U01",
               q_line_data    > 0 &&
               q_gp28_samples > 0 &&
               q_decoded_char > 0 &&
               q_barcode_cmd  > 0 &&
               mbx_char_map   > 0,
               "all S3 queue IDs are valid after init");
}

// line_data_pack() is tested by packing known values and checking that the getters recover them
static void test_line_data_pack(void)
{
    /* Test: centred, no junction, no lost */
    LINE_DATA_MSG m = line_data_pack(LINE_ZONE_CENTRE, false, false);
    TEST_CHECK("S3-U02a",
               line_data_zone(m)     == LINE_ZONE_CENTRE &&
               line_data_junction(m) == false            &&
               line_data_lost(m)     == false,
               "line_data_pack: centre, no junction, no lost");

    /* Test: zone 2 (drifting left), junction set, line-lost set */
    m = line_data_pack(2u, true, true);
    TEST_CHECK("S3-U02b",
               line_data_zone(m)     == 2u  &&
               line_data_junction(m) == true &&
               line_data_lost(m)     == true,
               "line_data_pack: zone 2, junction + lost both set");

    /* Test: maximum zone value 7 with no flags */
    m = line_data_pack(7u, false, false);
    TEST_CHECK("S3-U02c",
               line_data_zone(m) == 7u,
               "line_data_pack: zone 7 stored correctly");
}

// gp28_pack() is tested by packing known values and checking that the getters recover them
static void test_gp28_pack(void)
{
    /* Test: maximum ADC (4095), maximum slot (15) */
    GP28_SAMPLE_MSG s = gp28_pack(4095u, 15u);
    TEST_CHECK("S3-U03a",
               gp28_adc(s)  == 4095u &&
               gp28_slot(s) == 15u,
               "gp28_pack: max ADC and max slot round-trip");

    /* Test: zero ADC, slot 0 */
    s = gp28_pack(0u, 0u);
    TEST_CHECK("S3-U03b",
               gp28_adc(s)  == 0u &&
               gp28_slot(s) == 0u,
               "gp28_pack: zero ADC and zero slot round-trip");

    /* Test: mid-range ADC (2048), slot 7 */
    s = gp28_pack(2048u, 7u);
    TEST_CHECK("S3-U03c",
               gp28_adc(s)  == 2048u &&
               gp28_slot(s) == 7u,
               "gp28_pack: mid ADC and mid slot round-trip");
}

// char_map validation: since Commit 4 this tests the REAL function used by
// barcode_decode_task (s3_char_map_valid), not a copy inside the test.
#define validate_char_map(m)   s3_char_map_valid(m)

static void test_char_map_validation(void)
{
    /* Valid mapping: four distinct letters */
    CHAR_MAP_MSG good = { 'A', 'B', 'C', 'D' };
    TEST_CHECK("S3-U04a", validate_char_map(&good),
               "char_map: four distinct A–Z letters accepted");

    /* Invalid: duplicate letter (forward == left) */
    CHAR_MAP_MSG dup = { 'A', 'B', 'A', 'D' };
    TEST_CHECK("S3-U04b", !validate_char_map(&dup),
               "char_map: duplicate letter rejected");

    /* Invalid: one letter outside A–Z */
    CHAR_MAP_MSG bad = { 'A', 'B', 'C', '1' };
    TEST_CHECK("S3-U04c", !validate_char_map(&bad),
               "char_map: non-letter character rejected");
}

// S3-U05: the Code 39 lookup table (filled by code39_init() in subsystem3_init())
// maps known forward 9-bit patterns to the right characters.
// Pattern bits: bit 8 = first element (a bar), 1 = wide, 0 = narrow.
static void test_code39_lut(void)
{
    TEST_CHECK("S3-U05",
               code39_lookup(0x109u)          == 'A' &&
               code39_lookup(0x0D0u)          == 'Z' &&
               code39_lookup(0x019u)          == 'D' &&
               code39_lookup(CODE39_STAR_FWD) == '*' &&
               code39_lookup(0x1FFu)          == CODE39_INVALID,
               "code39_lookup: A, Z, D, * correct and all-wide pattern invalid");
}

// Write the 9 element widths of one Code 39 character into elems[at..at+8]
static void put_symbol(BcElem *elems, uint16_t at, uint16_t pattern9,
                       uint16_t narrow, uint16_t wide)
{
    uint16_t i;

    for (i = 0; i < 9u; i++) {
        /* bit 8 is the first element, bit 0 the last */
        elems[at + i].w = (pattern9 & (1u << (8u - i))) ? wide : narrow;
    }
}

// S3-U06: code39_decode_symbol() decodes a hand-built "*A*" element array,
// both in the forward direction and fully reversed (robot crossing the
// barcode the other way).
static void test_code39_decode_symbol(void)
{
    enum { NARROW = 5, WIDE = 12 };     /* ratio 2.4 > 1.5 threshold */
    BcElem   elems[29];
    BcElem   tmp;
    uint8_t  fwd, rev;
    uint16_t i;

    /* Layout: * (9) | gap (1) | A (9) | gap (1) | * (9) = 29 elements */
    put_symbol(elems,  0u, CODE39_STAR_FWD, NARROW, WIDE);
    elems[9].w  = NARROW;                              /* inter-character gap */
    put_symbol(elems, 10u, 0x109u /* 'A' */,  NARROW, WIDE);
    elems[19].w = NARROW;                              /* inter-character gap */
    put_symbol(elems, 20u, CODE39_STAR_FWD, NARROW, WIDE);

    fwd = code39_decode_symbol(elems, BC_MIN_NARROW_SAMPLES, BC_WIDE_RATIO_X10);

    /* Reverse in place: swap elems[0]<->elems[28], elems[1]<->elems[27], ... */
    for (i = 0; i < 29u / 2u; i++) {
        tmp           = elems[i];
        elems[i]      = elems[28u - i];
        elems[28u - i] = tmp;
    }

    rev = code39_decode_symbol(elems, BC_MIN_NARROW_SAMPLES, BC_WIDE_RATIO_X10);

    TEST_CHECK("S3-U06",
               fwd == 'A' && rev == 'A',
               "code39_decode_symbol: *A* decodes to 'A' forward and reversed");
}

#if S3_RUN_INT_TESTS
// Integration test: check that line data flows from line_sensor_task to q_line_data within 100 ms
static void test_line_data_flow(void)
{
    LINE_DATA_MSG msg;

    /* tk_rcv_mbf returns the message size, or a negative error code */
    INT rc = tk_rcv_mbf(q_line_data, (void *)&msg, 100);

    TEST_CHECK("INT-01",
               rc == (INT)sizeof(msg),
               "line data flows from line_sensor_task to q_line_data within 100 ms");
}

// INT-02: GP28 samples are put into q_gp28_samples within 10 ms.
// Since Commit 4 barcode_decode_task empties the queue immediately, so the
// test counts successful sends by line_sensor_task instead of polling the
// queue (polling would race the decoder).
static void test_gp28_flow(void)
{
    UW before = line_sensor_gp28_sent();

    tk_dly_tsk(10);                 /* 2 ms sub-cycle: ~5 samples expected */

    TEST_CHECK("INT-02",
               line_sensor_gp28_sent() > before,
               "GP28 samples flow from line_sensor_task to q_gp28_samples within 10 ms");
}
#endif /* S3_RUN_INT_TESTS */

/* ---------------------------------------------------------------------
 *  S3-U07 / S3-U08: barcode_decode_task end to end, with a hand-made
 *  barcode injected into q_gp28_samples.  line_sensor_task's own GP28
 *  samples are paused meanwhile so they cannot mix with the injected ones.
 * --------------------------------------------------------------------- */
#define INJ_NARROW   5u     /* samples per narrow element */
#define INJ_WIDE     12u    /* samples per wide element   */
#define INJ_QUIET    50u    /* quiet zone, > BC_QUIET_SAMPLES (40) */

static uint8_t inj_slot;

static void inject(bool dark, uint16_t count)
{
    uint16_t        i;
    GP28_SAMPLE_MSG s;

    for (i = 0; i < count; i++) {
        s = gp28_pack(line_sensor_gp28_test_level(dark), inj_slot);
        inj_slot = (uint8_t)((inj_slot + 1u) & 0x0Fu);
        /* wait (up to 100 ms) for room: the decode task is reading */
        (void)tk_snd_mbf(q_gp28_samples, &s, sizeof(s), 100);
    }
}

/* One Code 39 character: 9 elements, bar first, bit 8 = first element */
static void inject_char(uint16_t pattern9)
{
    uint16_t i;

    for (i = 0; i < 9u; i++) {
        bool wide = (pattern9 & (1u << (8u - i))) != 0u;
        inject((i % 2u) == 0u, wide ? INJ_WIDE : INJ_NARROW);
    }
}

static void drain_u8(ID mbf)
{
    uint8_t junk;
    while (tk_rcv_mbf(mbf, &junk, TMO_POL) > 0) { }
}

static void drain_gp28(void)
{
    GP28_SAMPLE_MSG junk;
    while (tk_rcv_mbf(q_gp28_samples, &junk, TMO_POL) > 0) { }
}

// S3-U07: valid map A,B,C,D + injected "*A*" -> BARCODE_CMD_FORWARD within 200 ms
static void test_barcode_valid_map(void)
{
    CHAR_MAP_MSG map = { 'A', 'B', 'C', 'D' };   /* fwd, U-turn, left, right */
    BARCODE_CMD  cmd = BARCODE_CMD_NONE;
    INT          rc;

    (void)tk_snd_mbf(mbx_char_map, &map, sizeof(map), 100);
    tk_dly_tsk(50);                              /* decode task picks it up */

    line_sensor_pause_gp28(true);
    tk_dly_tsk(20);
    drain_gp28();
    drain_u8(q_barcode_cmd);
    drain_u8(q_decoded_char);

    inj_slot = 0;
    inject(false, INJ_QUIET);                    /* quiet zone              */
    inject_char(CODE39_STAR_FWD);                /* start '*'               */
    inject(false, INJ_NARROW);                   /* inter-character gap     */
    inject_char(0x109u);                         /* 'A'                     */
    inject(false, INJ_NARROW);                   /* inter-character gap     */
    inject_char(CODE39_STAR_FWD);                /* stop '*'                */
    inject(false, INJ_QUIET);                    /* quiet zone              */

    rc = tk_rcv_mbf(q_barcode_cmd, &cmd, 200);

    TEST_CHECK("S3-U07",
               rc == (INT)sizeof(cmd) && cmd == BARCODE_CMD_FORWARD,
               "barcode: valid map + injected *A* gives BARCODE_CMD_FORWARD");
}

// S3-U08: an invalid map (duplicate letter) is dropped silently: no command appears
static void test_barcode_invalid_map(void)
{
    CHAR_MAP_MSG bad = { 'A', 'A', 'C', 'D' };
    BARCODE_CMD  cmd;
    INT          rc;

    drain_u8(q_barcode_cmd);
    (void)tk_snd_mbf(mbx_char_map, &bad, sizeof(bad), 100);
    tk_dly_tsk(50);

    rc = tk_rcv_mbf(q_barcode_cmd, &cmd, TMO_POL);

    TEST_CHECK("S3-U08",
               rc == E_TMOUT,
               "barcode: invalid map dropped, no command sent");

    line_sensor_pause_gp28(false);               /* back to live samples */
}

// subsystem3_test() runs all unit and integration tests for subsystem 3
EXPORT void subsystem3_test(void)
{
    /* USB serial drops on reset: give the PC time to reopen the port
       so the S3 lines reach the log. */
    tk_dly_tsk(5000);
    tm_printf((UB*)"S3: tests starting\n");

    test_pass_ = 0;
    test_fail_ = 0;

    /* Unit tests — no timing dependency */
    test_queues_created();
    test_line_data_pack();
    test_gp28_pack();
    test_char_map_validation();
    test_code39_lut();             /* S3-U05 (Commit 2) */
    test_code39_decode_symbol();   /* S3-U06 (Commit 2) */

#if S3_RUN_INT_TESTS
    /* Integration tests — require line_sensor_task to be running.
       Calibrate first: sweep the robot slowly side to side across the
       line while this runs (about 1.3 s, gives up after 5 s). */
    tm_printf((UB*)"S3: calibrating -- sweep the robot across the line now\n");
    if (subsystem3_run_calibration()) {
        tm_printf((UB*)"S3: calibration done\n");
    } else {
        tm_printf((UB*)"S3: calibration timed out, default range kept\n");
    }
    test_line_data_flow();
    test_gp28_flow();
#endif

    /* Barcode decode task (Commit 4) -- after calibration, while the
       robot stays still */
    test_barcode_valid_map();      /* S3-U07 */
    test_barcode_invalid_map();    /* S3-U08 */

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
