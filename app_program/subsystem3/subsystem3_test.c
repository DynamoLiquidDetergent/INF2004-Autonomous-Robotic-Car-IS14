/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and
 *	integration results in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "code39.h"
#include "../test_report.h"

#if APP_TEST

/* INT-01 / INT-02 need line_sensor_task, which arrives in Commit 3.
   Set to 1 then. */
#define S3_RUN_INT_TESTS 0

/* TEST_INIT() normally declares the pass/fail counters as locals of one
   function.  The checks here are split across helper functions, so the
   counters live at file scope instead (expands to: static INT test_pass_ = 0, ...). */
static TEST_INIT();

// queue creation is tested by checking that the queue IDs are non-zero after init
static void test_queues_created(void)
{
    TEST_CHECK("S3-U01",
               q_line_data > 0 &&
                   q_gp28_samples > 0 &&
                   q_decoded_char > 0 &&
                   q_barcode_cmd > 0 &&
                   mbx_char_map > 0,
               "all S3 queue IDs are valid after init");
}

// line_data_pack() is tested by packing known values and checking that the getters recover them
static void test_line_data_pack(void)
{
    /* Test: centred, no junction, no lost */
    LINE_DATA_MSG m = line_data_pack(LINE_ZONE_CENTRE, false, false);
    TEST_CHECK("S3-U02a",
               line_data_zone(m) == LINE_ZONE_CENTRE &&
                   line_data_junction(m) == false &&
                   line_data_lost(m) == false,
               "line_data_pack: centre, no junction, no lost");

    /* Test: zone 2 (drifting left), junction set, line-lost set */
    m = line_data_pack(2u, true, true);
    TEST_CHECK("S3-U02b",
               line_data_zone(m) == 2u &&
                   line_data_junction(m) == true &&
                   line_data_lost(m) == true,
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
               gp28_adc(s) == 4095u &&
                   gp28_slot(s) == 15u,
               "gp28_pack: max ADC and max slot round-trip");

    /* Test: zero ADC, slot 0 */
    s = gp28_pack(0u, 0u);
    TEST_CHECK("S3-U03b",
               gp28_adc(s) == 0u &&
                   gp28_slot(s) == 0u,
               "gp28_pack: zero ADC and zero slot round-trip");

    /* Test: mid-range ADC (2048), slot 7 */
    s = gp28_pack(2048u, 7u);
    TEST_CHECK("S3-U03c",
               gp28_adc(s) == 2048u &&
                   gp28_slot(s) == 7u,
               "gp28_pack: mid ADC and mid slot round-trip");
}

// char_map validation is tested by checking that valid maps are accepted and invalid maps are rejected
static bool validate_char_map(const CHAR_MAP_MSG *m)
{
    return (m->ch_forward >= 'A' && m->ch_forward <= 'Z') &&
           (m->ch_uturn >= 'A' && m->ch_uturn <= 'Z') &&
           (m->ch_left >= 'A' && m->ch_left <= 'Z') &&
           (m->ch_right >= 'A' && m->ch_right <= 'Z') &&
           (m->ch_forward != m->ch_uturn) &&
           (m->ch_forward != m->ch_left) &&
           (m->ch_forward != m->ch_right) &&
           (m->ch_uturn != m->ch_left) &&
           (m->ch_uturn != m->ch_right) &&
           (m->ch_left != m->ch_right);
}

static void test_char_map_validation(void)
{
    /* Valid mapping: four distinct letters */
    CHAR_MAP_MSG good = {'A', 'B', 'C', 'D'};
    TEST_CHECK("S3-U04a", validate_char_map(&good),
               "char_map: four distinct A–Z letters accepted");

    /* Invalid: duplicate letter (forward == left) */
    CHAR_MAP_MSG dup = {'A', 'B', 'A', 'D'};
    TEST_CHECK("S3-U04b", !validate_char_map(&dup),
               "char_map: duplicate letter rejected");

    /* Invalid: one letter outside A–Z */
    CHAR_MAP_MSG bad = {'A', 'B', 'C', '1'};
    TEST_CHECK("S3-U04c", !validate_char_map(&bad),
               "char_map: non-letter character rejected");
}

// S3-U05: the Code 39 lookup table (filled by code39_init() in subsystem3_init())
// maps known forward 9-bit patterns to the right characters.
// Pattern bits: bit 8 = first element (a bar), 1 = wide, 0 = narrow.
static void test_code39_lut(void)
{
    TEST_CHECK("S3-U05",
               code39_lookup(0x109u) == 'A' &&
                   code39_lookup(0x0D0u) == 'Z' &&
                   code39_lookup(0x019u) == 'D' &&
                   code39_lookup(CODE39_STAR_FWD) == '*' &&
                   code39_lookup(0x1FFu) == CODE39_INVALID,
               "code39_lookup: A, Z, D, * correct and all-wide pattern invalid");
}

// Write the 9 element widths of one Code 39 character into elems[at..at+8]
static void put_symbol(BcElem *elems, uint16_t at, uint16_t pattern9,
                       uint16_t narrow, uint16_t wide)
{
    uint16_t i;

    for (i = 0; i < 9u; i++)
    {
        /* bit 8 is the first element, bit 0 the last */
        elems[at + i].w = (pattern9 & (1u << (8u - i))) ? wide : narrow;
    }
}

// S3-U06: code39_decode_symbol() decodes a hand-built "*A*" element array,
// both in the forward direction and fully reversed (robot crossing the
// barcode the other way).
static void test_code39_decode_symbol(void)
{
    enum
    {
        NARROW = 5,
        WIDE = 12
    }; /* ratio 2.4 > 1.5 threshold */
    BcElem elems[29];
    BcElem tmp;
    uint8_t fwd, rev;
    uint16_t i;

    /* Layout: * (9) | gap (1) | A (9) | gap (1) | * (9) = 29 elements */
    put_symbol(elems, 0u, CODE39_STAR_FWD, NARROW, WIDE);
    elems[9].w = NARROW; /* inter-character gap */
    put_symbol(elems, 10u, 0x109u /* 'A' */, NARROW, WIDE);
    elems[19].w = NARROW; /* inter-character gap */
    put_symbol(elems, 20u, CODE39_STAR_FWD, NARROW, WIDE);

    fwd = code39_decode_symbol(elems, BC_MIN_NARROW_SAMPLES, BC_WIDE_RATIO_X10);

    /* Reverse in place: swap elems[0]<->elems[28], elems[1]<->elems[27], ... */
    for (i = 0; i < 29u / 2u; i++)
    {
        tmp = elems[i];
        elems[i] = elems[28u - i];
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

// gp28_flow() is tested by waiting 10 ms and then polling q_gp28_samples for a sample
static void test_gp28_flow(void)
{
    GP28_SAMPLE_MSG sample;

    /* Wait 10 ms then poll: at 2 ms per sample, several should be queued */
    tk_dly_tsk(10);
    INT rc = tk_rcv_mbf(q_gp28_samples, (void *)&sample, TMO_POL);

    TEST_CHECK("INT-02",
               rc == (INT)sizeof(sample),
               "GP28 samples flow from line_sensor_task to q_gp28_samples within 10 ms");
}
#endif /* S3_RUN_INT_TESTS */

// subsystem3_test() runs all unit and integration tests for subsystem 3
EXPORT void subsystem3_test(void)
{
    test_pass_ = 0;
    test_fail_ = 0;

    /* Unit tests — no timing dependency */
    test_queues_created();
    test_line_data_pack();
    test_gp28_pack();
    test_char_map_validation();
    test_code39_lut();           /* S3-U05 (Commit 2) */
    test_code39_decode_symbol(); /* S3-U06 (Commit 2) */

#if S3_RUN_INT_TESTS
    /* Integration tests — require tasks to be running and calibrated.
       Allow 500 ms for calibration to complete before checking flows. */
    tk_dly_tsk(500);
    test_line_data_flow();
    test_gp28_flow();
#endif

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */