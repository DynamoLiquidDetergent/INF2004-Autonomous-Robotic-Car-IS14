# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | all five S3 queue IDs are valid (> 0) after `subsystem3_init()` | IF-05, IF-06, IF-07, MQTT map | auto | `subsystem3_test.c` → `test_queues_created()` | `[TEST] S3-U01 PASS` |
| S3-U02a | `line_data_pack`: zone 4 (centre), no junction, no lost round-trips | IF-05 `LINE_DATA_MSG` | auto | `subsystem3_test.c` → `test_line_data_pack()` | `[TEST] S3-U02a PASS` |
| S3-U02b | `line_data_pack`: zone 2, junction + lost both set round-trips | IF-05 `LINE_DATA_MSG` | auto | `subsystem3_test.c` → `test_line_data_pack()` | `[TEST] S3-U02b PASS` |
| S3-U02c | `line_data_pack`: max zone 7 stored correctly | IF-05 `LINE_DATA_MSG` | auto | `subsystem3_test.c` → `test_line_data_pack()` | `[TEST] S3-U02c PASS` |
| S3-U03a | `gp28_pack`: max ADC 4095 + max slot 15 round-trip | IF-06 `GP28_SAMPLE_MSG` | auto | `subsystem3_test.c` → `test_gp28_pack()` | `[TEST] S3-U03a PASS` |
| S3-U03b | `gp28_pack`: ADC 0 + slot 0 round-trip | IF-06 `GP28_SAMPLE_MSG` | auto | `subsystem3_test.c` → `test_gp28_pack()` | `[TEST] S3-U03b PASS` |
| S3-U03c | `gp28_pack`: mid ADC 2048 + slot 7 round-trip | IF-06 `GP28_SAMPLE_MSG` | auto | `subsystem3_test.c` → `test_gp28_pack()` | `[TEST] S3-U03c PASS` |
| S3-U04a | char map: four distinct A–Z letters accepted | MQTT map `CHAR_MAP_MSG` | auto | `subsystem3_test.c` → `test_char_map_validation()` | `[TEST] S3-U04a PASS` |
| S3-U04b | char map: duplicate letter rejected | MQTT map `CHAR_MAP_MSG` | auto | `subsystem3_test.c` → `test_char_map_validation()` | `[TEST] S3-U04b PASS` |
| S3-U04c | char map: non-letter character rejected | MQTT map `CHAR_MAP_MSG` | auto | `subsystem3_test.c` → `test_char_map_validation()` | `[TEST] S3-U04c PASS` |
| S3-U05 | Code 39 LUT: `code39_lookup()` returns 'A' (0x109), 'Z' (0x0D0), '*' (0x094), 'D' (0x019), `CODE39_INVALID` (0x1FF) | Code 39 decoder (`code39.c`) | auto | `subsystem3_test.c`, after `code39_init()` in `subsystem3_init()` | `[TEST] S3-U05 PASS` |
| S3-U06 | `code39_decode_symbol()` decodes a hand-built `*A*` element array (narrow 5, wide 12 samples) as 'A' forward and reversed | Code 39 decoder (`code39.c`) | auto | `subsystem3_test.c`, `code39_decode_symbol(elems, 4, 15)` | `[TEST] S3-U06 PASS` |
| INT-01 | `line_sensor_task` delivers a `LINE_DATA_MSG` on `q_line_data` within 100 ms | IF-05 | auto | `subsystem3_test.c` → `test_line_data_flow()`, after `subsystem3_run_calibration()` | `[TEST] INT-01 PASS` |
| INT-02 | `line_sensor_task` puts GP28 samples into `q_gp28_samples` within 10 ms | IF-06 | auto | `subsystem3_test.c` → `test_gp28_flow()`: counts successful sends (`line_sensor_gp28_sent()`), because `barcode_decode_task` empties the queue at once | `[TEST] INT-02 PASS` |
| S3-U07 | valid char map ('A','B','C','D') + injected GP28 samples for a full `*A*` symbol produce `BARCODE_CMD_FORWARD` on `q_barcode_cmd` within 200 ms | IF-06 → IF-07, MQTT map | auto | `subsystem3_test.c`: send map to `mbx_char_map`, pause live GP28, inject quiet zone + `*A*` + quiet zone into `q_gp28_samples` | `[TEST] S3-U07 PASS` |
| S3-U08 | invalid char map (duplicate 'A') is dropped: nothing appears on `q_barcode_cmd` | MQTT map, IF-07 | auto | `subsystem3_test.c`: send bad map, wait 50 ms, poll `q_barcode_cmd` with `TMO_POL` | `[TEST] S3-U08 PASS` |
| S3-M01 | live scan: printed `*A*` and `*C*` pass under GP28 forward and reverse; correct letter and command printed | IF-06 → IF-07, Code 39 bidirectional | manual (video) | debug build with `BARCODE:` prints and test map A,B,C,D; robot pushed by hand over each barcode, both directions | `BARCODE: ch=A cmd=1` both ways; `ch=C cmd=3` |
| S3-M02 | driving scan: robot drives straight at 100 mm/s over the 141 mm printed `*A*` without stopping; letter decoded once and printed, robot keeps going | IF-06 → IF-07, Code 39 (wide-bar decoder) | manual | normal build (`make -j8`), `S3_BC_DRIVE_DEMO 1`; robot placed ≥ 10 cm before the barcode, square to the bars; read the serial monitor | `BARCODE: ch=A cmd=1` then `DEMO: barcode #1 read ... still driving`; exactly one read per pass |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | S3-U01 | 0f29374 | Daniel | PASS | `logs/2026-10-28_S3U01-U04.txt` | |
| 2026-10-28 | S3-U02 | 0f29374 | Daniel | PASS | `logs/2026-10-28_S3U01-U04.txt` | |
| 2026-10-28 | S3-U03 | 0f29374 | Daniel | PASS | `logs/2026-10-28_S3U01-U04.txt` | |
| 2026-10-28 | S3-U04 | 0f29374 | Daniel | PASS | `logs/2026-10-28_S3U01-U04.txt` | |
| 2026-10-28 | S3-U05 | 03f60e1 | Daniel | PASS | `logs/2026-10-28_S3U05-U06.txt` | |
| 2026-10-28 | S3-U06 | 03f60e1 | Daniel | PASS | `logs/2026-10-28_S3U05-U06.txt` | |
| 2026-10-28 | S3-INT-01 | f6905f8 | Daniel | PASS | `logs/2026-10-28_S3U05-U06.txt` | |
| 2026-10-28 | S3-INT-02 | f6905f8 | Daniel | PASS | `logs/2026-10-28_S3U05-U06.txt` | |
| 2026-10-29 | S3-U01 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | regression |
| 2026-10-29 | S3-U02 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | regression (a/b/c) |
| 2026-10-29 | S3-U03 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | regression (a/b/c) |
| 2026-10-29 | S3-U04 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | regression (a/b/c); now tests the real `s3_char_map_valid()` |
| 2026-10-29 | S3-U05 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | regression |
| 2026-10-29 | S3-U06 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | regression |
| 2026-10-29 | INT-01 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | calibration timed out (left/centre sensors unplugged), default range kept |
| 2026-10-29 | INT-02 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | after fix (counts sends) |
| 2026-10-29 | S3-U07 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | |
| 2026-10-29 | S3-U08 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | |
| 2026-10-30 | S3-M02 |  | Daniel | PASS | `logs/2026-10-30_S3M02.txt` | read in reverse direction; fit 0.13, ratio 1.99, beat U by 8 (min 6) -- thin margin, repeat runs |
