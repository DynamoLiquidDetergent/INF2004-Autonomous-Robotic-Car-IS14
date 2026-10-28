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
