# Subsystem 3 — test log

**Subsystem:** IR Perception: Line Tracking, Junction Detection, Barcode Decode  ·  **Student ID:** 2500172  ·  **Code:** `app_program/subsystem3/`

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
| S3-INT-01 | `line_sensor_task` delivers a `LINE_DATA_MSG` on `q_line_data` within 100 ms | IF-05 | auto | `subsystem3_test.c` → `test_line_data_flow()`, after `subsystem3_run_calibration()` | `[TEST] INT-01 PASS` |
| S3-INT-02 | `line_sensor_task` puts GP28 samples into `q_gp28_samples` within 10 ms | IF-06 | auto | `subsystem3_test.c` → `test_gp28_flow()`: counts successful sends (`line_sensor_gp28_sent()`), because `barcode_decode_task` empties the queue at once | `[TEST] INT-02 PASS` |
| S3-U07 | valid char map ('A','B','C','D') + injected GP28 samples for a full `*A*` symbol produce `BARCODE_CMD_FORWARD` on `q_barcode_cmd` within 200 ms | IF-06 → IF-07, MQTT map | auto | `subsystem3_test.c`: send map to `mbx_char_map`, pause live GP28, inject quiet zone + `*A*` + quiet zone into `q_gp28_samples` | `[TEST] S3-U07 PASS` |
| S3-U08 | invalid char map (duplicate 'A') is dropped: nothing appears on `q_barcode_cmd` | MQTT map, IF-07 | auto | `subsystem3_test.c`: send bad map, wait 50 ms, poll `q_barcode_cmd` with `TMO_POL` | `[TEST] S3-U08 PASS` |
| S3-M01 | live scan: printed `*A*` and `*C*` pass under GP28 forward and reverse; correct letter and command printed | IF-06 → IF-07, Code 39 bidirectional | manual (video) | debug build with `BARCODE:` prints and test map A,B,C,D; robot pushed by hand over each barcode, both directions | `BARCODE: ch=A cmd=1` both ways; `ch=C cmd=3` |
| S3-I01 | `line_sensor_task` delivers a `LINE_DATA_MSG` on `q_line_data` within 100 ms (internal to S3; was INT-01) | IF-05 | auto | `subsystem3_test.c` → `test_line_data_flow()`, after `subsystem3_run_calibration()` | `[TEST] S3-I01 PASS` |
| S3-I02 | `line_sensor_task` puts GP28 samples into `q_gp28_samples` within 10 ms, counted with `line_sensor_gp28_sent()` (internal to S3; was INT-02) | IF-06 | auto | `subsystem3_test.c` → `test_gp28_flow()` | `[TEST] S3-I02 PASS` |
| S3-M02 | driving scan: robot drives straight at 100 mm/s over the 141 mm printed `*A*` without stopping; letter decoded once and printed, robot keeps going | IF-06 → IF-07, Code 39 (wide-bar decoder) | manual | normal build (`make -j8`), `S3_BC_DRIVE_DEMO 1`; robot placed ≥ 10 cm before the barcode, square to the bars; read the serial monitor | `BARCODE: ch=A cmd=1` then `DEMO: barcode #1 read ... still driving`; exactly one read per pass |
| S3-M03 | driving scan over two barcodes in a row (`*Z*` and `*A*`, 141 mm prints, ≥ 10 cm apart) with the wide-bar decoder restricted to the 4 mapped letters; each letter decoded once with the right command, robot keeps going | IF-06 → IF-07, MQTT map, Code 39 (wide-bar decoder, mapped letters only) | manual | normal build (`make -j8`), `S3_BC_DRIVE_DEMO 1`, test map `'A','Z','C','D'` in `barcode_decode_task`; barcodes on plain white in a straight line; robot ≥ 10 cm before the first, square to the bars; read the serial monitor. Run once Z-then-A and once A-then-Z, so each barcode is read in both directions | `BARCODE: ch=Z cmd=2` and `BARCODE: ch=A cmd=1`, each followed by `DEMO: barcode #n read ... still driving`; exactly one read per barcode; no `unsure` lines |
| S3-U09a | `s3_edge_zone()`: zones 2..7 from the analog left/centre levels; zone 2 still counts as on the line | IF-05 zone | auto | `subsystem3_test.c` → `test_edge_zone()` | `[TEST] S3-U09a PASS` |
| S3-U09b | `s3_edge_zone()`: line lost → zone 1 or 7 from the previous zone, then held | IF-05 zone, lost | auto | `subsystem3_test.c` → `test_edge_zone()` | `[TEST] S3-U09b PASS` |
| S3-U10a | junction: crossbar (both full ~25 mm + GP28 dark) → zone held at 4, exactly one flag pulse after it | IF-05 junction | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10a PASS` |
| S3-U10b | junction: 4 mm both-full (line swept across by a correction) → no flag | IF-05 junction | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10b PASS` |
| S3-U10c | junction: 100 mm both-full (main line in a curve) → no flag, zone released | IF-05 junction | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10c PASS` |
| S3-U10d | junction: 25 mm both-full with GP28 white (line drifted under GP26) → no flag | IF-05 junction | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10d PASS` |
| S3-U10e | junction: crooked approach, GP28 meets the bar 8 mm after the run → one pulse | IF-05 junction | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10e PASS` |
| S3-U10f | junction: everything dark but robot not moving (odometry still) → no flag | IF-05 junction, IF-17 | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10f PASS` |
| S3-U10g | junction: one pulse per junction (500 ms refractory), the next junction is flagged again | IF-05 junction | auto | `subsystem3_test.c` → `test_junction()` | `[TEST] S3-U10g PASS` |
| S3-M04 | line-following demo: robot follows the black line on its own through straights and curves using the left + centre sensors (GP28 kept for barcodes), stops when the line ends | IF-05 → IF-10 | manual (video) | normal build (`make -j8`), `S3_BC_DRIVE_DEMO 1` + `S3_LINE_FOLLOW_DEMO 1`; sweep the robot across the line during the calibration window, put it down with the centre sensor on the line | follows the whole track without losing the line; stops ~1.5 s (`DEMO_LF_LOST_STOP_MS`) after the line ends |
| S3-M05 | junction flag on the real track: while line following, each `+` crossbar raises the junction flag exactly once; curves and barcodes do not | IF-05 junction, IF-17 | manual (video) | same build as S3-M04 (`S3_LINE_DEBUG` on); track with at least one `+` junction and one curve; read the serial monitor | one `S3: JUNCTION  (bar N mm, GP28 dark too)` line per crossbar; none on curves or barcodes; robot keeps following |
| S3-H01 | wide-bar decoder (`code39_wide.c`) on a real GP28 capture of the printed `*A*` (`tools/barcodes`), driven over at about 118 mm/s (S5-M42 run 4): a: map C D A B -> A; b: any of the 26 letters allowed -> A or unsure, never a wrong letter | IF-06 -> IF-07, Code 39 | auto (PC) | `sh testing/subsystem3/host/run.sh` (WSL), input `testing/subsystem3/host/cap_2026-11-22_A_118mms.txt` | `[HOST] S3-H01a PASS`, `S3-H01b PASS` |
| S3-H02 | synthetic traces like the track print (sharp spot, sigma 1.2 mm; narrow bars 15 % dark), both directions, 50 / 100 / 150 mm/s, noise +/-30: a: all 26 letters, each with a 4-letter map, never a wrong letter; b: map C D A B, every letter read | IF-06 -> IF-07, Code 39 | auto (PC) | as S3-H01 | `S3-H02a PASS`, `S3-H02b PASS` |
| S3-H03 | synthetic traces like the printed test sheet on today's GP28 (blurred spot, sigma 3.1 mm = the 8 mm edge of S3-H01; narrow bars solid black), all 26 letters, both directions, 50 / 100 / 150 mm/s: every letter read, never a wrong one | IF-06 -> IF-07, Code 39 | auto (PC) | as S3-H01 | `S3-H03 PASS` |
| S3-H04 | as S3-H03 with the car's map C D A B: every letter read | IF-06 -> IF-07, MQTT map | auto (PC) | as S3-H01 | `S3-H04 PASS` |
| S3-H05 | no letter from 20 cm of white floor, a 20 mm junction crossbar and 30 cm of black line (blurred spot, 100 mm/s) | IF-06 -> IF-07 | auto (PC) | as S3-H01 | `S3-H05 PASS` |
| S3-H06 | wide bars of uneven darkness (each bar 0..x lighter), map C D A B, both directions, 50 / 100 / 150 mm/s: a: track-print-like (sharp spot, faint narrow bars), x = 45 %; b: track-print-like under today's blur (sigma 3.1 mm), x = 20 %; c: printed sheet (solid narrow bars) under today's blur, x = 20 %: every letter read; d: blur and x = 45 %: never a wrong letter | IF-06 -> IF-07, Code 39 | auto (PC) | as S3-H01 | `S3-H06a..d PASS` |
| S3-H07 | partial Code 39 read: a symbol whose stop marker faded out (the last bars too faint for GP28). With 24 to 28 elements: wide / narrow split at the middle of the narrowest and widest element, the narrowest wide at least 1.5x the widest narrow, start marker and letter complete and valid, every stop element read so far matching; only a letter in the dashboard map is sent. a: lab trace `*Q*` 25 of 29: Q; b: cut to 24: Q; c: 23: nothing; d: a stop element wrong: nothing; e: start marker wrong: nothing; f: wide only 1.2x narrow: nothing; g: forward `*A*` 25 of 29: A; h: all 29 elements: left to the full decoder | FR3, IF-07 | auto (PC) | `sh testing/subsystem3/host/run.sh` | `S3-H07a..h PASS` |
| S3-H08 | wide-bar fit limit `C39W_MAX_ERR_X100` raised from 35 to 50 (0.50 narrow units): the S3-H01..H06 tests run at 35, 45, 50 and 60 must give 0 wrong letters at the chosen limit | FR3, IF-07 | auto (PC) | `gcc ... -DC39W_MAX_ERR_X100=N` for N = 35, 45, 50, 60, then `sh testing/subsystem3/host/run.sh` | 0 wrong letters at 50; reads not fewer than at 35 |

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
| 2026-10-29 | INT-01 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` & video1 | calibration timed out (left/centre sensors unplugged), default range kept |
| 2026-10-29 | INT-02 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | after fix (counts sends) |
| 2026-10-29 | S3-U07 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | |
| 2026-10-29 | S3-U08 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | |
| 2026-10-29 | S3-U05 | 03f60e1 | Daniel | PASS | `logs/2026-10-28_S3U1,INT01-02.txt` | CORRECTION of 2026-10-28 S3-U05 row: the log it cites has no S3-U05 line; the PASS line is in this log |
| 2026-10-29 | S3-U06 | 03f60e1 | Daniel | PASS | `logs/2026-10-28_S3U1,INT01-02.txt` | CORRECTION of 2026-10-28 S3-U06 row: same as above |
| 2026-10-29 | S3-I01 | f6905f8 | Daniel | PASS | `logs/2026-10-28_S3U1,INT01-02.txt` | CORRECTION of 2026-10-28 S3-INT-01 row: wrong log cited; printed as `INT-01` before the rename; calibration timed out |
| 2026-10-29 | S3-I02 | f6905f8 | Daniel | PASS | `logs/2026-10-28_S3U1,INT01-02.txt` | CORRECTION of 2026-10-28 S3-INT-02 row: wrong log cited (the INT-02 line in that log is subsystem 4's test); printed as `INT-02`; old queue-polling method |
| 2026-10-29 | S3-U01 | 494b82f | Daniel | NOT SHOWN | `logs/2026-10-28_S3U07-U08.txt` | CORRECTION of 2026-10-29 S3-U01 row: no `S3-U01` line in the log (start of S3 output lost on reset; SUMMARY says 16 passed); re-run needed |
| 2026-10-29 | S3-U02 | 494b82f | Daniel | PASS | `logs/2026-10-28_S3U07-U08.txt` | CORRECTION of log path: the 2026-10-29 rows cite `2026-10-29_S3U07-U08.txt`, the file is named `2026-10-28_S3U07-U08.txt`; applies to U02–U08 rows of that run |
| 2026-10-29 | S3-I01 | 494b82f | Daniel | PASS | `logs/2026-10-28_S3U07-U08.txt` | CORRECTION of 2026-10-29 INT-01 row (ID clash + log path); printed as `INT-01`; calibration timed out, left/centre sensors unplugged |
| 2026-10-29 | S3-I02 | 494b82f | Daniel | PASS | `logs/2026-10-28_S3U07-U08.txt` | CORRECTION of 2026-10-29 INT-02 row (ID clash + log path); printed as `INT-02`; counts sends |
| 2026-10-30 | S3-M02 | ccf825a | Daniel | PASS | `logs/2026-10-30_S3M02.txt` | read in reverse direction; fit 0.13, ratio 1.99, beat U by 8 (min 6) -- thin margin, repeat runs |
| 2026-10-30 | S3-M03 | ccf825a | Daniel | PASS (half) | `logs/2026-10-30_S3M03.txt` & video2 | Z-then-A run only; It also works no matter the barcode direction but not shown in video. Able to show during live demo|
| 2026-10-29 | S3-U02 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` | CORRECTION of the log-path correction rows above: commit `9b96d0e` renamed the file to `2026-10-29_S3U07-U08.txt`, so the original 2026-10-29 rows cite the right file; applies to the U02–U08 and S3-I01/I02 rows of that run |
| 2026-10-29 | S3-I01 | 494b82f | Daniel | PASS | `logs/2026-10-29_S3U07-U08.txt` & https://youtu.be/egCM2ayKMbE | adds the video1 link to the 2026-10-29 INT-01 / S3-I01 rows |
| 2026-10-30 | S3-M03 | ccf825a | Daniel | PASS (half) | `logs/2026-10-30_S3M03.txt` & https://youtu.be/FmM1FnGhDMA| adds the video2 link to the 2026-10-30 S3-M03 row |
| 2026-10-30 | S3-M01 | ccf825a | Daniel | NOT RUN | – | replaced by S3-M02 / S3-M03: the robot drives itself over the barcode instead of being pushed by hand |
| 2026-11-04 | S3-M04 | 90e353c | Daniel | PASS | https://youtube.com/shorts/brAS69HngTo | line-following demo; debug switches on as committed in `90e353c` |
| 2026-11-22 | S3-M05 | 980df87 | Daniel | PASS | https://youtu.be/ikz_5fpIcXo | video shows `1f336f2`, the same commit before the branch history was rewritten (see `e887419`). S5's Vehicle Controller drives; S3's junction flag (IF-05) raised at the crossbar, printed as `vc: JUNCTION n passed at … mm` |
| 2026-11-22 | S3-H01..H05 | `6494e22` | Min Han | PASS (7 of 7: H01a A, H01b never wrong, H02a 127/156 read 0 wrong, H02b 24/24, H03 156/156, H04 24/24, H05 no letter) | `sh testing/subsystem3/host/run.sh` (WSL, address + undefined-behaviour checks) | New PC tests for the wide-bar decoder after S5-M42 run 4 (GP28 much blurrier than when tuned: an edge takes ~8 mm). Changed in `code39_wide.c/.h`: split at 85 % of the peak (was 40 %), centre and width from the hump top (readings >= 85 % of the peak), the 150-sample limit on that top width, and a pulse whose peak is below 80 % of the others is skipped (groups of solid narrow bars). The same tests on the decoder before the change: H01a FAIL, H03 0/156, H04 0/24, H02a 127/156 and H02b 24/24 (unchanged), H05 PASS. Firmware (`S5_PHASE 7`, WiFi + MQTT and `WIFI=none`) builds with no warnings |
| 2026-11-22 | S3-H01..H06 | `9240c6c` | Min Han | PASS (11 of 11: H01a A, H01b never wrong, H02a 128/156 read 0 wrong, H02b 24/24, H03 156/156, H04 24/24, H05 no letter, H06a 24/24, H06b 24/24, H06c 24/24, H06d 21/24 read 0 wrong) | `sh testing/subsystem3/host/run.sh` (WSL, address + undefined-behaviour checks) | Fixes two faults of `6494e22` found on the car (S5-M42 runs 6, 7): (1) the "peak below 80 % of the others" rule dropped real wide bars of uneven darkness (new S3-H06a: 2/24 with it, 24/24 with the original decoder); it is replaced by keeping up to 7 pulses and, when 6 in a row do not fit, trying the 7 without the lowest one, so an extra hump from a group of solid narrow bars is only dropped when the rest then fits a letter. (2) `RATIO_MIN` 1.8 -> 1.4: under today's blur the measured wide/narrow ratio is 1.8-1.95, at the old floor, so every read was "unsure" (S3-H06b 11/24 -> 24/24). No wrong letter in any test. Firmware (WiFi + MQTT and `WIFI=none`) builds with no warnings |
| 2026-11-23 | S3-H07 | `47dee88` | Min Han | PASS (8 of 8; S3-H01..H06 11 of 11 still pass) | `sh testing/subsystem3/host/run.sh` (WSL, address + undefined-behaviour checks) | From S5-M42 run 17 (lab, Q not read): the narrow decoder had `*`, Q and 5 stop elements clean, then GP28 fell to 635 (below dark) and it gave up after 25 of 29. Firmware (WiFi + MQTT and `WIFI=none`) builds with no warnings. To confirm on the car |
| 2026-11-23 | S3-H08 | `76f3870` | Min Han | PASS (0 wrong letters at 35, 45 and 50; at 60, H06b 1 and H06d 2 wrong letters, so 50 is the limit) | `sh testing/subsystem3/host/run.sh` (WSL) | From S5-M42 run 18 (lab, Z: 6 wide bars, best fit 0.42 against the 0.35 limit). Reads unchanged on the synthetic traces at 45 and 50 (H02a 128/156, H06d 21/24). Only the 4 mapped letters are compared on the car. S3-H01..H07 pass, firmware builds with no warnings. To confirm on the car |
