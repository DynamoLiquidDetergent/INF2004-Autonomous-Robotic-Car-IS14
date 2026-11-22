# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Detect a barcode while line following (demo item 1) | demo rubric 1, S3 barcode, IF-18 | manual | 1. Flash the integrated image (`S5_PHASE 7`, WiFi + MQTT). 2. Place a Code 39 track barcode (`tools/barcodes/IS14_test_barcodes.pdf`) on the line before a junction. 3. RESUME from the dashboard and let the car drive over it | console `BARCODE:` line with the letter; dashboard barcode count goes up |
| SYS-02 | Decode the barcode and get the code (demo item 2) | demo rubric 2, S3, IF-18 | manual | as SYS-01, with letters A and Z and the letter map sent from the dashboard while READY | the decoded letter is the one printed, and it is mapped to the command set on the dashboard (`BARCODE: mapping F U L R`) |
| SYS-03 | Measure the hump height (demo item 3) | demo rubric 3, S4, IF-17 | manual | drive the right wheel over the reference hump (23 mm, measured) while line following | S4 prints `height_valid=1` and `height_mm` within +/-10 % of the reference |
| SYS-04 | Follow the line (demo item 4) | demo rubric 4, S3 + Vehicle Controller | manual | run on the track for at least 30 s, with rounded corners and junction marks | the car stays on the line, no stop or event it should not have |
| SYS-05 | Follow the barcode's instruction at the next junction (demo item 5) | demo rubric 5, Vehicle Controller, IF-18 | manual | barcode before a junction, mapped in turn to LEFT, RIGHT, FORWARD and U-TURN | the car takes the commanded branch and follows the line on |
| SYS-06 | Detect an obstacle in the path (demo item 6) | demo rubric 6, S5 | manual | box 10 cm wide on or near the line about 80 cm ahead of the start | the car stops before the box (scan starts at 30 cm), `obstacle: PROFILE ... detected 1` |
| SYS-07 | Measure the obstacle width (demo item 7) | demo rubric 7, S5 | manual | as SYS-06; note the PROFILE width of the close scan | PROFILE width close to the real width (10 cm box) |
| SYS-08 | Go round the obstacle and rejoin the line (demo item 8) | demo rubric 8, S5 + Vehicle Controller | manual | as SYS-06; box offset to one side of the line and the other | `vc: back on the line`, `bypass done in N ms`, mission RUNNING, box not touched |
| SYS-09 | Telemetry and commands over MQTT (demo item 9) | demo rubric 9, S1, IF-14, IF-15 | manual | integrated WiFi image, dashboard on the laptop, hotspot. RESUME / PAUSE, switch the hotspot off and on again | dashboard connected and showing the car's data; commands work; after a link loss the car stays safe and the link comes back by itself |
| SYS-10 | Stop the motors when lifted (demo item 10, ends the run) | demo rubric 10, S4 + S5 handling | manual | lift the car about 10 cm while it drives | `handling: PICKED UP`, `vc: PICKED UP: motors stopped`, mission ABORTED, wheels stay stopped when put down |
| SYS-11 | Full demo course in one run: start, line, barcode and junction turn, hump, obstacle bypass, lift at the end | all demo items | manual | one run of the whole course on the final image, filmed | every item of SYS-01..SYS-10 in one run, no FAULT |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Results taken from the subsystem test logs (the runs named in each row) on the integrated image; one row per demo item.

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-22 | SYS-01 | `9240c6c` | Min Han | PASS (5 of 6 driven runs read the barcode on this image) | `../subsystem5/logs/2026-11-22_S5-M42_run8_A_read_left_turn.txt`, run10, run12, run13, run14; FAIL run 9 (`../subsystem5/logs/2026-11-22_S5-M42_run9_A_not_read_car_jolted.txt`) | From S5-M42 (subsystem 5 TEST_LOG). Letter A read in runs 8 and 10, Z in runs 12, 13 and 14; A missed once (run 9). Runs 1..7 were before the decoder fixes in `9240c6c` (S3-H01..H06). Same decoder in the later `737713a` image |
| 2026-11-22 | SYS-02 | `9240c6c` | Min Han | PASS (A and Z decoded and mapped) | as SYS-01 | Only A and Z are printed. Map sent from the dashboard while READY; each of the four commands reached the controller from a decoded letter (see SYS-05) |
| 2026-11-22 | SYS-03 | `737713a` | Min Han | PASS for measuring (3 of 3 crossings gave a valid height); accuracy 1 of 3 within +/-10 % | `../subsystem4/logs/2026-11-22_S4-H10_trials3-5_integrated_after_fix.txt` | From S4-H10 trials 3..5: 28, 23, 27 mm against 23 mm (mean 26 mm, readings high). Trial 2 (`2f8fa45`) dropped the height as NOT_FORWARD; fixed by S5-H141 before trials 3..5. The 5.5 mm bias is left as is (decision 22 Nov) |
| 2026-11-21 | SYS-04 | `d6249b1` | Min Han | PASS | `../subsystem5/logs/2026-11-21_S5-M40_run1_normal_run_then_lift.txt` | From S5-M40 step 1: about 32 s and 2.8 m of line following with rounded corners and two junction marks, no false event. The line was also followed before and after every junction in S5-M42 (`9240c6c`) and after the bypass in S5-M33 run 21 (`737713a`) |
| 2026-11-22 | SYS-05 | `9240c6c` | Min Han | PASS (all four commands) | `../subsystem5/logs/2026-11-22_S5-M42_run8_A_read_left_turn.txt` (LEFT), `../subsystem5/logs/2026-11-22_S5-M42_run10_A_as_uturn.txt` (U-TURN), `../subsystem5/logs/2026-11-22_S5-M42_run13_Z_as_forward.txt` (FORWARD), `../subsystem5/logs/2026-11-22_S5-M42_run14_Z_as_right.txt` (RIGHT) | From S5-M42. Commands set by remapping the letters on the dashboard, since only A and Z are printed |
| 2026-11-22 | SYS-06 | `737713a` | Min Han | PASS | `../subsystem5/logs/2026-11-22_S5-M33_run21_box_right_went_right.txt`; video (run 22): https://youtu.be/LF7pgxlZWmA | From S5-M33 run 21: held at 30 cm, first scan `PROFILE seq 1 detected 1 closest 29 cm`, creep to 17 cm, second scan `detected 1 closest 17 cm` |
| 2026-11-22 | SYS-07 | `737713a` | Min Han | PASS (9 cm (+/-4) for a 10 cm box) | `../subsystem5/logs/2026-11-22_S5-M33_run21_box_right_went_right.txt`; video (run 22): https://youtu.be/LF7pgxlZWmA | From S5-M33 run 21, close scan PROFILE seq 2. One run on this image; earlier images over-read a square-on box (S5-M38) |
| 2026-11-22 | SYS-08 | `737713a` | Min Han | PASS (2 of 2: round the right in run 21, round the left in run 22) | `../subsystem5/logs/2026-11-22_S5-M33_run21_box_right_went_right.txt`; run 22 video: https://youtu.be/LF7pgxlZWmA | From S5-M33 runs 21 and 22: back on the line both times. Box 3 cm right of the line went right (inside the sensor's left/right error), 6 cm right went left. A box next to a corner (S5-M37) was last run on 17 Nov with mixed results and is not retested on this image |
| 2026-11-22 | SYS-09 | `737713a` | Min Han | PASS for the link, commands and link loss | `../subsystem5/logs/2026-11-22_S5-M43_run1_dashboard_commands_wifi_loss.txt`; video (link to be added) | From S5-M43 run 1: RESUME / PAUSE from the dashboard, hotspot off (`s1: link down, buffering telemetry`, car stays PAUSED, no fault), hotspot on (`mqtt: connected`, `s1: link up, 20 frame(s) to replay`), commands work again. The dashboard showed the car's data during the barcode runs (barcode count, S5-M42); a field-by-field check of the telemetry while driving was not recorded (decided not needed, 22 Nov) |
| 2026-11-21 | SYS-10 | `d6249b1` | Min Han | PASS | `../subsystem5/logs/2026-11-21_S5-M40_run1_normal_run_then_lift.txt` | From S5-M40 step 3: lifted while driving, `handling: PICKED UP`, `vc: PICKED UP: motors stopped`, ABORTED, stayed stopped when put down. On `737713a` the pick-up also ended S5-M33 run 21 (after a PAUSE, not while driving) |
| 2026-11-22 | SYS-11 | `737713a` | Min Han | NOT RUN | - | Not run before the demo: the only floor available makes the car shake a lot, so a full course run there would not be representative. Each item was shown separately (SYS-01..SYS-10); the full course is first run at the demo on 23 Nov |
